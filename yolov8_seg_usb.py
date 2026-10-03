#!/usr/bin/env python3
"""Live YOLOv8 segmentation from a USB camera using TensorRT 10.x."""

import argparse
import ctypes
import time
from pathlib import Path

import cv2
import numpy as np
import tensorrt as trt
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image


DEFAULT_CLASS_NAMES = ("bridge", "car")  # class 0, class 1


class CudaRuntime:
    HOST_TO_DEVICE = 1
    DEVICE_TO_HOST = 2

    def __init__(self):
        self.lib = ctypes.CDLL("libcudart.so")
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p

    def check(self, result):
        if result != 0:
            message = self.lib.cudaGetErrorString(result).decode("utf-8")
            raise RuntimeError(f"CUDA error {result}: {message}")

    def malloc(self, size):
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size))
        return pointer

    def stream(self):
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(pointer)))
        return pointer


class SegmentationEngine:
    def __init__(self, path):
        self.cuda = CudaRuntime()
        self.stream = self.cuda.stream()
        self.buffers = {}
        try:
            logger = trt.Logger(trt.Logger.WARNING)
            with open(path, "rb") as model_file:
                self.engine = trt.Runtime(logger).deserialize_cuda_engine(model_file.read())
            if self.engine is None:
                raise RuntimeError(f"Cannot deserialize engine: {path}")
            self.context = self.engine.create_execution_context()
            self.inputs = []
            self.outputs = {}
            for index in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(index)
                shape = tuple(self.engine.get_tensor_shape(name))
                if any(size < 0 for size in shape):
                    raise ValueError(f"Dynamic tensor {name} needs an optimization profile")
                dtype = trt.nptype(self.engine.get_tensor_dtype(name))
                host = np.empty(shape, dtype=dtype)
                device = self.cuda.malloc(host.nbytes)
                self.buffers[name] = device
                self.context.set_tensor_address(name, device.value)
                if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                    self.inputs.append((name, host))
                else:
                    self.outputs[name] = host
            if len(self.inputs) != 1 or len(self.outputs) != 2:
                raise ValueError("Expected one input and two YOLOv8 segmentation outputs")
            self.input_name, self.input = self.inputs[0]
            if self.input.shape[0:2] != (1, 3):
                raise ValueError(f"Expected NCHW RGB input, got {self.input.shape}")
            self.height, self.width = self.input.shape[2:]
        except Exception:
            self.close()
            raise

    def infer(self, image):
        np.copyto(self.input, image)
        self.cuda.check(self.cuda.lib.cudaMemcpyAsync(
            self.buffers[self.input_name], self.input.ctypes.data,
            self.input.nbytes, self.cuda.HOST_TO_DEVICE, self.stream,
        ))
        if not self.context.execute_async_v3(self.stream.value):
            raise RuntimeError("TensorRT inference failed")
        for name, host in self.outputs.items():
            self.cuda.check(self.cuda.lib.cudaMemcpyAsync(
                host.ctypes.data, self.buffers[name], host.nbytes,
                self.cuda.DEVICE_TO_HOST, self.stream,
            ))
        self.cuda.check(self.cuda.lib.cudaStreamSynchronize(self.stream))
        return self.outputs

    def close(self):
        if hasattr(self, "stream"):
            self.cuda.lib.cudaStreamSynchronize(self.stream)
        for pointer in self.buffers.values():
            self.cuda.lib.cudaFree(pointer)
        self.buffers.clear()
        if hasattr(self, "stream"):
            self.cuda.lib.cudaStreamDestroy(self.stream)
            del self.stream


def letterbox(frame, width, height):
    source_height, source_width = frame.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = round(source_width * scale)
    resized_height = round(source_height * scale)
    left = round((width - resized_width) / 2 - 0.1)
    top = round((height - resized_height) / 2 - 0.1)
    resized = cv2.resize(frame, (resized_width, resized_height))
    canvas = np.full((height, width, 3), 114, dtype=np.uint8)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
    return tensor, scale, left, top


def decode(outputs, frame_shape, scale, left, top, input_size, confidence, iou,
           max_det, make_masks=True):
    prediction = next((value[0] for value in outputs.values() if value.ndim == 3), None)
    proto = next((value[0] for value in outputs.values() if value.ndim == 4), None)
    if prediction is None or proto is None:
        raise ValueError("Unexpected YOLOv8 segmentation output shapes")
    channels = prediction.shape[0]
    mask_channels = proto.shape[0]
    class_count = channels - 4 - mask_channels
    if class_count <= 0:
        raise ValueError(f"Invalid prediction shape: {prediction.shape}")

    scores = prediction[4:4 + class_count]
    class_ids = scores.argmax(axis=0)
    confidences = scores[class_ids, np.arange(scores.shape[1])]
    candidates = np.flatnonzero(confidences >= confidence)
    if not len(candidates):
        return []
    boxes = prediction[:4, candidates].T.copy()
    boxes[:, :2] -= boxes[:, 2:] / 2
    boxes[:, 0] = (boxes[:, 0] - left) / scale
    boxes[:, 1] = (boxes[:, 1] - top) / scale
    boxes[:, 2:] /= scale
    source_height, source_width = frame_shape[:2]
    boxes[:, 0] = np.clip(boxes[:, 0], 0, source_width)
    boxes[:, 1] = np.clip(boxes[:, 1], 0, source_height)
    boxes[:, 2] = np.clip(boxes[:, 2], 0, source_width - boxes[:, 0])
    boxes[:, 3] = np.clip(boxes[:, 3], 0, source_height - boxes[:, 1])

    # Offset boxes by class for class-aware OpenCV NMS.
    nms_boxes = boxes.copy()
    nms_boxes[:, :2] += class_ids[candidates, None] * max(source_width, source_height)
    kept = cv2.dnn.NMSBoxes(
        nms_boxes.tolist(), confidences[candidates].tolist(), confidence, iou,
        top_k=max_det,
    )
    if len(kept) == 0:
        return []
    detections = []
    proto_height, proto_width = proto.shape[1:]
    input_width, input_height = input_size
    flat_proto = proto.reshape(mask_channels, -1)
    for index in np.asarray(kept).reshape(-1)[:max_det]:
        candidate = candidates[index]
        x, y, width, height = boxes[index]
        x1, y1 = max(0, int(np.floor(x))), max(0, int(np.floor(y)))
        x2 = min(source_width, int(np.ceil(x + width)))
        y2 = min(source_height, int(np.ceil(y + height)))
        if x2 <= x1 or y2 <= y1:
            continue
        px1 = max(0, int(np.floor((x1 * scale + left) * proto_width / input_width)))
        py1 = max(0, int(np.floor((y1 * scale + top) * proto_height / input_height)))
        px2 = min(proto_width, int(np.ceil((x2 * scale + left) * proto_width / input_width)))
        py2 = min(proto_height, int(np.ceil((y2 * scale + top) * proto_height / input_height)))
        mask = None
        if make_masks:
            logits = prediction[4 + class_count:, candidate] @ flat_proto
            logits = logits.reshape(proto_height, proto_width)
            mask = np.zeros((y2 - y1, x2 - x1), dtype=bool)
            if px2 > px1 and py2 > py1:
                crop = logits[py1:py2, px1:px2]
                mask = cv2.resize(crop, (x2 - x1, y2 - y1), interpolation=cv2.INTER_LINEAR) > 0
        detections.append((int(class_ids[candidate]), float(confidences[candidate]),
                           (x1, y1, x2, y2), mask))
    return detections


def draw(frame, detections, class_names):
    palette = [(0, 210, 255), (80, 220, 80), (255, 160, 60),
               (255, 80, 200), (70, 90, 255), (220, 220, 60)]
    overlay = frame.copy()
    for class_id, _, (x1, y1, x2, y2), mask in detections:
        color = palette[class_id % len(palette)]
        roi = overlay[y1:y2, x1:x2]
        roi[mask] = color
    cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, dst=frame)
    for class_id, score, (x1, y1, x2, y2), _ in detections:
        color = palette[class_id % len(palette)]
        name = class_names[class_id] if class_id < len(class_names) else f"class {class_id}"
        label = f"{name} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        text_y = max(20, y1 - 6)
        cv2.putText(frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2, cv2.LINE_AA)


def strongest_raw_prediction(outputs):
    prediction = next(value[0] for value in outputs.values() if value.ndim == 3)
    proto = next(value[0] for value in outputs.values() if value.ndim == 4)
    class_scores = prediction[4:prediction.shape[0] - proto.shape[0]]
    class_id, _ = np.unravel_index(np.argmax(class_scores), class_scores.shape)
    return int(class_id), float(class_scores.max())


def format_status(detections, class_names, fps, top_k, raw_best):
    timestamp = time.strftime("%H:%M:%S")
    if not detections:
        class_id, score = raw_best
        name = class_names[class_id] if class_id < len(class_names) else f"class {class_id}"
        return (f"[{timestamp}] FPS {fps:.1f} | 检测目标 0 | "
                f"原始最高置信度 {name}: {score:.2f}")
    strongest = sorted(detections, key=lambda item: item[1], reverse=True)[:top_k]
    entries = []
    for class_id, score, _, _ in strongest:
        name = class_names[class_id] if class_id < len(class_names) else f"class {class_id}"
        entries.append(f"{name}: {score:.2f}")
    remaining = len(detections) - len(strongest)
    suffix = f" 等另外 {remaining} 个" if remaining else ""
    return (f"[{timestamp}] FPS {fps:.1f} | 检测目标 {len(detections)} | "
            + ", ".join(entries) + suffix)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, default=Path(__file__).with_name("best.engine"))
    parser.add_argument("--image-topic", default="/target/debug_image",
                        help="ROS 2 image topic published by the camera node")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=100)
    parser.add_argument("--names", nargs="*", default=DEFAULT_CLASS_NAMES,
                        help="class names in training order (default: bridge car)")
    parser.add_argument("--no-display", action="store_true", default=True, help="terminal only; no image window")
    parser.add_argument("--print-interval", type=float, default=1.0,
                        help="minimum seconds between terminal status lines")
    parser.add_argument("--print-top-k", type=int, default=5,
                        help="maximum detections printed per status line")
    parser.add_argument("--topic", default="/yolo/detections",
                        help="ROS 2 topic for class-id/confidence pairs")
    args = parser.parse_args()
    if args.print_interval <= 0 or args.print_top_k <= 0:
        parser.error("--print-interval and --print-top-k must be positive")

    rclpy.init()
    ros_node = Node("yolov8_seg_usb")
    publisher = ros_node.create_publisher(Float32MultiArray, args.topic, 10)
    bridge = CvBridge()
    try:
        engine = SegmentationEngine(args.engine)
        try:
            previous = time.perf_counter()
            last_print = previous - args.print_interval
            smoothed_fps = 0.0
            running = True

            def process_image(message):
                nonlocal previous, last_print, smoothed_fps, running
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                image, scale, left, top = letterbox(frame, engine.width, engine.height)
                outputs = engine.infer(image)
                detections = decode(outputs, frame.shape, scale, left, top,
                                    (engine.width, engine.height), args.conf,
                                    args.iou, args.max_det,
                                    make_masks=not args.no_display)
                detection_msg = Float32MultiArray()
                detection_msg.data = [value for class_id, score, _, _ in detections
                                      for value in (float(class_id), score)]
                publisher.publish(detection_msg)
                now = time.perf_counter()
                fps = 1.0 / max(now - previous, 1e-6)
                smoothed_fps = fps if smoothed_fps == 0 else 0.9 * smoothed_fps + 0.1 * fps
                previous = now
                if now - last_print >= args.print_interval:
                    print(format_status(detections, args.names, smoothed_fps,
                                        args.print_top_k,
                                        strongest_raw_prediction(outputs)), flush=True)
                    last_print = now
                if not args.no_display:
                    draw(frame, detections, args.names)
                    cv2.putText(frame, f"FPS {smoothed_fps:.1f}  Objects {len(detections)}",
                                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.imshow("YOLOv8 TensorRT Segmentation - press q to quit", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        running = False

            ros_node.create_subscription(
                Image, args.image_topic, process_image, qos_profile_sensor_data
            )
            print(f"Subscribing to {args.image_topic}", flush=True)
            while rclpy.ok() and running:
                rclpy.spin_once(ros_node, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\n已停止", flush=True)
        finally:
            engine.close()
    finally:
        cv2.destroyAllWindows()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

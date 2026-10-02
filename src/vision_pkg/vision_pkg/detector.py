from dataclasses import dataclass

import cv2
import numpy as np


BLUE_LOWER = (85, 170, 100)
BLUE_UPPER = (120, 255, 255)


@dataclass
class DetectionResult:
    mask_blue: np.ndarray
    outer_contour: np.ndarray | None
    contour_center: tuple[int, int] | None
    circle_center: tuple[int, int] | None
    reason: str


def get_contour_center(contour):
    moments = cv2.moments(contour)

    if moments["m00"] == 0:
        return None

    x_center = int(
        round(moments["m10"] / moments["m00"])
    )

    y_center = int(
        round(moments["m01"] / moments["m00"])
    )

    return x_center, y_center


def is_complete_contour(contour, image_shape, margin=5):
    height, width = image_shape[:2]

    x, y, contour_width, contour_height = cv2.boundingRect(contour)

    # 轮廓碰到图像边界
    if x <= margin or y <= margin:
        return False, "touches left/top edge"

    if (
        x + contour_width >= width - margin
        or
        y + contour_height >= height - margin
    ):
        return False, "touches right/bottom edge"

    # 面积检查
    area = cv2.contourArea(contour)

    if area < 10000:
        return False, "contour is too small"

    # fitEllipse 至少需要 5 个点
    if len(contour) < 5:
        return False, "not enough contour points"

    # 椭圆形状检查
    (_, _), (axis_a, axis_b), _ = cv2.fitEllipse(contour)

    major_axis = max(axis_a, axis_b)
    minor_axis = min(axis_a, axis_b)

    axis_ratio = (
        minor_axis / major_axis
        if major_axis > 0
        else 0
    )

    if axis_ratio < 0.45:
        return (
            False,
            f"ellipse axis ratio is too small: {axis_ratio:.3f}"
        )

    return True, "valid"


def find_inner_ellipse(contours, outer_contour):
    outer_ellipse = cv2.fitEllipse(outer_contour)

    outer_center = np.array(
        outer_ellipse[0],
        dtype=np.float64
    )

    outer_major_axis = max(
        outer_ellipse[1]
    )

    candidates = []

    for contour in contours:

        if len(contour) < 20:
            continue

        if np.array_equal(
            contour,
            outer_contour
        ):
            continue

        ellipse = cv2.fitEllipse(contour)

        major_axis = max(
            ellipse[1]
        )

        minor_axis = min(
            ellipse[1]
        )

        size_ratio = (
            major_axis
            / outer_major_axis
        )

        axis_ratio = (
            minor_axis / major_axis
            if major_axis > 0
            else 0
        )

        center_offset = (
            np.linalg.norm(
                np.array(ellipse[0])
                - outer_center
            )
            / outer_major_axis
        )

        if (
            0.52 <= size_ratio <= 0.67
            and axis_ratio > 0.55
            and center_offset < 0.12
        ):

            score = (
                abs(size_ratio - 0.60)
                + center_offset
            )

            candidates.append(
                (score, ellipse)
            )

    if not candidates:
        return None

    return min(
        candidates,
        key=lambda candidate: candidate[0]
    )[1]


def get_center_from_black_lines(
    image,
    outer_contour,
    contours
):
    # ========================================================
    # 外椭圆
    # ========================================================

    ellipse = cv2.fitEllipse(
        outer_contour
    )

    ellipse_center = np.array(
        ellipse[0],
        dtype=np.float64
    )

    axis_a, axis_b = ellipse[1]

    # ========================================================
    # 创建圆环 ROI
    # ========================================================

    line_roi = np.zeros(
        image.shape[:2],
        dtype=np.uint8
    )

    outer_ellipse = (
        (
            float(ellipse_center[0]),
            float(ellipse_center[1])
        ),
        (
            axis_a * 0.96,
            axis_b * 0.96
        ),
        float(ellipse[2])
    )

    detected_inner_ellipse = (
        find_inner_ellipse(
            contours,
            outer_contour
        )
    )

    if detected_inner_ellipse is None:

        inner_ellipse = (
            (
                float(ellipse_center[0]),
                float(ellipse_center[1])
            ),
            (
                axis_a * 0.58,
                axis_b * 0.58
            ),
            float(ellipse[2])
        )

    else:

        inner_ellipse = (
            (
                float(
                    detected_inner_ellipse[0][0]
                ),
                float(
                    detected_inner_ellipse[0][1]
                )
            ),
            (
                detected_inner_ellipse[1][0] * 1.02,
                detected_inner_ellipse[1][1] * 1.02
            ),
            float(
                detected_inner_ellipse[2]
            )
        )

    cv2.ellipse(
        line_roi,
        outer_ellipse,
        255,
        cv2.FILLED
    )

    cv2.ellipse(
        line_roi,
        inner_ellipse,
        0,
        cv2.FILLED
    )

    # ========================================================
    # 灰度 + Canny
    # ========================================================

    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY
    )

    gray = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    edges = cv2.Canny(
        gray,
        30,
        100
    )

    edges = cv2.bitwise_and(
        edges,
        line_roi
    )

    # ========================================================
    # Hough 检测
    # ========================================================

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 360,
        threshold=12,
        minLineLength=20,
        maxLineGap=25
    )

    if lines is None:
        return None, "no black lines"

    # ========================================================
    # 筛选合理直线
    # ========================================================

    line_candidates = []

    max_line_offset = max(
        50,
        min(axis_a, axis_b) * 0.14
    )

    for x1, y1, x2, y2 in lines[:, 0]:

        dx = x2 - x1
        dy = y2 - y1

        length = np.hypot(
            dx,
            dy
        )

        if length < 20:
            continue

        normal = (
            np.array(
                [-dy, dx],
                dtype=np.float64
            )
            / length
        )

        value = np.dot(
            normal,
            np.array(
                [x1, y1],
                dtype=np.float64
            )
        )

        distance_to_center = abs(
            np.dot(
                normal,
                ellipse_center
            )
            - value
        )

        if (
            distance_to_center
            < max_line_offset
        ):
            line_candidates.append(
                (
                    normal,
                    value,
                    length
                )
            )

    # ========================================================
    # 求交点
    # ========================================================

    intersections = []

    max_center_offset = max(
        45,
        min(axis_a, axis_b) * 0.16
    )

    for first_index in range(
        len(line_candidates)
    ):

        for second_index in range(
            first_index + 1,
            len(line_candidates)
        ):

            (
                first_normal,
                first_value,
                _
            ) = line_candidates[
                first_index
            ]

            (
                second_normal,
                second_value,
                _
            ) = line_candidates[
                second_index
            ]

            # 太平行
            if (
                abs(
                    np.dot(
                        first_normal,
                        second_normal
                    )
                )
                > 0.94
            ):
                continue

            matrix = np.stack(
                [
                    first_normal,
                    second_normal
                ]
            )

            try:
                point = np.linalg.solve(
                    matrix,
                    np.array(
                        [
                            first_value,
                            second_value
                        ]
                    )
                )

            except np.linalg.LinAlgError:
                continue

            if (
                np.linalg.norm(
                    point
                    - ellipse_center
                )
                < max_center_offset
            ):
                intersections.append(
                    point
                )

    if len(intersections) < 3:
        return (
            None,
            "not enough line intersections: "
            f"{len(intersections)}"
        )

    # ========================================================
    # 交点中值
    # ========================================================

    center_median = np.median(
        np.array(intersections),
        axis=0
    )

    supporting_lines = [
        line
        for line in line_candidates
        if abs(
            np.dot(
                line[0],
                center_median
            )
            - line[1]
        ) < 14
    ]

    if len(supporting_lines) < 2:
        return (
            None,
            "not enough supporting lines: "
            f"{len(supporting_lines)}"
        )

    # ========================================================
    # 加权最小二乘
    # ========================================================

    matrix = np.zeros(
        (2, 2),
        dtype=np.float64
    )

    vector = np.zeros(
        2,
        dtype=np.float64
    )

    for (
        normal,
        value,
        length
    ) in supporting_lines:

        matrix += (
            length
            * np.outer(
                normal,
                normal
            )
        )

        vector += (
            length
            * normal
            * value
        )

    if np.linalg.cond(matrix) > 100:
        return (
            None,
            "black lines are nearly parallel"
        )

    center = np.linalg.solve(
        matrix,
        vector
    )

    center = tuple(
        np.round(center).astype(int)
    )

    return (
        center,
        f"black lines: {len(supporting_lines)}"
    )


def detect_frame(image):
    hsv = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2HSV
    )

    mask_blue = cv2.inRange(
        hsv,
        BLUE_LOWER,
        BLUE_UPPER
    )

    mask_blue = cv2.erode(
        mask_blue,
        None,
        iterations=2
    )

    mask_blue = cv2.dilate(
        mask_blue,
        None,
        iterations=2
    )

    contours, _ = cv2.findContours(
        mask_blue,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=None,
            contour_center=None,
            circle_center=None,
            reason="no blue contour"
        )

    outer_contour = max(
        contours,
        key=cv2.contourArea
    )

    contour_area = cv2.contourArea(
        outer_contour
    )

    if contour_area < 10000:

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=None,
            contour_center=None,
            circle_center=None,
            reason="contour is too small"
        )

    contour_center = get_contour_center(
        outer_contour
    )

    contour_valid, reason = is_complete_contour(
        outer_contour,
        image.shape
    )

    if not contour_valid:

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=outer_contour,
            contour_center=contour_center,
            circle_center=None,
            reason=reason
        )

    circle_center, reason = get_center_from_black_lines(
        image,
        outer_contour,
        contours
    )

    return DetectionResult(
        mask_blue=mask_blue,
        outer_contour=outer_contour,
        contour_center=contour_center,
        circle_center=circle_center,
        reason=reason
    )


def draw_detection(image, detection):
    """
    Debug 图：

    黄色：
        蓝色外轮廓

    红色圆点 + 红色十字：
        contour_center

    绿色圆点 + 绿色十字：
        circle_center

    青色十字：
        图像中心

    contour_center 和 circle_center 会同时绘制。
    """

    result = image.copy()

    height, width = result.shape[:2]

    image_center = (
        width // 2,
        height // 2
    )

    # ========================================================
    # 画面中心
    # ========================================================

    cv2.drawMarker(
        result,
        image_center,
        (255, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2
    )

    # ========================================================
    # 外轮廓
    # ========================================================

    if detection.outer_contour is not None:

        cv2.drawContours(
            result,
            [detection.outer_contour],
            -1,
            (0, 255, 255),
            2
        )

    # ========================================================
    # contour_center
    #
    # 红色
    # 无论 circle_center 是否存在，都画
    # ========================================================

    if detection.contour_center is not None:

        contour_x, contour_y = (
            detection.contour_center
        )

        # 红色圆点
        cv2.circle(
            result,
            (
                contour_x,
                contour_y
            ),
            7,
            (0, 0, 255),
            -1
        )

        # 红色十字
        cv2.drawMarker(
            result,
            (
                contour_x,
                contour_y
            ),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=30,
            thickness=2
        )

        # 画面中心到 contour_center
        cv2.line(
            result,
            image_center,
            (
                contour_x,
                contour_y
            ),
            (0, 0, 255),
            1
        )

        cv2.putText(
            result,
            (
                "Contour center: "
                f"({contour_x}, {contour_y})"
            ),
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    # ========================================================
    # circle_center
    #
    # 绿色
    # 独立于 contour_center 绘制
    # ========================================================

    if detection.circle_center is not None:

        circle_x, circle_y = (
            detection.circle_center
        )

        # 绿色圆点
        cv2.circle(
            result,
            (
                circle_x,
                circle_y
            ),
            9,
            (0, 255, 0),
            -1
        )

        # 绿色十字
        cv2.drawMarker(
            result,
            (
                circle_x,
                circle_y
            ),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=36,
            thickness=2
        )

        # 画面中心到 circle_center
        cv2.line(
            result,
            image_center,
            (
                circle_x,
                circle_y
            ),
            (0, 255, 0),
            2
        )

        cv2.putText(
            result,
            (
                "Circle center: "
                f"({circle_x}, {circle_y})"
            ),
            (30, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

    # ========================================================
    # 两个点都没有
    # ========================================================

    if (
        detection.contour_center is None
        and detection.circle_center is None
    ):

        cv2.putText(
            result,
            "NO TARGET",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    # ========================================================
    # 当前检测状态
    # ========================================================

    cv2.putText(
        result,
        f"Reason: {detection.reason}",
        (30, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 0),
        2
    )

    # ========================================================
    # 蓝色像素数量
    # ========================================================

    blue_pixels = cv2.countNonZero(
        detection.mask_blue
    )

    cv2.putText(
        result,
        f"Blue pixels: {blue_pixels}",
        (30, 140),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 255, 0),
        2
    )

    # ========================================================
    # 图像尺寸
    # ========================================================

    cv2.putText(
        result,
        f"Image: {width} x {height}",
        (30, 170),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 255, 0),
        2
    )

    # ========================================================
    # 两个中心之间的误差
    # ========================================================

    if (
        detection.contour_center is not None
        and detection.circle_center is not None
    ):

        contour_x, contour_y = detection.contour_center
        circle_x, circle_y = detection.circle_center

        dx = circle_x - contour_x
        dy = circle_y - contour_y

        distance = np.hypot(
            dx,
            dy
        )

        cv2.putText(
            result,
            (
                f"Center diff: "
                f"dx={dx}, dy={dy}, "
                f"d={distance:.1f}px"
            ),
            (30, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 0),
            2
        )

        # contour_center 到 circle_center 的连线
        cv2.line(
            result,
            (
                contour_x,
                contour_y
            ),
            (
                circle_x,
                circle_y
            ),
            (255, 0, 255),
            2
        )

    return result
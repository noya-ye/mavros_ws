from dataclasses import dataclass

import cv2
import numpy as np


# ============================================================
# HSV 阈值
# ============================================================

BLUE_LOWER = (85, 170, 100)
BLUE_UPPER = (120, 255, 255)

RED_LOWER_1 = (0, 130, 100)
RED_UPPER_1 = (10, 255, 255)

RED_LOWER_2 = (170, 130, 100)
RED_UPPER_2 = (180, 255, 255)


# ============================================================
# 检测结果
# ============================================================

@dataclass
class DetectionResult:
    mask_blue: np.ndarray

    outer_contour: np.ndarray | None

    contour_center: tuple[int, int] | None

    circle_center: tuple[int, int] | None

    reason: str

    # 红色十字中心
    red_cross_center: tuple[int, int] | None = None


# ============================================================
# 红色十字中心检测
# ============================================================

def find_symbol_center(mask, excluded_contours=None):
    """
    从 mask 中寻找符合形状条件的红色十字。

    当前判定条件：
        1. 面积 >= 500
        2. approxPolyDP 后顶点数 == 12
        3. 中心不能位于需要排除的蓝色大轮廓内部

    返回：
        (x, y)
        或 None
    """

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []

    excluded_contours = excluded_contours or []

    for contour in contours:

        # ----------------------------------------------------
        # 面积检查
        # ----------------------------------------------------

        area = cv2.contourArea(contour)

        if area < 500:
            continue

        # ----------------------------------------------------
        # 多边形近似
        # ----------------------------------------------------

        perimeter = cv2.arcLength(
            contour,
            True
        )

        approximation = cv2.approxPolyDP(
            contour,
            0.02 * perimeter,
            True
        )

        # 红十字的轮廓形状
        if len(approximation) != 12:
            continue

        # ----------------------------------------------------
        # 计算中心
        # ----------------------------------------------------

        center = get_contour_center(contour)

        if center is None:
            continue

        # ----------------------------------------------------
        # 排除蓝色大轮廓内部的目标
        # ----------------------------------------------------

        if any(
            cv2.pointPolygonTest(
                excluded_contour,
                center,
                False
            ) >= 0
            for excluded_contour in excluded_contours
        ):
            continue

        candidates.append(
            (
                area,
                center
            )
        )

    # --------------------------------------------------------
    # 没有候选
    # --------------------------------------------------------

    if not candidates:
        return None

    # --------------------------------------------------------
    # 返回面积最大的候选
    # --------------------------------------------------------

    return max(
        candidates,
        key=lambda candidate: candidate[0]
    )[1]


# ============================================================
# 轮廓中心
# ============================================================

def get_contour_center(contour):

    moments = cv2.moments(contour)

    if moments["m00"] == 0:
        return None

    x_center = int(
        round(
            moments["m10"]
            / moments["m00"]
        )
    )

    y_center = int(
        round(
            moments["m01"]
            / moments["m00"]
        )
    )

    return x_center, y_center


# ============================================================
# 检查蓝色外轮廓是否完整
# ============================================================

def is_complete_contour(
    contour,
    image_shape,
    margin=5
):

    height, width = image_shape[:2]

    x, y, contour_width, contour_height = cv2.boundingRect(
        contour
    )

    # --------------------------------------------------------
    # 碰到图像边界
    # --------------------------------------------------------

    if (
        x <= margin
        or
        y <= margin
    ):
        return (
            False,
            "touches left/top edge"
        )

    if (
        x + contour_width >= width - margin
        or
        y + contour_height >= height - margin
    ):
        return (
            False,
            "touches right/bottom edge"
        )

    # --------------------------------------------------------
    # 面积
    # --------------------------------------------------------

    area = cv2.contourArea(contour)

    if (
        area < 10000
        or
        len(contour) < 5
    ):
        return (
            False,
            "contour is too small"
        )

    # --------------------------------------------------------
    # 椭圆检查
    # --------------------------------------------------------

    (_, _), (
        axis_a,
        axis_b
    ), _ = cv2.fitEllipse(
        contour
    )

    major_axis = max(
        axis_a,
        axis_b
    )

    minor_axis = min(
        axis_a,
        axis_b
    )

    axis_ratio = (
        minor_axis / major_axis
        if major_axis > 0
        else 0
    )

    if axis_ratio < 0.45:

        return (
            False,
            f"ellipse axis ratio is too small: "
            f"{axis_ratio:.3f}"
        )

    return True, "valid"


# ============================================================
# 寻找内部椭圆
# ============================================================

def find_inner_ellipse(
    contours,
    outer_contour,
    outer_ellipse=None
):

    if outer_ellipse is None:
        outer_ellipse = cv2.fitEllipse(
            outer_contour
        )

    outer_center = np.array(
        outer_ellipse[0],
        dtype=np.float64
    )

    outer_major_axis = max(
        outer_ellipse[1]
    )

    candidates = []

    for contour in contours:

        if (
            len(contour) < 20
            or
            contour is outer_contour
        ):
            continue

        ellipse = cv2.fitEllipse(
            contour
        )

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
                np.array(
                    ellipse[0]
                )
                - outer_center
            )
            / outer_major_axis
        )

        if (
            0.52 <= size_ratio <= 0.67
            and
            axis_ratio > 0.55
            and
            center_offset < 0.12
        ):

            score = (
                abs(
                    size_ratio - 0.60
                )
                + center_offset
            )

            candidates.append(
                (
                    score,
                    ellipse
                )
            )

    if not candidates:
        return None

    return min(
        candidates,
        key=lambda candidate: candidate[0]
    )[1]


# ============================================================
# 根据圆环内部黑线计算 circle_center
# ============================================================

def get_center_from_black_lines(
    image,
    outer_contour,
    contours
):

    # ========================================================
    # 蓝色外椭圆
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
    # 只截取目标附近 ROI
    # ========================================================

    x, y, width, height = cv2.boundingRect(
        outer_contour
    )

    padding = 100

    x0 = max(
        0,
        x - padding
    )

    y0 = max(
        0,
        y - padding
    )

    x1 = min(
        image.shape[1],
        x + width + padding
    )

    y1 = min(
        image.shape[0],
        y + height + padding
    )

    origin = np.array(
        [x0, y0],
        dtype=np.float64
    )

    # 全局中心 -> ROI 局部中心

    local_center = (
        ellipse_center
        - origin
    )

    # ========================================================
    # 创建圆环 ROI
    # ========================================================

    line_roi = np.zeros(
        (
            y1 - y0,
            x1 - x0
        ),
        dtype=np.uint8
    )

    outer_ellipse = (
        (
            float(
                local_center[0]
            ),
            float(
                local_center[1]
            )
        ),
        (
            axis_a * 0.96,
            axis_b * 0.96
        ),
        float(
            ellipse[2]
        )
    )

    # ========================================================
    # 寻找内部蓝色椭圆
    # ========================================================

    detected_inner_ellipse = find_inner_ellipse(
        contours,
        outer_contour,
        ellipse
    )

    # ========================================================
    # 没找到真实内圈 -> 使用比例估计
    # ========================================================

    if detected_inner_ellipse is None:

        inner_ellipse = (
            (
                float(
                    local_center[0]
                ),
                float(
                    local_center[1]
                )
            ),
            (
                axis_a * 0.58,
                axis_b * 0.58
            ),
            float(
                ellipse[2]
            )
        )

    else:

        # 全局坐标 -> ROI 坐标

        inner_center = (
            np.array(
                detected_inner_ellipse[0],
                dtype=np.float64
            )
            - origin
        )

        inner_ellipse = (
            (
                float(
                    inner_center[0]
                ),
                float(
                    inner_center[1]
                )
            ),
            (
                detected_inner_ellipse[1][0]
                * 1.02,

                detected_inner_ellipse[1][1]
                * 1.02
            ),
            float(
                detected_inner_ellipse[2]
            )
        )

    # ========================================================
    # 画圆环区域
    # ========================================================

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
    # ROI 图像
    # ========================================================

    roi_image = image[
        y0:y1,
        x0:x1
    ]

    # ========================================================
    # 灰度
    # ========================================================

    gray = cv2.cvtColor(
        roi_image,
        cv2.COLOR_BGR2GRAY
    )

    gray = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    # ========================================================
    # Canny
    # ========================================================

    edges = cv2.Canny(
        gray,
        30,
        100
    )

    # 只保留蓝色圆环内部区域

    edges = cv2.bitwise_and(
        edges,
        line_roi
    )

    # ========================================================
    # Hough Lines
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

        return (
            None,
            "no black lines"
        )

    # ========================================================
    # 筛选合理直线
    # ========================================================

    line_candidates = []

    max_line_offset = max(
        50,
        min(
            axis_a,
            axis_b
        ) * 0.14
    )

    center_x, center_y = local_center

    for (
        line_x1,
        line_y1,
        line_x2,
        line_y2
    ) in lines[:, 0]:

        dx = (
            line_x2
            - line_x1
        )

        dy = (
            line_y2
            - line_y1
        )

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

        value = (
            normal[0] * line_x1
            +
            normal[1] * line_y1
        )

        distance_to_center = abs(
            normal[0] * center_x
            +
            normal[1] * center_y
            -
            value
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
    # 求所有合理直线的交点
    # ========================================================

    intersections = []

    max_center_offset = max(
        45,
        min(
            axis_a,
            axis_b
        ) * 0.16
    )

    for first_index in range(
        len(
            line_candidates
        )
    ):

        for second_index in range(
            first_index + 1,
            len(
                line_candidates
            )
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

            # ------------------------------------------------
            # 太平行的两条线不求交
            # ------------------------------------------------

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

            # ------------------------------------------------
            # 交点必须位于中心附近
            # ------------------------------------------------

            if (
                np.linalg.norm(
                    point
                    - local_center
                )
                < max_center_offset
            ):

                intersections.append(
                    point
                )

    # ========================================================
    # 交点数量不足
    # ========================================================

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
        np.array(
            intersections
        ),
        axis=0
    )

    # ========================================================
    # 找真正穿过中心区域的直线
    # ========================================================

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

    # ========================================================
    # 防止直线几乎平行
    # ========================================================

    if np.linalg.cond(
        matrix
    ) > 100:

        return (
            None,
            "black lines are nearly parallel"
        )

    # ========================================================
    # 求局部中心
    # ========================================================

    center = np.linalg.solve(
        matrix,
        vector
    )

    # ========================================================
    # ROI 坐标 -> 原图坐标
    # ========================================================

    center = (
        center
        + origin
    )

    center = tuple(
        np.round(
            center
        ).astype(int)
    )

    return (
        center,
        f"black lines: "
        f"{len(supporting_lines)}"
    )


# ============================================================
# 整帧检测
# ============================================================

def detect_frame(image):

    # ========================================================
    # BGR -> HSV
    # ========================================================

    hsv = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2HSV
    )

    # ========================================================
    # 蓝色 mask
    # ========================================================

    mask_blue = cv2.inRange(
        hsv,
        BLUE_LOWER,
        BLUE_UPPER
    )

    # ========================================================
    # 红色 mask
    #
    # 红色横跨 HSV Hue 的 0 度附近，所以需要两段
    # ========================================================

    mask_red = (
        cv2.inRange(
            hsv,
            RED_LOWER_1,
            RED_UPPER_1
        )
        |
        cv2.inRange(
            hsv,
            RED_LOWER_2,
            RED_UPPER_2
        )
    )

    # ========================================================
    # 蓝色 mask 去噪
    # ========================================================

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

    # ========================================================
    # 蓝色轮廓
    # ========================================================

    contours, _ = cv2.findContours(
        mask_blue,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # ========================================================
    # 用于排除红色检测中的蓝色区域
    # ========================================================

    large_blue_contours = [

        contour

        for contour in contours

        if cv2.contourArea(
            contour
        ) >= 10000

    ]

    # ========================================================
    # 红色十字检测
    # ========================================================

    red_cross_center = find_symbol_center(
        mask_red,
        large_blue_contours
    )

    # ========================================================
    # 没有蓝色轮廓
    # ========================================================

    if not contours:

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=None,
            contour_center=None,
            circle_center=None,
            reason="no blue contour",
            red_cross_center=red_cross_center
        )

    # ========================================================
    # 最大蓝色轮廓
    # ========================================================

    outer_contour = max(
        contours,
        key=cv2.contourArea
    )

    # ========================================================
    # 面积过小
    # ========================================================

    if (
        cv2.contourArea(
            outer_contour
        )
        < 10000
    ):

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=None,
            contour_center=None,
            circle_center=None,
            reason="contour is too small",
            red_cross_center=red_cross_center
        )

    # ========================================================
    # 蓝色轮廓中心
    # ========================================================

    contour_center = get_contour_center(
        outer_contour
    )

    # ========================================================
    # 蓝色轮廓是否完整
    # ========================================================

    contour_valid, reason = (
        is_complete_contour(
            outer_contour,
            image.shape
        )
    )

    if not contour_valid:

        return DetectionResult(
            mask_blue=mask_blue,
            outer_contour=outer_contour,
            contour_center=contour_center,
            circle_center=None,
            reason=reason,
            red_cross_center=red_cross_center
        )

    # ========================================================
    # 计算 circle_center
    # ========================================================

    circle_center, reason = (
        get_center_from_black_lines(
            image,
            outer_contour,
            contours
        )
    )

    # ========================================================
    # 返回结果
    # ========================================================

    return DetectionResult(
        mask_blue=mask_blue,
        outer_contour=outer_contour,
        contour_center=contour_center,
        circle_center=circle_center,
        reason=reason,
        red_cross_center=red_cross_center
    )


# ============================================================
# Debug 图
# ============================================================

def draw_detection(
    image,
    detection
):

    """
    Debug 图：

    黄色：
        蓝色外轮廓

    红色：
        contour_center

    绿色：
        circle_center

    紫色：
        red_cross_center

    青色：
        图像中心

    三个中心可以同时显示。
    """

    result = image.copy()

    height, width = result.shape[:2]

    image_center = (
        width // 2,
        height // 2
    )

    # ========================================================
    # 图像中心
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
    # 蓝色外轮廓
    # ========================================================

    if detection.outer_contour is not None:

        cv2.drawContours(
            result,
            [
                detection.outer_contour
            ],
            -1,
            (0, 255, 255),
            2
        )

    # ========================================================
    # contour_center
    # ========================================================

    if detection.contour_center is not None:

        contour_x, contour_y = (
            detection.contour_center
        )

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
    # ========================================================

    if detection.circle_center is not None:

        circle_x, circle_y = (
            detection.circle_center
        )

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
    # red_cross_center
    # ========================================================

    if (
        detection.red_cross_center
        is not None
    ):

        red_x, red_y = (
            detection.red_cross_center
        )

        # 紫色圆点
        cv2.circle(
            result,
            (
                red_x,
                red_y
            ),
            8,
            (255, 0, 255),
            -1
        )

        # 紫色十字
        cv2.drawMarker(
            result,
            (
                red_x,
                red_y
            ),
            (255, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=32,
            thickness=2
        )

        # 图像中心到红十字
        cv2.line(
            result,
            image_center,
            (
                red_x,
                red_y
            ),
            (255, 0, 255),
            2
        )

        cv2.putText(
            result,
            (
                "Red cross: "
                f"({red_x}, {red_y})"
            ),
            (30, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 255),
            2
        )

    # ========================================================
    # 所有目标都没有
    # ========================================================

    if (
        detection.contour_center is None
        and
        detection.circle_center is None
        and
        detection.red_cross_center is None
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
    # Reason
    # ========================================================

    cv2.putText(
        result,
        f"Reason: {detection.reason}",
        (30, 145),
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
        (30, 175),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 255, 0),
        2
    )

    # ========================================================
    # 图像大小
    # ========================================================

    cv2.putText(
        result,
        f"Image: {width} x {height}",
        (30, 205),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 255, 0),
        2
    )

    # ========================================================
    # contour_center 和 circle_center 差值
    # ========================================================

    if (
        detection.contour_center
        is not None
        and
        detection.circle_center
        is not None
    ):

        contour_x, contour_y = (
            detection.contour_center
        )

        circle_x, circle_y = (
            detection.circle_center
        )

        dx = (
            circle_x
            - contour_x
        )

        dy = (
            circle_y
            - contour_y
        )

        distance = np.hypot(
            dx,
            dy
        )

        cv2.putText(
            result,
            (
                "Center diff: "
                f"dx={dx}, "
                f"dy={dy}, "
                f"d={distance:.1f}px"
            ),
            (30, 235),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 0),
            2
        )

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
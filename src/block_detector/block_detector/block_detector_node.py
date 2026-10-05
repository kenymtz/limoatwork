import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


COLOR_ORDER = ["marron", "verde", "amarillo", "azul", "naranja", "rojo"]
HSV_DEFAULTS = {
    "marron": [5, 80, 25, 25, 255, 140],
    "verde": [35, 45, 35, 85, 255, 255],
    "amarillo": [20, 70, 70, 35, 255, 255],
    "azul": [90, 55, 35, 130, 255, 255],
    "naranja": [5, 90, 80, 20, 255, 255],
    "rojo": [0, 80, 50, 10, 255, 255, 170, 80, 50, 179, 255, 255],
}
SIZE_CLASSES = [
    ("bloque_3cm", 0.030),
    ("bloque_4cm", 0.040),
    ("bloque_6cm", 0.060),
    ("bloque_7_5cm", 0.075),
]
DRAW_COLORS = {
    "marron": (42, 85, 140),
    "verde": (40, 180, 60),
    "amarillo": (0, 220, 255),
    "azul": (220, 90, 30),
    "naranja": (0, 140, 255),
    "rojo": (40, 40, 230),
}


@dataclass
class BlockDetection:
    detection_id: int
    color: str
    block_type: str
    center_u: int
    center_v: int
    position_xyz: Optional[Tuple[float, float, float]]
    orientation_deg: float
    estimated_length_m: Optional[float]
    estimated_width_m: Optional[float]
    area_pixels: float
    confidence: float
    rectangularity: float
    solidity: float
    color_fill_ratio: float
    frame_id: str
    contour_pixels: List[List[int]]
    rect_points: List[List[int]]
    depth_valid_pixels: int
    depth_status: str

    def to_dict(self) -> Dict:
        position = None
        if self.position_xyz is not None:
            position = {
                "x": self.position_xyz[0],
                "y": self.position_xyz[1],
                "z": self.position_xyz[2],
            }

        return {
            "id": self.detection_id,
            "color": self.color,
            "block_type": self.block_type,
            "center_pixel": {"u": self.center_u, "v": self.center_v},
            "position_camera": position,
            "orientation_deg": self.orientation_deg,
            "estimated_length_m": self.estimated_length_m,
            "estimated_width_m": self.estimated_width_m,
            "area_pixels": self.area_pixels,
            "confidence": self.confidence,
            "rectangularity": self.rectangularity,
            "solidity": self.solidity,
            "color_fill_ratio": self.color_fill_ratio,
            "frame_id": self.frame_id,
            "contour_pixels": self.contour_pixels,
            "rect_points": self.rect_points,
            "depth_valid_pixels": self.depth_valid_pixels,
            "depth_status": self.depth_status,
        }

    @property
    def id_text(self) -> str:
        return f"#{self.detection_id}"

    @property
    def position_z(self) -> Optional[float]:
        if self.position_xyz is None:
            return None
        return self.position_xyz[2]


class BlockDetectorNode(Node):
    """Detects colored construction blocks with OpenCV and RGB-D geometry."""

    def __init__(self):
        super().__init__("block_detector")
        self.bridge = CvBridge()
        self.last_log_times: Dict[str, float] = {}
        self.last_dimensions_warning: Optional[Tuple[int, int, int, int]] = None

        self.declare_node_parameters()
        self.reload_parameters()

        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.mask_publishers = {
            color: self.create_publisher(
                Image,
                f"{self.masks_topic_prefix}/{color}",
                10,
            )
            for color in COLOR_ORDER
        }

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos,
        )
        self.color_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self.color_info_topic,
            qos_profile=qos,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.color_info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_seconds,
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(
            "Block detector iniciado: "
            f"color={self.color_topic}, depth={self.depth_topic}, "
            f"camera_info={self.color_info_topic}"
        )

    def declare_node_parameters(self):
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("debug_image_topic", "/block_detector/debug_image")
        self.declare_parameter("detections_topic", "/block_detector/detections")
        self.declare_parameter("masks_topic_prefix", "/block_detector/masks")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_seconds", 0.10)
        self.declare_parameter("min_area_pixels", 700.0)
        self.declare_parameter("max_area_pixels", 50000.0)
        self.declare_parameter("min_rect_width_pixels", 14.0)
        self.declare_parameter("min_rect_height_pixels", 10.0)
        self.declare_parameter("max_rect_width_pixels", 640.0)
        self.declare_parameter("max_rect_height_pixels", 480.0)
        self.declare_parameter("min_aspect_ratio", 0.25)
        self.declare_parameter("max_aspect_ratio", 4.5)
        self.declare_parameter("min_rectangularity", 0.60)
        self.declare_parameter("min_solidity", 0.85)
        self.declare_parameter("min_color_fill_ratio", 0.60)
        self.declare_parameter("border_margin_pixels", 6)
        self.declare_parameter("max_detections_per_frame", 10)
        self.declare_parameter("roi_x_min_norm", 0.0)
        self.declare_parameter("roi_y_min_norm", 0.0)
        self.declare_parameter("roi_x_max_norm", 1.0)
        self.declare_parameter("roi_y_max_norm", 1.0)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.50)
        self.declare_parameter("min_valid_depth_pixels", 40)
        self.declare_parameter("depth_erosion_kernel_size", 5)
        self.declare_parameter("allow_depth_resize", False)
        self.declare_parameter("morphology_kernel_size", 5)
        self.declare_parameter("morphology_open_iterations", 2)
        self.declare_parameter("morphology_close_iterations", 2)
        self.declare_parameter("size_class_tolerance_m", 0.010)
        self.declare_parameter("publish_masks", False)
        self.declare_parameter("draw_contour_points_in_json", True)
        self.declare_parameter("max_contour_points_in_json", 80)
        for color, ranges in HSV_DEFAULTS.items():
            self.declare_parameter(f"hsv_ranges.{color}", ranges)

    def reload_parameters(self):
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.color_info_topic = str(self.get_parameter("color_info_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.masks_topic_prefix = str(self.get_parameter("masks_topic_prefix").value)
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop_seconds = float(self.get_parameter("sync_slop_seconds").value)
        self.min_area_pixels = float(self.get_parameter("min_area_pixels").value)
        self.max_area_pixels = float(self.get_parameter("max_area_pixels").value)
        self.min_rect_width_pixels = float(
            self.get_parameter("min_rect_width_pixels").value
        )
        self.min_rect_height_pixels = float(
            self.get_parameter("min_rect_height_pixels").value
        )
        self.max_rect_width_pixels = float(
            self.get_parameter("max_rect_width_pixels").value
        )
        self.max_rect_height_pixels = float(
            self.get_parameter("max_rect_height_pixels").value
        )
        self.min_aspect_ratio = float(self.get_parameter("min_aspect_ratio").value)
        self.max_aspect_ratio = float(self.get_parameter("max_aspect_ratio").value)
        self.min_rectangularity = float(
            self.get_parameter("min_rectangularity").value
        )
        self.min_solidity = float(self.get_parameter("min_solidity").value)
        self.min_color_fill_ratio = float(
            self.get_parameter("min_color_fill_ratio").value
        )
        self.border_margin_pixels = int(self.get_parameter("border_margin_pixels").value)
        self.max_detections_per_frame = int(
            self.get_parameter("max_detections_per_frame").value
        )
        self.roi_x_min_norm = float(self.get_parameter("roi_x_min_norm").value)
        self.roi_y_min_norm = float(self.get_parameter("roi_y_min_norm").value)
        self.roi_x_max_norm = float(self.get_parameter("roi_x_max_norm").value)
        self.roi_y_max_norm = float(self.get_parameter("roi_y_max_norm").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_valid_depth_pixels = int(
            self.get_parameter("min_valid_depth_pixels").value
        )
        self.depth_erosion_kernel_size = int(
            self.get_parameter("depth_erosion_kernel_size").value
        )
        self.allow_depth_resize = bool(self.get_parameter("allow_depth_resize").value)
        self.morphology_kernel_size = int(
            self.get_parameter("morphology_kernel_size").value
        )
        self.morphology_open_iterations = int(
            self.get_parameter("morphology_open_iterations").value
        )
        self.morphology_close_iterations = int(
            self.get_parameter("morphology_close_iterations").value
        )
        self.size_class_tolerance_m = float(
            self.get_parameter("size_class_tolerance_m").value
        )
        self.publish_masks = bool(self.get_parameter("publish_masks").value)
        self.draw_contour_points_in_json = bool(
            self.get_parameter("draw_contour_points_in_json").value
        )
        self.max_contour_points_in_json = int(
            self.get_parameter("max_contour_points_in_json").value
        )
        self.hsv_ranges = self.load_hsv_ranges()

    def load_hsv_ranges(self) -> Dict[str, List[Tuple[np.ndarray, np.ndarray]]]:
        ranges_by_color = {}
        for color in COLOR_ORDER:
            raw = list(self.get_parameter(f"hsv_ranges.{color}").value)
            chunks = []
            if len(raw) % 6 != 0 or not raw:
                self.warn_throttled(
                    f"hsv_{color}",
                    f"Parametro hsv_ranges.{color} invalido: debe tener grupos de 6 enteros.",
                )
                raw = HSV_DEFAULTS[color]

            for index in range(0, len(raw), 6):
                low = np.array(raw[index : index + 3], dtype=np.uint8)
                high = np.array(raw[index + 3 : index + 6], dtype=np.uint8)
                chunks.append((low, high))
            ranges_by_color[color] = chunks
        return ranges_by_color

    def synced_callback(
        self,
        color_msg: Image,
        depth_msg: Image,
        color_info_msg: CameraInfo,
    ):
        start_time = time.monotonic()
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.warn_throttled("color_convert", f"No pude convertir imagen color: {exc}")
            return

        depth_m = self.convert_depth(depth_msg)
        if depth_m is None:
            return

        camera_model = self.extract_camera_model(color_info_msg)
        if camera_model is None:
            return

        depth_state = self.check_depth_compatibility(color_msg, depth_msg)
        hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
        detections, masks = self.detect_blocks(
            hsv,
            depth_m,
            color_msg,
            depth_msg,
            camera_model,
            depth_state,
        )
        self.publish_detections(color_msg.header.stamp, detections)
        self.publish_debug_image(color_msg, color_bgr, detections, depth_state)
        self.publish_masks_if_enabled(color_msg, masks)

        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        if elapsed_ms > 80.0:
            self.warn_throttled(
                "slow_callback",
                f"Callback de deteccion lento: {elapsed_ms:.1f} ms",
                period_sec=5.0,
            )

    def convert_depth(self, depth_msg: Image) -> Optional[np.ndarray]:
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.warn_throttled("depth_convert", f"No pude convertir profundidad: {exc}")
            return None

        if depth_msg.encoding in ("16UC1", "mono16"):
            depth_m = depth.astype(np.float32) * 0.001
        elif depth_msg.encoding == "32FC1":
            depth_m = depth.astype(np.float32)
        else:
            self.warn_throttled(
                "depth_encoding",
                "Encoding de profundidad no esperado "
                f"'{depth_msg.encoding}'. Intento interpretarlo como metros.",
            )
            depth_m = depth.astype(np.float32)

        valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        )
        depth_m[~valid] = np.nan
        return depth_m

    def extract_camera_model(
        self,
        camera_info_msg: CameraInfo,
    ) -> Optional[Tuple[float, float, float, float]]:
        fx = float(camera_info_msg.k[0])
        fy = float(camera_info_msg.k[4])
        cx = float(camera_info_msg.k[2])
        cy = float(camera_info_msg.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.warn_throttled("camera_info", "CameraInfo sin intrinsecos validos.")
            return None
        return fx, fy, cx, cy

    def check_depth_compatibility(self, color_msg: Image, depth_msg: Image) -> Dict:
        same_size = (
            color_msg.width == depth_msg.width
            and color_msg.height == depth_msg.height
        )
        same_frame = color_msg.header.frame_id == depth_msg.header.frame_id
        compatible = same_size and same_frame
        state = {
            "compatible": compatible,
            "same_size": same_size,
            "same_frame": same_frame,
            "color_size": [int(color_msg.width), int(color_msg.height)],
            "depth_size": [int(depth_msg.width), int(depth_msg.height)],
            "color_frame": color_msg.header.frame_id,
            "depth_frame": depth_msg.header.frame_id,
            "using_resized_depth": False,
        }

        if not same_size:
            dims = (
                int(color_msg.width),
                int(color_msg.height),
                int(depth_msg.width),
                int(depth_msg.height),
            )
            if dims != self.last_dimensions_warning:
                self.last_dimensions_warning = dims
                self.get_logger().warn(
                    "La profundidad no tiene la misma resolucion que color: "
                    f"color={color_msg.width}x{color_msg.height}, "
                    f"depth={depth_msg.width}x{depth_msg.height}. "
                    "Activa depth_registration/alineacion de la camara o usa "
                    "allow_depth_resize solo para calibracion aproximada."
                )

        if same_size and not same_frame:
            self.warn_throttled(
                "depth_frame",
                "Color y profundidad tienen la misma resolucion pero frames distintos: "
                f"color={color_msg.header.frame_id}, depth={depth_msg.header.frame_id}. "
                "Verifica que la profundidad este registrada al frame optico de color.",
                period_sec=10.0,
            )

        if not compatible and self.allow_depth_resize and same_frame:
            state["compatible"] = True
            state["using_resized_depth"] = True
            self.warn_throttled(
                "depth_resize",
                "Usando profundidad reescalada por parametro allow_depth_resize. "
                "La posicion XYZ sera aproximada.",
                period_sec=10.0,
            )

        return state

    def detect_blocks(
        self,
        hsv: np.ndarray,
        depth_m: np.ndarray,
        color_msg: Image,
        depth_msg: Image,
        camera_model: Tuple[float, float, float, float],
        depth_state: Dict,
    ) -> Tuple[List[BlockDetection], Dict[str, np.ndarray]]:
        detections = []
        masks = {}
        next_id = 0

        for color in COLOR_ORDER:
            mask = self.segment_color(hsv, self.hsv_ranges[color])
            masks[color] = mask
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.min_area_pixels or area > self.max_area_pixels:
                    continue

                rect = cv2.minAreaRect(contour)
                (center_u_f, center_v_f), (rect_w, rect_h), raw_angle = rect
                if not self.rect_dimensions_valid(rect_w, rect_h):
                    continue

                rect_points = cv2.boxPoints(rect).astype(np.int32)
                if self.touches_image_border(rect_points, hsv.shape[:2]):
                    continue

                aspect = max(rect_w, rect_h) / max(min(rect_w, rect_h), 1.0)
                if aspect < self.min_aspect_ratio or aspect > self.max_aspect_ratio:
                    continue

                shape_metrics = self.compute_shape_metrics(contour, rect_points, mask)
                if shape_metrics is None:
                    continue
                rectangularity, solidity, color_fill_ratio = shape_metrics
                if rectangularity < self.min_rectangularity:
                    continue
                if solidity < self.min_solidity:
                    continue
                if color_fill_ratio < self.min_color_fill_ratio:
                    continue

                center_u = int(round(center_u_f))
                center_v = int(round(center_v_f))
                if not self.center_inside_roi(center_u, center_v, hsv.shape[:2]):
                    continue

                orientation = self.normalize_orientation(raw_angle, rect_w, rect_h)
                depth_result = self.extract_block_depth(
                    contour,
                    depth_m,
                    color_msg,
                    depth_msg,
                    depth_state,
                )

                estimated_length_m = None
                estimated_width_m = None
                position_xyz = None
                block_type = "desconocido"
                frame_id = color_msg.header.frame_id

                if depth_result is not None:
                    depth_z, valid_count, depth_status = depth_result
                    position_xyz = self.pixel_to_camera(
                        center_u,
                        center_v,
                        depth_z,
                        camera_model,
                    )
                    estimated_length_m, estimated_width_m = self.estimate_real_size(
                        rect_points,
                        depth_z,
                        camera_model,
                    )
                    block_type = self.classify_size(estimated_length_m)
                else:
                    valid_count = 0
                    depth_status = self.depth_failure_reason(depth_state)

                confidence = self.compute_confidence(
                    area,
                    rectangularity,
                    solidity,
                    color_fill_ratio,
                    depth_result is not None,
                )
                detections.append(
                    BlockDetection(
                        detection_id=next_id,
                        color=color,
                        block_type=block_type,
                        center_u=center_u,
                        center_v=center_v,
                        position_xyz=position_xyz,
                        orientation_deg=orientation,
                        estimated_length_m=estimated_length_m,
                        estimated_width_m=estimated_width_m,
                        area_pixels=area,
                        confidence=confidence,
                        rectangularity=rectangularity,
                        solidity=solidity,
                        color_fill_ratio=color_fill_ratio,
                        frame_id=frame_id,
                        contour_pixels=self.contour_to_json(contour),
                        rect_points=rect_points.tolist(),
                        depth_valid_pixels=valid_count,
                        depth_status=depth_status,
                    )
                )
                next_id += 1

        detections.sort(
            key=lambda det: (det.confidence, det.area_pixels),
            reverse=True,
        )
        if self.max_detections_per_frame > 0:
            detections = detections[: self.max_detections_per_frame]
        for index, detection in enumerate(detections):
            detection.detection_id = index
        return detections, masks

    def segment_color(
        self,
        hsv: np.ndarray,
        ranges: Iterable[Tuple[np.ndarray, np.ndarray]],
    ) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))

        kernel_size = max(1, self.morphology_kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        if self.morphology_open_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
                iterations=self.morphology_open_iterations,
            )
        if self.morphology_close_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=self.morphology_close_iterations,
            )
        return mask

    def rect_dimensions_valid(self, rect_w: float, rect_h: float) -> bool:
        width = max(rect_w, rect_h)
        height = min(rect_w, rect_h)
        return (
            width >= self.min_rect_width_pixels
            and height >= self.min_rect_height_pixels
            and width <= self.max_rect_width_pixels
            and height <= self.max_rect_height_pixels
        )

    def touches_image_border(
        self,
        rect_points: np.ndarray,
        image_shape: Tuple[int, int],
    ) -> bool:
        if self.border_margin_pixels <= 0:
            return False

        height, width = image_shape
        margin = self.border_margin_pixels
        xs = rect_points[:, 0]
        ys = rect_points[:, 1]
        return bool(
            np.any(xs <= margin)
            or np.any(ys <= margin)
            or np.any(xs >= width - 1 - margin)
            or np.any(ys >= height - 1 - margin)
        )

    def center_inside_roi(
        self,
        center_u: int,
        center_v: int,
        image_shape: Tuple[int, int],
    ) -> bool:
        height, width = image_shape
        x_min = int(np.clip(self.roi_x_min_norm, 0.0, 1.0) * width)
        y_min = int(np.clip(self.roi_y_min_norm, 0.0, 1.0) * height)
        x_max = int(np.clip(self.roi_x_max_norm, 0.0, 1.0) * width)
        y_max = int(np.clip(self.roi_y_max_norm, 0.0, 1.0) * height)
        if x_max <= x_min or y_max <= y_min:
            return True
        return x_min <= center_u <= x_max and y_min <= center_v <= y_max

    def compute_shape_metrics(
        self,
        contour: np.ndarray,
        rect_points: np.ndarray,
        color_mask: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        area = float(cv2.contourArea(contour))
        rect_area = float(cv2.contourArea(rect_points))
        if area <= 0.0 or rect_area <= 0.0:
            return None

        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area <= 0.0:
            return None

        rect_mask = np.zeros(color_mask.shape[:2], dtype=np.uint8)
        cv2.fillPoly(rect_mask, [rect_points], 255)
        rect_pixels = int(cv2.countNonZero(rect_mask))
        if rect_pixels <= 0:
            return None

        color_pixels_in_rect = int(cv2.countNonZero(cv2.bitwise_and(color_mask, rect_mask)))
        rectangularity = area / rect_area
        solidity = area / hull_area
        color_fill_ratio = float(color_pixels_in_rect) / float(rect_pixels)
        return rectangularity, solidity, color_fill_ratio

    def compute_confidence(
        self,
        area: float,
        rectangularity: float,
        solidity: float,
        color_fill_ratio: float,
        has_depth: bool,
    ) -> float:
        area_score = min(area / max(self.min_area_pixels * 4.0, 1.0), 1.0)
        shape_score = 0.35 * rectangularity + 0.35 * solidity + 0.30 * color_fill_ratio
        depth_bonus = 0.10 if has_depth else 0.0
        confidence = 0.20 * area_score + 0.80 * shape_score + depth_bonus
        return float(round(min(confidence, 1.0), 3))

    def extract_block_depth(
        self,
        contour: np.ndarray,
        depth_m: np.ndarray,
        color_msg: Image,
        depth_msg: Image,
        depth_state: Dict,
    ) -> Optional[Tuple[float, int, str]]:
        if not depth_state["compatible"]:
            return None

        color_mask = np.zeros((color_msg.height, color_msg.width), dtype=np.uint8)
        cv2.drawContours(color_mask, [contour], -1, 255, thickness=cv2.FILLED)

        kernel_size = max(1, self.depth_erosion_kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        erosion_kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        color_mask = cv2.erode(color_mask, erosion_kernel, iterations=1)

        if depth_state["using_resized_depth"]:
            depth_mask = cv2.resize(
                color_mask,
                (int(depth_msg.width), int(depth_msg.height)),
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            depth_mask = color_mask

        values = depth_m[depth_mask > 0]
        values = values[np.isfinite(values)]
        valid_count = int(values.size)
        if valid_count < self.min_valid_depth_pixels:
            return None

        depth_z = float(np.median(values))
        if not math.isfinite(depth_z) or depth_z <= 0.0:
            return None

        status = "ok"
        if depth_state["using_resized_depth"]:
            status = "ok_resized_depth_approximate"
        return depth_z, valid_count, status

    def depth_failure_reason(self, depth_state: Dict) -> str:
        if not depth_state["same_size"]:
            return (
                "depth_not_aligned_resolution_mismatch:"
                f"color={depth_state['color_size']},depth={depth_state['depth_size']}"
            )
        if not depth_state["same_frame"]:
            return (
                "depth_not_aligned_frame_mismatch:"
                f"color={depth_state['color_frame']},depth={depth_state['depth_frame']}"
            )
        return "insufficient_valid_depth_pixels"

    def pixel_to_camera(
        self,
        u: int,
        v: int,
        z: float,
        camera_model: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        fx, fy, cx, cy = camera_model
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return float(x), float(y), float(z)

    def estimate_real_size(
        self,
        rect_points: np.ndarray,
        z: float,
        camera_model: Tuple[float, float, float, float],
    ) -> Tuple[float, float]:
        points_3d = [
            self.pixel_to_camera(int(point[0]), int(point[1]), z, camera_model)
            for point in rect_points
        ]
        edge_lengths = []
        for index in range(4):
            p0 = np.array(points_3d[index])
            p1 = np.array(points_3d[(index + 1) % 4])
            edge_lengths.append(float(np.linalg.norm(p1 - p0)))
        return max(edge_lengths), min(edge_lengths)

    def classify_size(self, estimated_length_m: Optional[float]) -> str:
        if estimated_length_m is None:
            return "desconocido"

        best_name = "desconocido"
        best_error = float("inf")
        for name, nominal_length in SIZE_CLASSES:
            error = abs(estimated_length_m - nominal_length)
            if error < best_error:
                best_error = error
                best_name = name

        if best_error <= self.size_class_tolerance_m:
            return best_name
        return "desconocido"

    def normalize_orientation(
        self,
        raw_angle: float,
        rect_w: float,
        rect_h: float,
    ) -> float:
        angle = float(raw_angle)
        if rect_w < rect_h:
            angle += 90.0
        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        return float(round(angle, 3))

    def contour_to_json(self, contour: np.ndarray) -> List[List[int]]:
        if not self.draw_contour_points_in_json:
            return []

        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        points = approx.reshape(-1, 2)
        if len(points) > self.max_contour_points_in_json:
            step = int(math.ceil(len(points) / self.max_contour_points_in_json))
            points = points[::step]
        return [[int(u), int(v)] for u, v in points]

    def publish_detections(self, stamp, detections: List[BlockDetection]):
        payload = {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "count": len(detections),
            "detections": [detection.to_dict() for detection in detections],
            "note": (
                "estimated_length_m is approximate and depends on aligned depth, "
                "HSV calibration, and camera intrinsics."
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(msg)

    def publish_debug_image(
        self,
        color_msg: Image,
        color_bgr: np.ndarray,
        detections: List[BlockDetection],
        depth_state: Dict,
    ):
        debug = color_bgr.copy()
        for detection in detections:
            draw_color = DRAW_COLORS.get(detection.color, (255, 255, 255))
            rect_points = np.array(detection.rect_points, dtype=np.int32)
            if rect_points.size > 0:
                cv2.polylines(debug, [rect_points], True, draw_color, 2)

            cv2.circle(debug, (detection.center_u, detection.center_v), 4, draw_color, -1)
            line_y = max(18, detection.center_v - 12)
            label = (
                f"{detection.id_text} {detection.color} {detection.block_type} "
                f"z={self.format_optional(detection.position_z)}m "
                f"ang={detection.orientation_deg:.1f}"
            )
            cv2.putText(
                debug,
                label,
                (max(0, detection.center_u - 80), line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                draw_color,
                1,
                cv2.LINE_AA,
            )

            xyz_text = self.xyz_text(detection.position_xyz)
            cv2.putText(
                debug,
                xyz_text,
                (max(0, detection.center_u - 80), line_y + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                draw_color,
                1,
                cv2.LINE_AA,
            )

        if not depth_state["compatible"]:
            warning = "Profundidad no alineada: sin XYZ confiable"
            cv2.putText(
                debug,
                warning,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        self.draw_roi(debug)

        try:
            out = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            out.header = color_msg.header
            self.debug_pub.publish(out)
        except Exception as exc:
            self.warn_throttled("debug_publish", f"No pude publicar debug image: {exc}")

    def publish_masks_if_enabled(self, color_msg: Image, masks: Dict[str, np.ndarray]):
        if not self.publish_masks:
            return

        for color, mask in masks.items():
            try:
                msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
                msg.header = color_msg.header
                self.mask_publishers[color].publish(msg)
            except Exception as exc:
                self.warn_throttled(
                    f"mask_{color}",
                    f"No pude publicar mascara {color}: {exc}",
                )

    def xyz_text(self, xyz: Optional[Tuple[float, float, float]]) -> str:
        if xyz is None:
            return "XYZ: sin profundidad"
        return f"XYZ=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"

    def draw_roi(self, debug: np.ndarray):
        height, width = debug.shape[:2]
        x_min = int(np.clip(self.roi_x_min_norm, 0.0, 1.0) * width)
        y_min = int(np.clip(self.roi_y_min_norm, 0.0, 1.0) * height)
        x_max = int(np.clip(self.roi_x_max_norm, 0.0, 1.0) * width)
        y_max = int(np.clip(self.roi_y_max_norm, 0.0, 1.0) * height)
        if x_min <= 0 and y_min <= 0 and x_max >= width and y_max >= height:
            return
        if x_max <= x_min or y_max <= y_min:
            return
        cv2.rectangle(debug, (x_min, y_min), (x_max, y_max), (255, 255, 255), 1)

    def format_optional(self, value: Optional[float]) -> str:
        if value is None:
            return "nan"
        return f"{value:.3f}"

    def warn_throttled(self, key: str, message: str, period_sec: float = 3.0):
        now = time.monotonic()
        last = self.last_log_times.get(key, 0.0)
        if now - last >= period_sec:
            self.last_log_times[key] = now
            self.get_logger().warn(message)

def main(args=None):
    rclpy.init(args=args)
    node = BlockDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

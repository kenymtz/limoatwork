import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


@dataclass
class Detection:
    x: int
    y: int
    w: int
    h: int
    area: int
    fill: float
    aspect: float
    distance_m: float
    depth_std_m: float
    confidence: float


class VisionDockingDetectorNode(Node):
    """Detects a broad frontal workstation face using depth, without moving."""

    def __init__(self):
        super().__init__("vision_docking_detector")

        self.bridge = CvBridge()
        self.color_msg = None
        self.depth_msg = None
        self.color_info = None
        self.depth_info = None
        self.last_valid_time = None
        self.stable_count = 0

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("debug_image_topic", "/dock/vision_debug_image")
        self.declare_parameter("target_topic", "/dock/vision_target")
        self.declare_parameter("target_valid_topic", "/dock/vision_target_valid")

        self.declare_parameter("min_depth_m", 0.25)
        self.declare_parameter("max_depth_m", 1.20)
        self.declare_parameter("roi_y_min", 0.40)
        self.declare_parameter("roi_y_max", 0.90)
        self.declare_parameter("min_component_area_px", 2000)
        self.declare_parameter("min_bbox_width_px", 220)
        self.declare_parameter("min_bbox_height_px", 25)
        self.declare_parameter("min_aspect_ratio", 1.6)
        self.declare_parameter("max_depth_std_m", 0.10)
        self.declare_parameter("max_center_error_norm", 0.35)
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("stable_required_frames", 3)
        self.declare_parameter("debug_rate_hz", 10.0)

        self.color_topic = self.get_parameter_value("color_topic")
        self.depth_topic = self.get_parameter_value("depth_topic")
        self.color_info_topic = self.get_parameter_value("color_info_topic")
        self.depth_info_topic = self.get_parameter_value("depth_info_topic")

        self.target_pub = self.create_publisher(
            Vector3Stamped,
            self.get_parameter_value("target_topic"),
            10,
        )
        self.valid_pub = self.create_publisher(
            Bool,
            self.get_parameter_value("target_valid_topic"),
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter_value("debug_image_topic"),
            10,
        )

        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.color_info_topic,
            self.color_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.depth_info_callback,
            qos_profile_sensor_data,
        )

        rate_hz = max(1.0, float(self.get_parameter_value("debug_rate_hz")))
        self.timer = self.create_timer(1.0 / rate_hz, self.process_once)

        self.get_logger().info(
            "Vision docking detector iniciado: "
            f"color={self.color_topic}, depth={self.depth_topic}"
        )

    def get_parameter_value(self, name):
        return self.get_parameter(name).get_parameter_value().string_value or (
            self.get_parameter(name).value
        )

    def color_callback(self, msg):
        self.color_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

    def color_info_callback(self, msg):
        self.color_info = msg

    def depth_info_callback(self, msg):
        self.depth_info = msg

    def process_once(self):
        if self.depth_msg is None:
            self.publish_valid(False)
            return

        color = self.convert_color()
        depth_m = self.convert_depth()

        if depth_m is None:
            self.publish_valid(False)
            return

        detection, mask = self.detect_workstation(depth_m)
        valid = detection is not None

        if valid:
            self.stable_count += 1
            self.last_valid_time = time.monotonic()
        else:
            self.stable_count = 0

        stable = valid and (
            self.stable_count
            >= int(self.get_parameter_value("stable_required_frames"))
        )

        self.publish_valid(stable)

        if detection is not None:
            self.publish_target(detection, depth_m.shape[1])

        self.publish_debug(color, depth_m, mask, detection, stable)

    def convert_color(self):
        if self.color_msg is None:
            return None

        try:
            return self.bridge.imgmsg_to_cv2(
                self.color_msg,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().warn(f"No pude convertir color image: {exc}")
            return None

    def convert_depth(self):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                self.depth_msg,
                desired_encoding="passthrough",
            )
        except Exception as exc:
            self.get_logger().warn(f"No pude convertir depth image: {exc}")
            return None

        depth = depth.astype(np.float32)

        if self.depth_msg.encoding in ("16UC1", "mono16"):
            depth *= 0.001

        valid = np.isfinite(depth) & (depth > 0.05) & (depth < 10.0)
        depth[~valid] = np.nan
        return depth

    def detect_workstation(self, depth_m):
        height, width = depth_m.shape[:2]
        y_min = int(height * float(self.get_parameter_value("roi_y_min")))
        y_max = int(height * float(self.get_parameter_value("roi_y_max")))
        y_min = max(0, min(height - 1, y_min))
        y_max = max(y_min + 1, min(height, y_max))

        min_depth = float(self.get_parameter_value("min_depth_m"))
        max_depth = float(self.get_parameter_value("max_depth_m"))

        mask = np.zeros((height, width), dtype=np.uint8)
        roi = depth_m[y_min:y_max, :]
        roi_mask = (
            np.isfinite(roi)
            & (roi >= min_depth)
            & (roi <= max_depth)
        )
        mask[y_min:y_max, :] = roi_mask.astype(np.uint8) * 255

        close_kernel = np.ones((17, 17), dtype=np.uint8)
        open_kernel = np.ones((7, 7), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, 1)

        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        candidates = []

        for label in range(1, n_labels):
            x, y, w, h, area = stats[label]

            if area < int(self.get_parameter_value("min_component_area_px")):
                continue
            if w < int(self.get_parameter_value("min_bbox_width_px")):
                continue
            if h < int(self.get_parameter_value("min_bbox_height_px")):
                continue

            fill = float(area) / max(float(w * h), 1.0)
            aspect = float(w) / max(float(h), 1.0)

            if aspect < float(self.get_parameter_value("min_aspect_ratio")):
                continue

            component_depth = depth_m[(labels == label) & np.isfinite(depth_m)]

            if component_depth.size < 100:
                continue

            distance = float(np.median(component_depth))
            depth_std = float(np.std(component_depth))

            if depth_std > float(self.get_parameter_value("max_depth_std_m")):
                continue

            center_error = abs((x + 0.5 * w) - 0.5 * width) / max(0.5 * width, 1.0)

            if center_error > float(self.get_parameter_value("max_center_error_norm")):
                continue

            confidence = (
                min(area / 70000.0, 1.0)
                + min(fill, 1.0)
                + min(aspect / 4.0, 1.0)
                + max(0.0, 1.0 - depth_std / 0.10)
                + max(0.0, 1.0 - center_error)
            ) / 5.0

            if confidence < float(self.get_parameter_value("min_confidence")):
                continue

            candidates.append(
                Detection(
                    x=int(x),
                    y=int(y),
                    w=int(w),
                    h=int(h),
                    area=int(area),
                    fill=fill,
                    aspect=aspect,
                    distance_m=distance,
                    depth_std_m=depth_std,
                    confidence=confidence,
                )
            )

        if not candidates:
            return None, mask

        candidates.sort(key=lambda det: det.confidence, reverse=True)
        return candidates[0], mask

    def publish_valid(self, valid):
        msg = Bool()
        msg.data = bool(valid)
        self.valid_pub.publish(msg)

    def publish_target(self, detection, image_width):
        target = Vector3Stamped()
        frame_id = self.depth_msg.header.frame_id
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = frame_id

        center_x = detection.x + 0.5 * detection.w
        fx = self.get_fx()
        cx = self.get_cx(image_width)
        angle_error = math.atan2(center_x - cx, fx)

        target.vector.x = float(angle_error)
        target.vector.y = float((center_x - cx) / max(cx, 1.0))
        target.vector.z = float(detection.distance_m)
        self.target_pub.publish(target)

    def get_fx(self):
        if self.depth_info is not None and self.depth_info.k[0] > 0.0:
            return float(self.depth_info.k[0])
        if self.color_info is not None and self.color_info.k[0] > 0.0:
            return float(self.color_info.k[0])
        return 475.0

    def get_cx(self, width):
        if self.depth_info is not None and self.depth_info.k[2] > 0.0:
            return float(self.depth_info.k[2])
        if self.color_info is not None and self.color_info.k[2] > 0.0:
            return float(self.color_info.k[2])
        return 0.5 * float(width)

    def publish_debug(self, color, depth_m, mask, detection, stable):
        if color is None:
            debug = self.depth_to_debug_image(depth_m)
        else:
            debug = color.copy()

        if debug.shape[:2] != mask.shape[:2]:
            mask_for_debug = cv2.resize(
                mask,
                (debug.shape[1], debug.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            mask_for_debug = mask

        overlay = debug.copy()
        overlay[mask_for_debug > 0] = (0, 180, 255)
        debug = cv2.addWeighted(overlay, 0.25, debug, 0.75, 0.0)

        if detection is not None:
            x1, y1, x2, y2 = self.scale_bbox_to_debug(detection, debug, depth_m)
            color_box = (0, 255, 0) if stable else (0, 200, 255)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color_box, 2)
            cv2.putText(
                debug,
                (
                    f"z={detection.distance_m:.2f}m "
                    f"std={detection.depth_std_m:.3f} "
                    f"conf={detection.confidence:.2f}"
                ),
                (max(5, x1), max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color_box,
                2,
            )

        cv2.line(
            debug,
            (debug.shape[1] // 2, 0),
            (debug.shape[1] // 2, debug.shape[0]),
            (255, 0, 0),
            1,
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = (
                self.color_msg.header.frame_id
                if self.color_msg is not None
                else self.depth_msg.header.frame_id
            )
            self.debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"No pude publicar debug image: {exc}")

    def scale_bbox_to_debug(self, detection, debug, depth_m):
        depth_h, depth_w = depth_m.shape[:2]
        debug_h, debug_w = debug.shape[:2]
        sx = float(debug_w) / max(float(depth_w), 1.0)
        sy = float(debug_h) / max(float(depth_h), 1.0)
        x1 = int(detection.x * sx)
        y1 = int(detection.y * sy)
        x2 = int((detection.x + detection.w) * sx)
        y2 = int((detection.y + detection.h) * sy)
        return x1, y1, x2, y2

    def depth_to_debug_image(self, depth_m):
        valid = np.isfinite(depth_m)
        if not valid.any():
            return np.zeros((*depth_m.shape[:2], 3), dtype=np.uint8)

        vmin, vmax = np.nanpercentile(depth_m[valid], [2, 98])
        norm = np.clip((depth_m - vmin) / max(vmax - vmin, 1e-6), 0.0, 1.0)
        norm = np.nan_to_num(norm, nan=0.0)
        image = (255 * (1.0 - norm)).astype(np.uint8)
        return cv2.applyColorMap(image, cv2.COLORMAP_JET)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDockingDetectorNode()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Vision docking detector detenido.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

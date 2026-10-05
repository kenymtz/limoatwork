#!/usr/bin/env python3
import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from limo_mission_msgs.action import Dock


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class DockServer(Node):
    """
    Docking geométrico (face reta):
    - pega pontos do scan em FOV frontal
    - estima uma linha por PCA (centro + direção)
    - calcula erros: x (distância), y (centralização), yaw (alinhamento)
    - publica cmd_vel até satisfazer tolerâncias
    """

    def __init__(self):
        super().__init__("dock_server")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self._last_scan: LaserScan | None = None

        self._as = ActionServer(
            self,
            Dock,
            "/dock",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("DockServer rodando: action /dock")

    def _scan_cb(self, msg: LaserScan):
        self._last_scan = msg

    def goal_cb(self, goal_request: Dock.Goal):
        if goal_request.timeout_sec <= 0.5:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self._stop()
        return CancelResponse.ACCEPT

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _extract_points_front(self, scan: LaserScan, fov_deg: float, max_range: float) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        fov = math.radians(fov_deg)
        rmax = min(scan.range_max, max_range)

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= scan.range_min or r >= rmax:
                continue

            ang = scan.angle_min + i * scan.angle_increment
            if ang < -fov / 2.0 or ang > fov / 2.0:
                continue

            x = r * math.cos(ang)
            y = r * math.sin(ang)

            if x <= 0.05:
                continue

            pts.append((x, y))
        return pts

    def _pca_line(self, pts: List[Tuple[float, float]]) -> Tuple[float, float, float]:
        """
        Retorna (cx, cy, line_angle) onde line_angle é direção principal da linha (rad).
        """
        n = len(pts)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx = sum(xs) / n
        cy = sum(ys) / n

        sxx = sum((x - cx) ** 2 for x in xs) / n
        syy = sum((y - cy) ** 2 for y in ys) / n
        sxy = sum((xs[i] - cx) * (ys[i] - cy) for i in range(n)) / n

        # direção principal
        line_angle = 0.5 * math.atan2(2.0 * sxy, (sxx - syy))
        return cx, cy, line_angle

    def execute_cb(self, goal_handle):
        g = goal_handle.request
        start = time.time()

        # parâmetros do goal
        target_offset_x = g.target_offset_x if g.target_offset_x > 0.0 else 0.25
        max_scan_range = g.max_scan_range if g.max_scan_range > 0.0 else 2.5
        fov_deg = g.fov_deg if g.fov_deg > 0.0 else 120.0

        kx = g.kx if g.kx > 0.0 else 0.8
        ky = g.ky if g.ky > 0.0 else 2.0
        ktheta = g.ktheta if g.ktheta > 0.0 else 1.5

        tol_x = g.tol_x if g.tol_x > 0.0 else 0.02
        tol_y = g.tol_y if g.tol_y > 0.0 else 0.015
        tol_yaw = math.radians(g.tol_yaw_deg if g.tol_yaw_deg > 0.0 else 1.0)

        v_max = g.v_max if g.v_max > 0.0 else 0.15
        w_max = g.w_max if g.w_max > 0.0 else 0.8

        timeout = g.timeout_sec

        self.get_logger().info(
            f"Dock start: offset_x={target_offset_x:.2f} fov={fov_deg:.1f} range={max_scan_range:.1f} timeout={timeout:.1f}"
        )

        rate = self.create_rate(30)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                res = Dock.Result()
                res.success = False
                res.message = "Canceled"
                return res

            if (time.time() - start) > timeout:
                self._stop()
                goal_handle.abort()
                res = Dock.Result()
                res.success = False
                res.message = "Timeout docking"
                res.final_error_x = 999.0
                res.final_error_y = 999.0
                res.final_error_yaw_deg = 999.0
                return res

            scan = self._last_scan
            if scan is None:
                self._stop()
                rate.sleep()
                continue

            pts = self._extract_points_front(scan, fov_deg=fov_deg, max_range=max_scan_range)
            if len(pts) < 30:
                self._stop()
                rate.sleep()
                continue

            cx, cy, line_angle = self._pca_line(pts)

            # normal da linha (perpendicular) -> queremos normal apontando para +x (0 rad)
            normal = line_angle + math.pi / 2.0
            normal = math.atan2(math.sin(normal), math.cos(normal))
            yaw_error = normal  # objetivo 0

            x_error = cx - target_offset_x
            y_error = cy  # objetivo 0

            fb = Dock.Feedback()
            fb.error_x = float(x_error)
            fb.error_y = float(y_error)
            fb.error_yaw_deg = float(math.degrees(yaw_error))
            goal_handle.publish_feedback(fb)

            if abs(x_error) < tol_x and abs(y_error) < tol_y and abs(yaw_error) < tol_yaw:
                self._stop()
                goal_handle.succeed()
                res = Dock.Result()
                res.success = True
                res.message = "Docked"
                res.final_error_x = float(x_error)
                res.final_error_y = float(y_error)
                res.final_error_yaw_deg = float(math.degrees(yaw_error))
                return res

            v = clamp(kx * x_error, -v_max, v_max)
            w = clamp(ky * y_error + ktheta * yaw_error, -w_max, w_max)

            # se muito perto, limita avanço
            if cx < 0.20:
                v = clamp(v, -0.05, 0.05)

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)

            rate.sleep()

        self._stop()
        goal_handle.abort()
        res = Dock.Result()
        res.success = False
        res.message = "ROS shutdown"
        return res


def main():
    rclpy.init()
    node = DockServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

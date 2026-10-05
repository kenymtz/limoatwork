import argparse
import math
import sys

import rclpy

from limo_mission.nodes.robot_node import RobotNode
from limo_mission.core.navigator import Navigator
from limo_mission.core.docking_controller import DockingController
from limo_mission.mission_manager import MissionManager
from limo_mission.config.poses import pose_dict


DEFAULT_ROUTE = ["WS01", "WS02", "WS03", "WS04", "WS05"]


def parse_args(args=None):
    raw_args = sys.argv[1:] if args is None else args

    parser = argparse.ArgumentParser(
        description='Run the LIMO mission route with Nav2 and LiDAR docking.'
    )
    parser.add_argument(
        '--route',
        nargs='+',
        default=DEFAULT_ROUTE,
        help='Ordered waypoint names to visit. Default: WS01 WS02 WS03 WS04 WS05.',
    )
    parser.add_argument(
        '--set-initial-pose',
        action='store_true',
        help='Publish the initial pose before starting the route. This is now enabled by default.',
    )
    parser.add_argument(
        '--no-initial-pose',
        action='store_true',
        help='Do not publish /initialpose before starting the route.',
    )
    parser.add_argument(
        '--initial-pose',
        nargs=3,
        type=float,
        metavar=('X', 'Y', 'YAW_DEG'),
        default=(0.0, 0.0, 0.0),
        help='Initial AMCL pose in map frame. Default: 0.0 0.0 0.0.',
    )
    parser.add_argument(
        '--localization-wait-sec',
        type=float,
        default=5.0,
        help='Seconds to wait before sending the first Nav2 goal.',
    )
    parser.add_argument(
        '--undock-duration-sec',
        type=float,
        default=1.0,
        help='Backward motion duration after docking when another waypoint remains.',
    )
    parser.add_argument(
        '--undock-speed',
        type=float,
        default=-0.08,
        help='Linear x speed used to leave the docking face before the next waypoint.',
    )

    return parser.parse_known_args(raw_args)


def main(args=None):
    parsed_args, ros_args = parse_args(args)
    rclpy.init(args=[sys.argv[0], *ros_args])

    initial_x, initial_y, initial_yaw_deg = parsed_args.initial_pose
    initial_yaw_rad = math.radians(initial_yaw_deg)
    initial_pose = (
        initial_x,
        initial_y,
        0.0,
        math.sin(initial_yaw_rad / 2.0),
        math.cos(initial_yaw_rad / 2.0),
    )

    node = RobotNode()
    navigator = Navigator(node)
    docker = DockingController(node)
    publish_initial_pose = parsed_args.set_initial_pose or not parsed_args.no_initial_pose

    manager = MissionManager(
        node,
        navigator,
        docker,
        publish_initial_pose=publish_initial_pose,
        initial_pose=initial_pose,
        localization_wait_sec=parsed_args.localization_wait_sec,
        undock_duration_sec=parsed_args.undock_duration_sec,
        undock_speed=parsed_args.undock_speed,
    )

    missing_waypoints = [name for name in parsed_args.route if name not in pose_dict]
    if missing_waypoints:
        node.get_logger().error(
            f"Waypoints no encontrados en pose_dict: {', '.join(missing_waypoints)}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"Ruta configurada: {' -> '.join(parsed_args.route)}"
    )

    for waypoint_name in parsed_args.route:
        manager.add_mission(
            pose_dict[waypoint_name],
            action='dock',
            name=waypoint_name
        )

    manager.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

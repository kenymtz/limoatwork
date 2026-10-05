import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class GoalPoseCapture(Node):
    def __init__(self, topic, waypoint_name):
        super().__init__('goal_pose_capture')
        self.topic = topic
        self.waypoint_name = waypoint_name
        self.done = False

        self.create_subscription(PoseStamped, topic, self.pose_callback, 10)
        self.get_logger().info(
            f"Waiting for PoseStamped on {topic}. Send a goal from RViz."
        )

    def pose_callback(self, msg):
        pose = msg.pose
        yaw = quaternion_to_yaw(pose.orientation)
        pose_line = (
            f'    "{self.waypoint_name}": create_pose('
            f'{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}, '
            f'{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, '
            f'{pose.orientation.z:.6f}, {pose.orientation.w:.6f}),'
        )
        yaml_lines = [
            f"  {self.waypoint_name}:",
            f"    x: {pose.position.x:.3f}",
            f"    y: {pose.position.y:.3f}",
            f"    yaw: {yaw:.3f}",
        ]

        print()
        print("# Add this entry to src/limo_mission/limo_mission/config/poses.py:")
        print(pose_line)
        print()
        print("# YAML format, if using src/limo_mission/config/waypoints.yaml:")
        print("\n".join(yaml_lines))
        print()

        self.get_logger().info(
            f"Captured {self.waypoint_name}: "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, yaw={yaw:.3f}"
        )
        self.done = True


def main(args=None):
    raw_args = sys.argv[1:] if args is None else args

    parser = argparse.ArgumentParser(
        description='Capture one RViz/Nav2 goal pose and print mission waypoint formats.'
    )
    parser.add_argument(
        '--name',
        default='WS_NEW',
        help='Waypoint name to print, for example WS04 or START_DOCK.',
    )
    parser.add_argument(
        '--topic',
        default='/goal_pose',
        help='PoseStamped topic to listen to. Use /move_base_simple/goal if RViz is configured that way.',
    )

    parsed_args, ros_args = parser.parse_known_args(raw_args)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = GoalPoseCapture(parsed_args.topic, parsed_args.name)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Capture canceled by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

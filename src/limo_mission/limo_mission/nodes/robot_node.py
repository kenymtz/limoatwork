import time

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from rclpy.qos import qos_profile_sensor_data


class RobotNode(Node):
    def __init__(self):
        super().__init__('limo_mission_node')

        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.declare_parameter('scan_topic', '/scan_filtered')
        self.scan_topic = (
            self.get_parameter('scan_topic')
            .get_parameter_value()
            .string_value
        )

        self.laser_scan = None
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.laser_callback,
            qos_profile_sensor_data
        )

        self.declare_parameter('vision_target_topic', '/dock/vision_target')
        self.declare_parameter(
            'vision_target_valid_topic',
            '/dock/vision_target_valid'
        )
        self.vision_target_topic = (
            self.get_parameter('vision_target_topic')
            .get_parameter_value()
            .string_value
        )
        self.vision_target_valid_topic = (
            self.get_parameter('vision_target_valid_topic')
            .get_parameter_value()
            .string_value
        )

        self.vision_target = None
        self.vision_target_valid = False
        self.vision_target_time = None
        self.vision_valid_sub = self.create_subscription(
            Bool,
            self.vision_target_valid_topic,
            self.vision_valid_callback,
            10
        )
        self.vision_target_sub = self.create_subscription(
            Vector3Stamped,
            self.vision_target_topic,
            self.vision_target_callback,
            10
        )

        self.get_logger().info(
            f'RobotNode iniciado. Usando LiDAR en {self.scan_topic}. '
            f'Vision target en {self.vision_target_topic}.'
        )

    def laser_callback(self, msg):
        self.laser_scan = msg

    def vision_valid_callback(self, msg):
        self.vision_target_valid = bool(msg.data)
        if not msg.data:
            self.vision_target_time = None

    def vision_target_callback(self, msg):
        self.vision_target = msg
        self.vision_target_time = time.monotonic()

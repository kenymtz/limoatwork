from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
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

        self.get_logger().info(
            f'RobotNode iniciado. Usando LiDAR en {self.scan_topic}.'
        )

    def laser_callback(self, msg):
        self.laser_scan = msg

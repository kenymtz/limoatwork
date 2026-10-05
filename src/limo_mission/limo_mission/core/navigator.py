from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped


class Navigator:
    def __init__(self, node):
        self.node = node
        self._retry_timer = None

    def publish_initial_pose(self, x=0.0, y=0.0, z=0.0, qz=0.0, qw=1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        self.node.initial_pub.publish(msg)
        self.node.get_logger().info('Initial pose publicada.')

    def send_goal(self, pose_stamped, done_cb=None, retries=3, retry_delay_sec=2.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

        self.node.get_logger().info("Esperando servidor navigate_to_pose...")
        if not self.node.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("Servidor navigate_to_pose no disponible.")
            self._retry_or_fail(pose_stamped, done_cb, retries, retry_delay_sec)
            return

        future = self.node.nav_to_pose_client.send_goal_async(goal_msg)

        def goal_response_callback(fut):
            goal_handle = fut.result()

            if not goal_handle.accepted:
                self.node.get_logger().error("Meta rechazada.")
                self._retry_or_fail(pose_stamped, done_cb, retries, retry_delay_sec)
                return

            self.node.get_logger().info("Meta aceptada.")
            result_future = goal_handle.get_result_async()

            def result_callback(res_fut):
                result = res_fut.result()
                success = (result.status == GoalStatus.STATUS_SUCCEEDED)
                if not success:
                    self.node.get_logger().error(
                        f"Navegación terminó con status {result.status}."
                    )
                if done_cb:
                    done_cb(success)

            result_future.add_done_callback(result_callback)

        future.add_done_callback(goal_response_callback)

    def _retry_or_fail(self, pose_stamped, done_cb, retries, retry_delay_sec):
        if retries <= 0:
            if done_cb:
                done_cb(False)
            return

        self.node.get_logger().warn(
            f"Reintentando meta en {retry_delay_sec:.1f}s. Intentos restantes: {retries}"
        )

        if self._retry_timer is not None:
            self._retry_timer.cancel()

        def retry_once():
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            self.send_goal(
                pose_stamped,
                done_cb,
                retries=retries - 1,
                retry_delay_sec=retry_delay_sec,
            )

        self._retry_timer = self.node.create_timer(retry_delay_sec, retry_once)

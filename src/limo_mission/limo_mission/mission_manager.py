import time

from geometry_msgs.msg import Twist

from limo_mission.core.states import MissionState


class MissionManager:
    def __init__(
        self,
        node,
        navigator,
        docker,
        publish_initial_pose=False,
        initial_pose=None,
        localization_wait_sec=5.0,
        undock_duration_sec=1.0,
        undock_speed=-0.08,
    ):
        self.node = node
        self.navigator = navigator
        self.docker = docker
        self.publish_initial_pose = publish_initial_pose
        self.initial_pose = initial_pose or (0.0, 0.0, 0.0, 0.0, 1.0)
        self.localization_wait_sec = localization_wait_sec
        self.undock_duration_sec = undock_duration_sec
        self.undock_speed = undock_speed

        self.state = MissionState.INIT
        self.mission_queue = []
        self.current_mission = None
        self.startup_timer = None
        self.initial_pose_timer = None
        self.initial_pose_publish_count = 0
        self.initial_pose_repetitions = 15
        self.undock_timer = None
        self.undock_end_time = None

    def set_state(self, new_state):
        self.node.get_logger().info(
            f"STATE: {self.state.name} -> {new_state.name}"
        )
        self.state = new_state

    def add_mission(self, pose, action='goto_only', name=None):
        self.mission_queue.append({
            'name': name or f"mission_{len(self.mission_queue) + 1}",
            'pose': pose,
            'action': action
        })

    def start(self):
        if self.publish_initial_pose:
            self.set_state(MissionState.SET_INITIAL_POSE)
            self.initial_pose_publish_count = 0
            self._publish_initial_pose_once()
            self.initial_pose_timer = self.node.create_timer(
                0.2,
                self._publish_initial_pose_once
            )
        else:
            self.node.get_logger().info(
                "Initial pose no publicada; usando localización actual de AMCL/RViz."
            )

        self.set_state(MissionState.WAIT_LOCALIZATION)
        self.startup_timer = self.node.create_timer(
            self.localization_wait_sec,
            self._startup_once
        )

    def _publish_initial_pose_once(self):
        if self.initial_pose_publish_count >= self.initial_pose_repetitions:
            if self.initial_pose_timer is not None:
                self.initial_pose_timer.cancel()
                self.initial_pose_timer = None
            return

        self.navigator.publish_initial_pose(*self.initial_pose)
        self.initial_pose_publish_count += 1

    def _startup_once(self):
        if self.startup_timer is not None:
            self.startup_timer.cancel()
            self.startup_timer = None

        if self.initial_pose_timer is not None:
            self.initial_pose_timer.cancel()
            self.initial_pose_timer = None

        if self.state != MissionState.WAIT_LOCALIZATION:
            return

        self.set_state(MissionState.LOAD_NEXT_MISSION)
        self.load_next_mission()

    def load_next_mission(self):
        if not self.mission_queue:
            self.current_mission = None
            self.set_state(MissionState.FINISHED)
            self.node.get_logger().info("Todas las misiones completadas.")
            return

        self.current_mission = self.mission_queue.pop(0)
        self.node.get_logger().info(
            f"Cargando misión {self.current_mission['name']} "
            f"({self.current_mission['action']})."
        )
        self.set_state(MissionState.NAVIGATING)
        self.navigator.send_goal(
            self.current_mission['pose'],
            self.on_navigation_done
        )

    def on_navigation_done(self, success):
        if self.current_mission is None:
            self.set_state(MissionState.ERROR)
            self.node.get_logger().error("No hay misión actual en on_navigation_done.")
            return

        if not success:
            self.set_state(MissionState.ERROR)
            self.node.get_logger().error("Falló la navegación.")
            return

        self.node.get_logger().info("Navegación completada.")

        action = self.current_mission['action']

        if action == 'goto_only':
            self.set_state(MissionState.LOAD_NEXT_MISSION)
            self.load_next_mission()

        elif action == 'dock':
            self.set_state(MissionState.DOCKING)
            self.docker.start(done_cb=self.on_docking_done)

        else:
            self.set_state(MissionState.ERROR)
            self.node.get_logger().error(f"Acción desconocida: {action}")

    def on_docking_done(self, success):
        if self.current_mission is None:
            self.set_state(MissionState.ERROR)
            self.node.get_logger().error("No hay misión actual en on_docking_done.")
            return

        if not success:
            mission_name = self.current_mission['name']
            self.node.get_logger().warn(
                f"Docking falló en {mission_name}; continuando con la próxima misión."
            )

            if self.mission_queue and self.undock_duration_sec > 0.0:
                self.start_undocking()
                return

            self.set_state(MissionState.LOAD_NEXT_MISSION)
            self.load_next_mission()
            return

        self.node.get_logger().info("Docking completado.")

        if self.mission_queue and self.undock_duration_sec > 0.0:
            self.start_undocking()
            return

        self.set_state(MissionState.LOAD_NEXT_MISSION)
        self.load_next_mission()

    def start_undocking(self):
        self.set_state(MissionState.UNDOCKING)
        self.undock_end_time = time.monotonic() + self.undock_duration_sec

        if self.undock_timer is not None:
            self.undock_timer.cancel()

        self.node.get_logger().info(
            f"Saliendo de docking por {self.undock_duration_sec:.1f}s."
        )
        self.undock_timer = self.node.create_timer(0.05, self._undock_step)

    def _undock_step(self):
        if self.undock_end_time is None:
            self._finish_undocking()
            return

        if time.monotonic() >= self.undock_end_time:
            self._finish_undocking()
            return

        cmd = Twist()
        cmd.linear.x = float(self.undock_speed)
        self.node.cmd_pub.publish(cmd)

    def _finish_undocking(self):
        if self.undock_timer is not None:
            self.undock_timer.cancel()
            self.undock_timer = None

        self.undock_end_time = None
        self.node.cmd_pub.publish(Twist())
        self.set_state(MissionState.LOAD_NEXT_MISSION)
        self.load_next_mission()

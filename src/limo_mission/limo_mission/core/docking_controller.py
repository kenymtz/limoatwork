import math
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Twist

from limo_mission.core.states import DockingState


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_line_angle(angle: float) -> float:
    """
    Normaliza a orientação de uma linha para [-pi/2, pi/2].

    Uma linha possui periodicidade de pi, não de 2*pi. Portanto, +90 graus
    e -90 graus representam a mesma orientação geométrica.
    """
    return 0.5 * math.atan2(math.sin(2.0 * angle), math.cos(2.0 * angle))


class DockWindow(Enum):
    READY_FOR_DOCK = auto()
    ESCAPED_LEFT = auto()
    ESCAPED_RIGHT = auto()


class DockingController:
    def __init__(self, node):
        self.node = node

        self.state = DockingState.IDLE
        self.done_cb = None
        self.timer = None
        self.active = False
        self.start_time = None
        self.timeout_sec = 45.0

        # --------------------------------------------------
        # LiDAR / clusters
        # --------------------------------------------------
        # O setor continua largo para permitir rastrear a WS durante rotações.
        # A aquisição inicial usa uma janela bem mais estreita, definida abaixo.
        self.front_sector_deg = 55.0
        self.min_range = 0.05
        self.max_range = 1.20

        self.cluster_gap_threshold = 0.06
        self.min_cluster_points = 12

        self.min_front_distance = 0.10
        self.max_front_distance = 0.80
        self.min_width = 0.10
        self.max_width = 0.50
        self.max_abs_beta_deg = 45.0

        # Rejeita clusters pouco lineares, por exemplo cantos e conjuntos em L.
        # Pode ser reduzido caso a WS real produza uma nuvem muito ruidosa.
        self.min_linearity = 5.0

        # --------------------------------------------------
        # Associação e bloqueio da WS correta
        # --------------------------------------------------
        # Durante a aquisição, somente uma WS aproximadamente à frente pode
        # ser escolhida. Isso impede que uma WS lateral mais próxima vença.
        self.acquire_max_abs_beta_deg = 18.0
        self.acquire_max_abs_offset = 0.18
        self.acquire_max_abs_yaw_deg = 30.0
        self.acquire_ratio_max = 1.45
        self.target_nominal_width = 0.25

        # Limites máximos de variação entre dois scans consecutivos para que
        # um cluster seja considerado a mesma WS já bloqueada.
        self.track_max_centroid_jump = 0.20
        self.track_max_front_jump = 0.18
        self.track_max_offset_jump = 0.16
        self.track_max_yaw_jump_deg = 15.0
        self.track_max_width_jump = 0.12

        # A WS só é liberada depois de várias perdas consecutivas.
        self.target_lost_limit = 6
        self.target_lost_counter = 0
        self.locked_target: Optional[Dict[str, object]] = None

        # --------------------------------------------------
        # Distância de docking
        # --------------------------------------------------
        self.target_distance = 0.15

        # --------------------------------------------------
        # Janela lateral com histerese
        # --------------------------------------------------
        # Para entrar em APPROACHING, usa-se a janela estreita (ready).
        # Para abandonar APPROACHING, usa-se a janela larga (escape).
        self.ready_offset_tol = 0.040
        self.escape_offset_tol = 0.055

        self.ready_beta_deg = 12.0
        self.escape_beta_deg = 15.0

        self.ready_ratio_max = 1.25
        self.escape_ratio_max = 1.35

        # --------------------------------------------------
        # Controle COARSE
        # --------------------------------------------------
        self.k_yaw_coarse = 1.8
        self.max_w_coarse = 0.35
        self.min_w_coarse = 0.06

        self.coarse_to_fine_deg = 6.0
        self.coarse_exit_deg = 10.0

        # --------------------------------------------------
        # Controle FINE
        # --------------------------------------------------
        self.k_yaw_fine = 1.5
        self.max_w_fine = 0.15
        self.min_w_fine = 0.04

        self.align_tol_fine_deg = 0.8
        self.align_exit_tol_fine_deg = 1.5
        self.aligned_required_cycles_fine = 10

        # --------------------------------------------------
        # Controle RECENTER (base omnidirecional em Y)
        # --------------------------------------------------
        self.k_y_recenter = 1.2
        self.max_vy_recenter = 0.10
        self.min_vy_recenter = 0.03
        self.recenter_stop_tol = 0.020

        self.k_yaw_recenter = 0.6
        self.max_w_recenter = 0.08

        # Mude para True se o movimento lateral estiver invertido.
        self.invert_y_axis = False

        # --------------------------------------------------
        # Controle APPROACH
        # --------------------------------------------------
        self.k_dist = 0.9
        self.max_v_approach = 0.14
        self.min_v_approach = 0.025

        self.k_yaw_approach = 0.8
        self.max_w_approach = 0.10

        self.approach_exit_yaw_deg = 2.5

        # Tolerâncias finais mais estritas.
        self.done_dist_tol = 0.03
        self.done_yaw_tol_deg = 0.8
        self.done_required_cycles = 10
        self.done_counter = 0

        # --------------------------------------------------
        # SEARCH
        # --------------------------------------------------
        self.search_w = 0.12

        # Mude para False se o robô girar para o lado incorreto.
        self.invert_yaw_sign = True

        # --------------------------------------------------
        # Calibração angular fixa
        # --------------------------------------------------
        # Compensa somente um eventual erro fixo de montagem do LiDAR.
        # Não depende de beta e, portanto, não manda o robô terminar inclinado.
        self.lidar_yaw_bias_deg = 0.0

        # --------------------------------------------------
        # Filtros
        # --------------------------------------------------
        self.filter_alpha_coarse = 0.80
        self.filter_alpha_fine = 0.65
        self.filter_alpha_approach = 0.70
        self.filter_alpha_recenter = 0.70

        self.filtered_yaw = None
        self.filtered_front = None
        self.filtered_offset = None
        self.filtered_beta = None

        # --------------------------------------------------
        # Runtime
        # --------------------------------------------------
        self.aligned_counter = 0
        self.loop_counter = 0
        self.invalid_geom_counter = 0
        self.invalid_geom_limit = 6

        # --------------------------------------------------
        # Vision + LiDAR docking mode
        # --------------------------------------------------
        self.docking_mode = str(self._param('docking_mode', 'lidar'))
        self.vision_enable_motion = bool(
            self._param('vision_enable_motion', False)
        )
        self.vision_target_timeout_sec = float(
            self._param('vision_target_timeout_sec', 0.50)
        )
        self.vision_stable_required_cycles = int(
            self._param('vision_stable_required_cycles', 10)
        )
        self.vision_coarse_to_fine_deg = float(
            self._param('vision_coarse_to_fine_deg', 5.0)
        )
        self.vision_fine_tol_deg = float(
            self._param('vision_fine_tol_deg', 1.5)
        )
        self.vision_fine_exit_deg = float(
            self._param('vision_fine_exit_deg', 7.0)
        )
        self.vision_ready_center_tol = float(
            self._param('vision_ready_center_tol', 0.12)
        )
        self.vision_ready_min_distance = float(
            self._param('vision_ready_min_distance', 0.18)
        )
        self.vision_ready_max_distance = float(
            self._param('vision_ready_max_distance', 1.20)
        )
        self.vision_k_yaw_coarse = float(
            self._param('vision_k_yaw_coarse', 0.9)
        )
        self.vision_k_yaw_fine = float(
            self._param('vision_k_yaw_fine', 0.55)
        )
        self.vision_max_w_coarse = float(
            self._param('vision_max_w_coarse', 0.12)
        )
        self.vision_max_w_fine = float(
            self._param('vision_max_w_fine', 0.06)
        )
        self.vision_min_w = float(self._param('vision_min_w', 0.015))
        self.vision_yaw_sign = float(self._param('vision_yaw_sign', -1.0))
        self.vision_filter_alpha = float(
            self._param('vision_filter_alpha', 0.65)
        )
        self.vision_finish_on_ready = bool(
            self._param('vision_finish_on_ready', False)
        )

        self.filtered_vision_angle = None
        self.filtered_vision_center = None
        self.filtered_vision_distance = None
        self.vision_valid_counter = 0
        self.vision_aligned_counter = 0
        self._vision_observer_notice_sent = False

    # ======================================================
    # API pública
    # ======================================================
    def _param(self, name, default):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def start(self, done_cb=None):
        if self.active:
            self.node.get_logger().warn("Docking já está ativo.")
            return

        self.done_cb = done_cb
        self.active = True
        if self.docking_mode == 'vision_lidar':
            self.state = DockingState.VISION_ACQUIRE_TARGET
        else:
            self.state = DockingState.SEARCH
        self.start_time = time.monotonic()

        self._reset_filters()
        self._reset_vision_filters()
        self._release_target(log_message=False)

        self.aligned_counter = 0
        self.done_counter = 0
        self.loop_counter = 0
        self.invalid_geom_counter = 0
        self.vision_valid_counter = 0
        self.vision_aligned_counter = 0
        self._vision_observer_notice_sent = False

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.timer = self.node.create_timer(0.05, self.control_loop)
        self.node.get_logger().info(
            f"Docking iniciado. mode={self.docking_mode}, "
            f"vision_motion={self.vision_enable_motion}"
        )

    def stop(self):
        self.active = False
        self.start_time = None

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self._release_target(log_message=False)
        self._publish_cmd(0.0, 0.0, 0.0)

    def finish(self, success: bool):
        self.stop()

        if success:
            self.state = DockingState.DONE
            self.node.get_logger().info("Docking completado com sucesso.")
        else:
            self.state = DockingState.FAILED
            self.node.get_logger().error("Docking falhou.")

        if self.done_cb is not None:
            cb = self.done_cb
            self.done_cb = None
            cb(success)

    # ======================================================
    # Utilidades
    # ======================================================
    def _set_state(self, new_state: DockingState):
        if new_state != self.state:
            self.node.get_logger().info(
                f"DOCK STATE: {self.state.name} -> {new_state.name}"
            )
            self.state = new_state
            self.aligned_counter = 0
            self.done_counter = 0

    def _publish_cmd(self, vx: float, vy: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(w)
        self.node.cmd_pub.publish(cmd)

    def _reset_filters(self):
        self.filtered_yaw = None
        self.filtered_front = None
        self.filtered_offset = None
        self.filtered_beta = None

    def _reset_vision_filters(self):
        self.filtered_vision_angle = None
        self.filtered_vision_center = None
        self.filtered_vision_distance = None

    def lowpass(self, old_value, new_value, alpha):
        if old_value is None:
            return new_value
        return alpha * old_value + (1.0 - alpha) * new_value

    def lowpass_line_angle(self, old_value, new_value, alpha):
        if old_value is None:
            return wrap_line_angle(new_value)

        delta = wrap_line_angle(new_value - old_value)
        return wrap_line_angle(old_value + (1.0 - alpha) * delta)

    def get_distance_at_angle(
        self,
        target_angle_deg: float
    ) -> Optional[float]:
        scan = self.node.laser_scan
        if scan is None:
            return None

        if scan.angle_increment == 0.0:
            return None

        angle_rad = math.radians(target_angle_deg)
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))

        if 0 <= idx < len(scan.ranges):
            r = scan.ranges[idx]
            max_valid = min(self.max_range, scan.range_max)

            if math.isfinite(r) and self.min_range < r < max_valid:
                return float(r)

        return None

    # ======================================================
    # Extração da geometria
    # ======================================================
    def extract_points(self) -> List[np.ndarray]:
        scan = self.node.laser_scan
        if scan is None:
            return []

        points = []
        sector_rad = math.radians(self.front_sector_deg)
        max_valid = min(self.max_range, scan.range_max)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue

            if not (self.min_range < r < max_valid):
                continue

            theta = scan.angle_min + i * scan.angle_increment
            if abs(theta) > sector_rad:
                continue

            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append(np.array([x, y], dtype=float))

        return points

    def split_into_clusters(
        self,
        points: List[np.ndarray]
    ) -> List[np.ndarray]:
        if not points:
            return []

        clusters = []
        current = [points[0]]

        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - points[i - 1])

            if distance > self.cluster_gap_threshold:
                if len(current) >= self.min_cluster_points:
                    clusters.append(np.array(current))
                current = [points[i]]
            else:
                current.append(points[i])

        if len(current) >= self.min_cluster_points:
            clusters.append(np.array(current))

        return clusters

    def fit_face_pca(self, cluster: np.ndarray):
        if cluster is None or len(cluster) < 2:
            return None

        centroid = cluster.mean(axis=0)
        centered = cluster - centroid
        cov = np.cov(centered.T)

        if cov.shape != (2, 2) or not np.all(np.isfinite(cov)):
            return None

        eigvals, eigvecs = np.linalg.eigh(cov)

        major_index = int(np.argmax(eigvals))
        minor_index = 1 - major_index

        major_eig = float(max(eigvals[major_index], 0.0))
        minor_eig = float(max(eigvals[minor_index], 0.0))
        linearity = major_eig / max(minor_eig, 1e-9)

        tangent = eigvecs[:, major_index]
        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm < 1e-9:
            return None

        tangent = tangent / tangent_norm

        # Mantém uma orientação consistente para a tangente.
        if tangent[1] < 0.0:
            tangent = -tangent

        normal = np.array([tangent[1], -tangent[0]], dtype=float)
        normal_norm = np.linalg.norm(normal)

        if normal_norm < 1e-9:
            return None

        normal = normal / normal_norm

        # A normal deve apontar aproximadamente para a frente do robô.
        if normal[0] < 0.0:
            normal = -normal

        front_distance = float(np.dot(normal, centroid))
        lateral_offset = float(centroid[1])

        # Zero significa que a face está perpendicular ao eixo X do robô.
        yaw_error = wrap_line_angle(
            math.atan2(float(tangent[0]), float(tangent[1]))
        )

        beta = math.atan2(float(centroid[1]), float(centroid[0]))

        projections = centered @ tangent
        width_estimate = float(projections.max() - projections.min())

        ray0 = self.get_distance_at_angle(0.0)
        frontal_ratio = float('inf')

        if ray0 is not None and front_distance > 1e-6:
            frontal_ratio = float(ray0 / front_distance)

        return {
            'centroid': centroid,
            'tangent': tangent,
            'normal': normal,
            'front_distance': front_distance,
            'lateral_offset': lateral_offset,
            'yaw_error': yaw_error,
            'beta': beta,
            'width_estimate': width_estimate,
            'num_points': int(len(cluster)),
            'ray0': ray0,
            'frontal_ratio': frontal_ratio,
            'linearity': linearity,
        }

    def cluster_is_valid(self, geom) -> bool:
        if geom is None:
            return False

        pts = geom['num_points']
        front = geom['front_distance']
        width = geom['width_estimate']
        beta_deg = abs(math.degrees(geom['beta']))
        ratio = geom['frontal_ratio']
        linearity = geom['linearity']

        if pts < self.min_cluster_points:
            return False

        if not (self.min_front_distance <= front <= self.max_front_distance):
            return False

        if not (self.min_width <= width <= self.max_width):
            return False

        if beta_deg > self.max_abs_beta_deg:
            return False

        if not math.isfinite(ratio):
            return False

        if not math.isfinite(linearity) or linearity < self.min_linearity:
            return False

        return True

    # ======================================================
    # Associação da WS correta
    # ======================================================
    def _target_signature(self, geom) -> Dict[str, object]:
        return {
            'centroid': np.array(geom['centroid'], dtype=float).copy(),
            'front_distance': float(geom['front_distance']),
            'lateral_offset': float(geom['lateral_offset']),
            'yaw_error': float(geom['yaw_error']),
            'beta': float(geom['beta']),
            'width_estimate': float(geom['width_estimate']),
        }

    def _release_target(self, log_message: bool = True):
        had_target = self.locked_target is not None

        self.locked_target = None
        self.target_lost_counter = 0
        self._reset_filters()

        if log_message and had_target:
            self.node.get_logger().warn("WS bloqueada foi liberada.")

    def _register_target_miss(self):
        if self.locked_target is None:
            return

        self.target_lost_counter += 1

        if self.target_lost_counter >= self.target_lost_limit:
            self.node.get_logger().warn(
                "WS bloqueada perdida por vários scans consecutivos."
            )
            self._release_target(log_message=False)

    def _line_angle_difference(self, angle_a: float, angle_b: float) -> float:
        return abs(wrap_line_angle(angle_a - angle_b))

    def _acquire_target(
        self,
        candidates: List[Tuple[np.ndarray, Dict[str, object]]]
    ) -> Optional[np.ndarray]:
        """
        Adquire somente uma WS aproximadamente à frente do robô.

        A distância frontal possui peso pequeno. O centro, beta e orientação
        têm prioridade para impedir que uma WS lateral mais próxima seja
        escolhida.
        """
        acquisition_candidates = []

        for cluster, geom in candidates:
            beta_deg = abs(math.degrees(geom['beta']))
            offset = abs(geom['lateral_offset'])
            yaw_deg = abs(math.degrees(geom['yaw_error']))
            ratio = geom['frontal_ratio']
            width = geom['width_estimate']
            front = geom['front_distance']
            pts = geom['num_points']
            linearity = geom['linearity']

            if beta_deg > self.acquire_max_abs_beta_deg:
                continue

            if offset > self.acquire_max_abs_offset:
                continue

            if yaw_deg > self.acquire_max_abs_yaw_deg:
                continue

            if ratio > self.acquire_ratio_max:
                continue

            beta_cost = beta_deg / max(self.acquire_max_abs_beta_deg, 1e-6)
            offset_cost = offset / max(self.acquire_max_abs_offset, 1e-6)
            yaw_cost = yaw_deg / max(self.acquire_max_abs_yaw_deg, 1e-6)
            width_cost = (
                abs(width - self.target_nominal_width)
                / max(self.max_width - self.min_width, 1e-6)
            )

            # A distância participa apenas como desempate leve.
            front_cost = front / max(self.max_front_distance, 1e-6)

            # Maior linearidade e mais pontos são pequenas bonificações.
            line_bonus = min(linearity, 50.0) / 50.0
            point_bonus = min(pts, 100) / 100.0

            score = (
                5.0 * offset_cost
                + 4.0 * beta_cost
                + 2.0 * yaw_cost
                + 0.7 * width_cost
                + 0.20 * front_cost
                - 0.15 * line_bonus
                - 0.10 * point_bonus
            )

            acquisition_candidates.append((score, cluster, geom))

        if not acquisition_candidates:
            return None

        acquisition_candidates.sort(key=lambda item: item[0])
        _, selected_cluster, selected_geom = acquisition_candidates[0]

        self.locked_target = self._target_signature(selected_geom)
        self.target_lost_counter = 0
        self._reset_filters()

        self.node.get_logger().info(
            "WS bloqueada: "
            f"front={selected_geom['front_distance']:.3f} m, "
            f"offset={selected_geom['lateral_offset']:.3f} m, "
            f"beta={math.degrees(selected_geom['beta']):.2f} deg, "
            f"yaw={math.degrees(selected_geom['yaw_error']):.2f} deg"
        )

        return selected_cluster

    def _track_target(
        self,
        candidates: List[Tuple[np.ndarray, Dict[str, object]]]
    ) -> Optional[np.ndarray]:
        """
        Rastreia a WS já bloqueada por continuidade geométrica.

        Um cluster lateral não pode substituir o alvo somente por estar mais
        próximo. Ele precisa ser compatível com a posição, orientação e largura
        observadas no scan anterior.
        """
        if self.locked_target is None:
            return self._acquire_target(candidates)

        previous = self.locked_target
        tracked_candidates = []

        for cluster, geom in candidates:
            centroid_jump = float(
                np.linalg.norm(geom['centroid'] - previous['centroid'])
            )
            front_jump = abs(
                geom['front_distance'] - previous['front_distance']
            )
            offset_jump = abs(
                geom['lateral_offset'] - previous['lateral_offset']
            )
            yaw_jump = self._line_angle_difference(
                geom['yaw_error'],
                previous['yaw_error']
            )
            width_jump = abs(
                geom['width_estimate'] - previous['width_estimate']
            )

            if centroid_jump > self.track_max_centroid_jump:
                continue

            if front_jump > self.track_max_front_jump:
                continue

            if offset_jump > self.track_max_offset_jump:
                continue

            if yaw_jump > math.radians(self.track_max_yaw_jump_deg):
                continue

            if width_jump > self.track_max_width_jump:
                continue

            score = (
                4.0 * centroid_jump / self.track_max_centroid_jump
                + 2.0 * front_jump / self.track_max_front_jump
                + 3.0 * offset_jump / self.track_max_offset_jump
                + 3.0 * yaw_jump
                / math.radians(self.track_max_yaw_jump_deg)
                + 1.5 * width_jump / self.track_max_width_jump
            )

            tracked_candidates.append((score, cluster, geom))

        if not tracked_candidates:
            self._register_target_miss()
            return None

        tracked_candidates.sort(key=lambda item: item[0])
        _, selected_cluster, selected_geom = tracked_candidates[0]

        self.locked_target = self._target_signature(selected_geom)
        self.target_lost_counter = 0

        return selected_cluster

    def choose_box_cluster(
        self,
        clusters: List[np.ndarray]
    ) -> Optional[np.ndarray]:
        if not clusters:
            self._register_target_miss()
            return None

        candidates = []

        for cluster in clusters:
            geom = self.fit_face_pca(cluster)

            if not self.cluster_is_valid(geom):
                continue

            candidates.append((cluster, geom))

        if not candidates:
            self._register_target_miss()
            return None

        if self.locked_target is None:
            return self._acquire_target(candidates)

        return self._track_target(candidates)

    def analyze_geometry(self):
        points = self.extract_points()

        if len(points) < self.min_cluster_points:
            self._register_target_miss()
            return None

        clusters = self.split_into_clusters(points)

        if not clusters:
            self._register_target_miss()
            return None

        cluster = self.choose_box_cluster(clusters)

        if cluster is None:
            return None

        geom = self.fit_face_pca(cluster)

        if not self.cluster_is_valid(geom):
            self._register_target_miss()
            return None

        return geom

    # ======================================================
    # Janela lateral com histerese
    # ======================================================
    def is_ready_for_dock(
        self,
        offset: float,
        beta_deg: float,
        ratio: float,
        ray0: Optional[float]
    ) -> bool:
        return (
            abs(offset) < self.ready_offset_tol
            and abs(beta_deg) < self.ready_beta_deg
            and ratio < self.ready_ratio_max
            and ray0 is not None
        )

    def has_escaped_dock_window(
        self,
        offset: float,
        beta_deg: float,
        ratio: float,
        ray0: Optional[float]
    ) -> bool:
        return (
            abs(offset) > self.escape_offset_tol
            or abs(beta_deg) > self.escape_beta_deg
            or ratio > self.escape_ratio_max
            or ray0 is None
        )

    def classify_dock_window(
        self,
        offset: float,
        beta_deg: float,
        ratio: float,
        ray0: Optional[float]
    ) -> DockWindow:
        if self.is_ready_for_dock(offset, beta_deg, ratio, ray0):
            return DockWindow.READY_FOR_DOCK

        sign_metric = offset + 0.002 * beta_deg

        if sign_metric >= 0.0:
            return DockWindow.ESCAPED_LEFT

        return DockWindow.ESCAPED_RIGHT

    # ======================================================
    # Vision + LiDAR alignment
    # ======================================================
    def _get_fresh_vision_target(self):
        if not getattr(self.node, 'vision_target_valid', False):
            return None

        target = getattr(self.node, 'vision_target', None)
        target_time = getattr(self.node, 'vision_target_time', None)

        if target is None or target_time is None:
            return None

        if time.monotonic() - target_time > self.vision_target_timeout_sec:
            return None

        angle = float(target.vector.x)
        center = float(target.vector.y)
        distance = float(target.vector.z)

        if not all(math.isfinite(v) for v in (angle, center, distance)):
            return None

        return angle, center, distance

    def _vision_publish_cmd(self, vx: float, vy: float, w: float):
        if self.vision_enable_motion:
            self._publish_cmd(vx, vy, w)
            return

        self._publish_cmd(0.0, 0.0, 0.0)

        if not self._vision_observer_notice_sent:
            self.node.get_logger().warn(
                "Vision docking en modo observador: comandos calculados, "
                "pero no se mueve. Use vision_enable_motion:=true para mover."
            )
            self._vision_observer_notice_sent = True

    def _vision_alignment_ready(
        self,
        angle_deg: float,
        center_error: float,
        distance: float
    ) -> bool:
        return (
            abs(angle_deg) < self.vision_fine_tol_deg
            and abs(center_error) < self.vision_ready_center_tol
            and self.vision_ready_min_distance
            <= distance
            <= self.vision_ready_max_distance
        )

    def _vision_control_loop(self):
        target = self._get_fresh_vision_target()

        if target is None:
            self.vision_valid_counter = 0
            self.vision_aligned_counter = 0
            self._reset_vision_filters()
            self._vision_publish_cmd(0.0, 0.0, 0.0)
            self._set_state(DockingState.VISION_ACQUIRE_TARGET)

            if self.loop_counter % 10 == 0:
                self.node.get_logger().warn(
                    "Vision docking: esperando target visual estable."
                )
            return

        raw_angle, raw_center, raw_distance = target
        self.vision_valid_counter += 1

        alpha = self.vision_filter_alpha
        self.filtered_vision_angle = self.lowpass(
            self.filtered_vision_angle,
            raw_angle,
            alpha
        )
        self.filtered_vision_center = self.lowpass(
            self.filtered_vision_center,
            raw_center,
            alpha
        )
        self.filtered_vision_distance = self.lowpass(
            self.filtered_vision_distance,
            raw_distance,
            alpha
        )

        angle = self.filtered_vision_angle
        center = self.filtered_vision_center
        distance = self.filtered_vision_distance
        angle_deg = math.degrees(angle)
        ready = self._vision_alignment_ready(angle_deg, center, distance)

        if self.state == DockingState.VISION_ACQUIRE_TARGET:
            if self.vision_valid_counter >= 3:
                self._set_state(DockingState.VISION_ALIGN_COARSE)

        elif self.state == DockingState.VISION_ALIGN_COARSE:
            if abs(angle_deg) < self.vision_coarse_to_fine_deg:
                self._set_state(DockingState.VISION_ALIGN_FINE)

        elif self.state == DockingState.VISION_ALIGN_FINE:
            if abs(angle_deg) > self.vision_fine_exit_deg:
                self._set_state(DockingState.VISION_ALIGN_COARSE)
            elif ready:
                self.vision_aligned_counter += 1
                if (
                    self.vision_aligned_counter
                    >= self.vision_stable_required_cycles
                ):
                    self._set_state(DockingState.VISION_READY)
            else:
                self.vision_aligned_counter = 0

        elif self.state == DockingState.VISION_READY:
            if not ready:
                self.vision_aligned_counter = 0
                self._set_state(DockingState.VISION_ALIGN_FINE)
            elif self.vision_finish_on_ready:
                self.finish(True)
                return

        elif self.state == DockingState.VISION_APPROACHING:
            # Reserved for the next phase. For now alignment stops at READY.
            self._set_state(DockingState.VISION_READY)

        w = 0.0

        if self.state == DockingState.VISION_ALIGN_COARSE:
            w = clamp(
                self.vision_yaw_sign * self.vision_k_yaw_coarse * angle,
                -self.vision_max_w_coarse,
                self.vision_max_w_coarse
            )
        elif self.state == DockingState.VISION_ALIGN_FINE:
            w = clamp(
                self.vision_yaw_sign * self.vision_k_yaw_fine * angle,
                -self.vision_max_w_fine,
                self.vision_max_w_fine
            )

            if (
                abs(angle_deg) > self.vision_fine_tol_deg
                and abs(w) < self.vision_min_w
            ):
                w = math.copysign(self.vision_min_w, w)

        self._vision_publish_cmd(0.0, 0.0, w)

        if self.loop_counter % 4 == 0:
            self.node.get_logger().info(
                ' | '.join([
                    f'state={self.state.name}',
                    f'vision_valid={self.vision_valid_counter}',
                    f'angle={angle_deg:.2f}deg',
                    f'center={center:.3f}',
                    f'dist={distance:.3f}',
                    f'ready={ready}',
                    f'cmd_w={w:.3f}',
                    f'motion={self.vision_enable_motion}',
                ])
            )

    # ======================================================
    # Loop principal
    # ======================================================
    def control_loop(self):
        if not self.active:
            return

        if (
            self.start_time is not None
            and time.monotonic() - self.start_time > self.timeout_sec
        ):
            self.node.get_logger().error("Docking timeout.")
            self.finish(False)
            return

        self.loop_counter += 1

        if self.docking_mode == 'vision_lidar':
            self._vision_control_loop()
            return

        if self.node.laser_scan is None:
            if self.loop_counter % 10 == 0:
                self.node.get_logger().info("Esperando /scan...")

            self._publish_cmd(0.0, 0.0, 0.0)
            return

        geom = self.analyze_geometry()

        if geom is None:
            self.invalid_geom_counter += 1

            # Segurança: nunca mantenha o último comando quando o alvo some.
            self._publish_cmd(0.0, 0.0, 0.0)

            if self.invalid_geom_counter >= self.invalid_geom_limit:
                self._set_state(DockingState.SEARCH)

                # Só busca girando depois de o alvo bloqueado ter sido liberado.
                # Enquanto ele ainda está bloqueado, permanece parado esperando
                # a continuidade reaparecer, evitando trocar para outra WS.
                if self.locked_target is None:
                    self._publish_cmd(0.0, 0.0, self.search_w)

            if self.loop_counter % 10 == 0:
                self.node.get_logger().warn(
                    "Geometria inválida, alvo ambíguo ou WS perdida."
                )

            return

        self.invalid_geom_counter = 0

        raw_yaw = geom['yaw_error']
        raw_front = geom['front_distance']
        raw_offset = geom['lateral_offset']
        raw_beta = geom['beta']

        pts = geom['num_points']
        width = geom['width_estimate']
        ratio = geom['frontal_ratio']
        ray0 = geom['ray0']
        linearity = geom['linearity']

        if self.state == DockingState.ALIGNING_FINE:
            alpha = self.filter_alpha_fine
        elif self.state == DockingState.APPROACHING:
            alpha = self.filter_alpha_approach
        elif self.state == DockingState.RECENTER_FOR_DOCK:
            alpha = self.filter_alpha_recenter
        else:
            alpha = self.filter_alpha_coarse

        self.filtered_yaw = self.lowpass_line_angle(
            self.filtered_yaw,
            raw_yaw,
            alpha
        )
        self.filtered_front = self.lowpass(
            self.filtered_front,
            raw_front,
            alpha
        )
        self.filtered_offset = self.lowpass(
            self.filtered_offset,
            raw_offset,
            alpha
        )
        self.filtered_beta = self.lowpass(
            self.filtered_beta,
            raw_beta,
            alpha
        )

        yaw = self.filtered_yaw
        front = self.filtered_front
        offset = self.filtered_offset
        beta = self.filtered_beta

        beta_deg = math.degrees(beta)

        # Erro angular único para controle e transições.
        yaw_corr_rad = wrap_line_angle(
            yaw - math.radians(self.lidar_yaw_bias_deg)
        )
        yaw_corr_deg = math.degrees(yaw_corr_rad)

        yaw_control = (
            -yaw_corr_rad
            if self.invert_yaw_sign
            else yaw_corr_rad
        )

        dist_error = front - self.target_distance

        dock_window = self.classify_dock_window(
            offset,
            beta_deg,
            ratio,
            ray0
        )
        ready_for_dock = self.is_ready_for_dock(
            offset,
            beta_deg,
            ratio,
            ray0
        )
        escaped_dock_window = self.has_escaped_dock_window(
            offset,
            beta_deg,
            ratio,
            ray0
        )

        # --------------------------------------------------
        # Transições de estado
        # --------------------------------------------------
        if self.state == DockingState.SEARCH:
            self._set_state(DockingState.ALIGNING_COARSE)

        elif self.state == DockingState.ALIGNING_COARSE:
            if abs(yaw_corr_deg) < self.coarse_to_fine_deg:
                self._set_state(DockingState.ALIGNING_FINE)

        elif self.state == DockingState.ALIGNING_FINE:
            if abs(yaw_corr_deg) > self.coarse_exit_deg:
                self._set_state(DockingState.ALIGNING_COARSE)

            elif not ready_for_dock:
                self._set_state(DockingState.RECENTER_FOR_DOCK)

            else:
                if abs(yaw_corr_deg) < self.align_tol_fine_deg:
                    self.aligned_counter += 1
                elif abs(yaw_corr_deg) > self.align_exit_tol_fine_deg:
                    self.aligned_counter = 0

                # Não existe mais a condição que permitia avançar após uma
                # única leitura favorável. O alinhamento precisa ser estável.
                if (
                    self.aligned_counter
                    >= self.aligned_required_cycles_fine
                ):
                    self._set_state(DockingState.APPROACHING)

        elif self.state == DockingState.RECENTER_FOR_DOCK:
            if abs(yaw_corr_deg) > self.coarse_exit_deg:
                self._set_state(DockingState.ALIGNING_COARSE)

            elif ready_for_dock:
                self._set_state(DockingState.ALIGNING_FINE)

        elif self.state == DockingState.APPROACHING:
            if abs(yaw_corr_deg) > self.approach_exit_yaw_deg:
                self._set_state(DockingState.ALIGNING_FINE)

            # Usa os limites largos para sair, criando histerese.
            elif escaped_dock_window:
                self._set_state(DockingState.RECENTER_FOR_DOCK)

            else:
                if (
                    abs(dist_error) < self.done_dist_tol
                    and abs(yaw_corr_deg) < self.done_yaw_tol_deg
                ):
                    self.done_counter += 1
                else:
                    self.done_counter = 0

                if self.done_counter >= self.done_required_cycles:
                    self.finish(True)
                    return

        elif self.state == DockingState.DONE:
            return

        # --------------------------------------------------
        # Controle
        # --------------------------------------------------
        vx = 0.0
        vy = 0.0
        w = 0.0

        if self.state == DockingState.SEARCH:
            # Se ainda existe alvo bloqueado, não gira procurando outra WS.
            if self.locked_target is None:
                w = self.search_w

        elif self.state == DockingState.ALIGNING_COARSE:
            w = clamp(
                self.k_yaw_coarse * yaw_control,
                -self.max_w_coarse,
                self.max_w_coarse
            )

            if (
                abs(yaw_corr_deg) > self.coarse_to_fine_deg
                and abs(w) < self.min_w_coarse
            ):
                w = math.copysign(self.min_w_coarse, yaw_control)

        elif self.state == DockingState.ALIGNING_FINE:
            w = clamp(
                self.k_yaw_fine * yaw_control,
                -self.max_w_fine,
                self.max_w_fine
            )

            if abs(yaw_corr_deg) > 0.20 and abs(w) < self.min_w_fine:
                w = math.copysign(self.min_w_fine, yaw_control)

            if abs(yaw_corr_deg) < 0.15:
                w = 0.0

        elif self.state == DockingState.RECENTER_FOR_DOCK:
            lateral_error = offset

            vy = clamp(
                self.k_y_recenter * lateral_error,
                -self.max_vy_recenter,
                self.max_vy_recenter
            )

            if self.invert_y_axis:
                vy = -vy

            if (
                abs(lateral_error) > self.recenter_stop_tol
                and abs(vy) < self.min_vy_recenter
            ):
                vy = math.copysign(self.min_vy_recenter, vy)

            if abs(lateral_error) < self.recenter_stop_tol:
                vy = 0.0

            w = clamp(
                self.k_yaw_recenter * yaw_control,
                -self.max_w_recenter,
                self.max_w_recenter
            )

        elif self.state == DockingState.APPROACHING:
            if dist_error > self.done_dist_tol:
                vx = clamp(
                    self.k_dist * dist_error,
                    self.min_v_approach,
                    self.max_v_approach
                )
            else:
                vx = 0.0

            # Mesmo ao alcançar a distância, continua corrigindo yaw até
            # satisfazer a tolerância final durante vários ciclos.
            w = clamp(
                self.k_yaw_approach * yaw_control,
                -self.max_w_approach,
                self.max_w_approach
            )

        self._publish_cmd(vx, vy, w)

        # --------------------------------------------------
        # Debug
        # --------------------------------------------------
        if self.loop_counter % 4 == 0:
            d_m45 = self.get_distance_at_angle(-45.0)
            d_m30 = self.get_distance_at_angle(-30.0)
            d_0 = self.get_distance_at_angle(0.0)
            d_p30 = self.get_distance_at_angle(30.0)
            d_p45 = self.get_distance_at_angle(45.0)

            self.node.get_logger().info(
                ' | '.join([
                    f'state={self.state.name}',
                    f'window={dock_window.name}',
                    f'locked={self.locked_target is not None}',
                    f'pts={pts}',
                    f'front={front:.3f}',
                    f'dist_err={dist_error:.3f}',
                    f'offset={offset:.3f}',
                    f'beta={beta_deg:.2f}deg',
                    f'yaw_corr={yaw_corr_deg:.2f}deg',
                    f'width={width:.3f}',
                    f'linearity={linearity:.1f}',
                    f'ratio={ratio:.2f}',
                    f'cmd=({vx:.3f},{vy:.3f},{w:.3f})',
                    f'aligned={self.aligned_counter}',
                    f'done={self.done_counter}',
                    (
                        'rays[-45,-30,0,30,45]='
                        f'[{self._fmt(d_m45)}, {self._fmt(d_m30)}, '
                        f'{self._fmt(d_0)}, {self._fmt(d_p30)}, '
                        f'{self._fmt(d_p45)}]'
                    )
                ])
            )

    def _fmt(self, value: Optional[float]) -> str:
        return 'None' if value is None else f'{value:.3f}'

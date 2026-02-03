#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_handler_msgs.msg import DxlStates, DxlCommandsX
from std_msgs.msg import Float64MultiArray
import threading
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from typing import List, Optional


def _deg2rad(deg: float) -> float:
    return float(deg) * np.pi / 180.0


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v
    return v / n


def _rot_z(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _rot_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    # Rodrigues' rotation formula
    a = _normalize(axis)
    x, y, z = float(a[0]), float(a[1]), float(a[2])
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def _circle_points(center: np.ndarray, normal: np.ndarray, radius: float, n: int = 120) -> np.ndarray:
    nrm = _normalize(normal)
    ref = np.array([0.0, 0.0, 1.0], dtype=float)
    u = np.cross(nrm, ref)
    if float(np.linalg.norm(u)) < 1e-10:
        ref = np.array([1.0, 0.0, 0.0], dtype=float)
        u = np.cross(nrm, ref)
    u = _normalize(u)
    v = np.cross(nrm, u)
    t = np.linspace(0.0, 2.0 * np.pi, int(n), endpoint=True)
    return np.asarray(center, dtype=float).reshape(3)[None, :] + float(radius) * (
        np.cos(t)[:, None] * u[None, :] + np.sin(t)[:, None] * v[None, :]
    )


def _make_arm_pose(
    *,
    hinge: np.ndarray,
    arm_dir0: np.ndarray,
    phi_deg: float,
    psi_deg: float,
    theta_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns:
      (hinge(3,), arm_dir(3,), rotor_normal(3,))
    """
    z = np.array([0.0, 0.0, 1.0], dtype=float)

    # Keep sign convention consistent with auto_thrust_recorder/scripts/visualize_morph_drone.py
    phi = _deg2rad(-float(phi_deg))
    psi = _deg2rad(-float(psi_deg))
    theta = _deg2rad(float(theta_deg))

    R_fold = _rot_z(phi)
    arm_dir1 = _normalize(R_fold @ _normalize(arm_dir0))

    slant_axis = np.cross(z, arm_dir1)
    if float(np.linalg.norm(slant_axis)) < 1e-10:
        R_slant = np.eye(3, dtype=float)
    else:
        R_slant = _rot_axis_angle(slant_axis, psi)

    arm_dir2 = _normalize(R_slant @ arm_dir1)
    rotor_n2 = _normalize(R_slant @ (R_fold @ z))

    R_tilt = _rot_axis_angle(arm_dir2, theta)
    rotor_n3 = _normalize(R_tilt @ rotor_n2)

    return np.asarray(hinge, dtype=float).reshape(3), arm_dir2, rotor_n3


def _compute_poses(phi_deg: float, psi_deg: float, theta_deg: float, *, cx: float, cy: float, symmetry: str) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    if symmetry not in {"mirror_xy", "none"}:
        raise ValueError(f"Unknown symmetry: {symmetry}")

    base_hinge = np.array([+float(cx), +float(cy), 0.0], dtype=float)
    base_arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))
    base_pose = _make_arm_pose(
        hinge=base_hinge,
        arm_dir0=base_arm_dir0,
        phi_deg=float(phi_deg),
        psi_deg=float(psi_deg),
        theta_deg=float(theta_deg),
    )

    if symmetry == "mirror_xy":
        M_id = np.diag([1.0, 1.0, 1.0])
        M_x = np.diag([-1.0, 1.0, 1.0])
        M_y = np.diag([1.0, -1.0, 1.0])
        M_xy = np.diag([-1.0, -1.0, 1.0])
        Ms = [M_id, M_x, M_xy, M_y]
        out: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        for M in Ms:
            h, a, n = base_pose
            out.append((np.asarray(M @ h, dtype=float).reshape(3), _normalize(M @ a), _normalize(M @ n)))
        return out

    hinges = [
        np.array([+float(cx), +float(cy), 0.0], dtype=float),
        np.array([-float(cx), +float(cy), 0.0], dtype=float),
        np.array([-float(cx), -float(cy), 0.0], dtype=float),
        np.array([+float(cx), -float(cy), 0.0], dtype=float),
    ]
    arm_dirs0 = []
    for h in hinges:
        sx = 1.0 if float(h[0]) >= 0.0 else -1.0
        sy = 1.0 if float(h[1]) >= 0.0 else -1.0
        arm_dirs0.append(_normalize(np.array([sx, sy, 0.0], dtype=float)))
    return [
        _make_arm_pose(hinge=h, arm_dir0=d0, phi_deg=float(phi_deg), psi_deg=float(psi_deg), theta_deg=float(theta_deg))
        for (h, d0) in zip(hinges, arm_dirs0, strict=True)
    ]


class MorphDroneSliderNode(Node):
    """
    変形ドローン用Dynamixelモータ制御ノード（matplotlibスライダ付き）
    
    機能:
    - 2台のDynamixelモータをmatplotlibスライダで制御
    - extended position制御を使用
    - 原点は固定指定または起動時の現在値を原点とするモードに対応
    """
    
    def __init__(self):
        super().__init__('morph_drone_slider_node')
        
        # パラメータの宣言と取得
        self.declare_parameter('motor_ids', [2, 3])
        self.declare_parameter('control_scheme', 'fold_tilt')  # 'fold_tilt' or 'motor_delta'
        self.declare_parameter('origin_mode', 'startup')  # 'startup' or 'fixed'
        self.declare_parameter('origin_deg_list', [0.0, 0.0])
        # motor_delta 用（後方互換）
        self.declare_parameter('slider_min_deg', -180.0)
        self.declare_parameter('slider_max_deg', 180.0)
        # fold/tilt 用
        self.declare_parameter('fold_slider_min_deg', -90.0)
        self.declare_parameter('fold_slider_max_deg', 90.0)
        self.declare_parameter('tilt_slider_min_deg', -45.0)
        self.declare_parameter('tilt_slider_max_deg', 45.0)
        # 機構パラメータ
        # 1 motor rev -> fold gear rev = 0.01923
        self.declare_parameter('foldgear_rev_per_motor_rev', 0.01923)
        # tilt_deg = tilt_gain * (foldgear_rev_diff) * 360
        self.declare_parameter('tilt_gain', 1.706)
        # チルト角の符号（実機の+方向に合わせる）
        # +1.0: そのまま / -1.0: チルトを反転
        self.declare_parameter('tilt_sign', 1.0)
        # 差分スケール（実機で「差分」が(差/2)扱い等の場合に調整する）
        # - 1.0: diff = (g1 - g2)
        # - 2.0: diff = 2*(g1 - g2) 相当（tiltが半分に見える場合の補正）
        self.declare_parameter('tilt_diff_scale', 1.0)
        # モータ向き補正（必要なら -1 を指定）
        self.declare_parameter('motor_signs', [1.0, 1.0])
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_topic', '/dynamixel/commands/x')
        self.declare_parameter('state_topic', '/dynamixel/states')
        # 3Dプレビュー連携用（fold_tiltスキーム時に publish）
        # phi = fold, theta = tilt, psi は固定値（launch引数から渡す想定）
        self.declare_parameter('psi_fixed_deg', 0.0)
        self.declare_parameter('angles_topic', '/morph_drone/angles_deg')
        self.declare_parameter('profile_vel_deg_s', 100.0)
        self.declare_parameter('profile_acc_deg_ss', 200.0)
        # デバッグ表示
        self.declare_parameter('debug_print_commands', True)
        self.declare_parameter('debug_print_period_s', 0.5)
        
        self.motor_ids: List[int] = self.get_parameter('motor_ids').get_parameter_value().integer_array_value
        self.control_scheme: str = self.get_parameter('control_scheme').get_parameter_value().string_value
        self.origin_mode: str = self.get_parameter('origin_mode').get_parameter_value().string_value
        self.origin_deg_list: List[float] = self.get_parameter('origin_deg_list').get_parameter_value().double_array_value
        self.slider_min_deg: float = self.get_parameter('slider_min_deg').get_parameter_value().double_value
        self.slider_max_deg: float = self.get_parameter('slider_max_deg').get_parameter_value().double_value
        self.fold_slider_min_deg: float = self.get_parameter('fold_slider_min_deg').get_parameter_value().double_value
        self.fold_slider_max_deg: float = self.get_parameter('fold_slider_max_deg').get_parameter_value().double_value
        self.tilt_slider_min_deg: float = self.get_parameter('tilt_slider_min_deg').get_parameter_value().double_value
        self.tilt_slider_max_deg: float = self.get_parameter('tilt_slider_max_deg').get_parameter_value().double_value
        self.foldgear_rev_per_motor_rev: float = self.get_parameter('foldgear_rev_per_motor_rev').get_parameter_value().double_value
        self.tilt_gain: float = self.get_parameter('tilt_gain').get_parameter_value().double_value
        self.tilt_sign: float = float(self.get_parameter('tilt_sign').value)
        self.tilt_diff_scale: float = self.get_parameter('tilt_diff_scale').get_parameter_value().double_value
        self.motor_signs: List[float] = list(self.get_parameter('motor_signs').get_parameter_value().double_array_value)
        self.publish_rate_hz: float = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.command_topic: str = self.get_parameter('command_topic').get_parameter_value().string_value
        self.state_topic: str = self.get_parameter('state_topic').get_parameter_value().string_value
        self.psi_fixed_deg: float = float(self.get_parameter('psi_fixed_deg').value)
        self.angles_topic: str = str(self.get_parameter('angles_topic').value)
        self.debug_print_commands: bool = bool(self.get_parameter('debug_print_commands').value)
        self.debug_print_period_s: float = float(self.get_parameter('debug_print_period_s').value)

        if len(self.motor_ids) != 2:
            self.get_logger().error(f'control_scheme=fold_tilt expects exactly 2 motors, got motor_ids={self.motor_ids}')
        if len(self.motor_signs) != len(self.motor_ids):
            # 不足分は 1.0 で埋める
            if len(self.motor_signs) == 0:
                self.motor_signs = [1.0] * len(self.motor_ids)
            else:
                self.motor_signs = (self.motor_signs + [1.0] * len(self.motor_ids))[: len(self.motor_ids)]
        
        # profile_vel_deg_s と profile_acc_deg_ss はスカラーまたは配列
        # パラメータの型を安全に判定（例外処理を使用）
        profile_vel_param_value = self.get_parameter('profile_vel_deg_s').get_parameter_value()
        
        # まず配列として試す
        try:
            vel_array = profile_vel_param_value.double_array_value
            if len(vel_array) > 0:
                self.profile_vel_deg_s: List[float] = list(vel_array)
            else:
                # 空配列の場合はデフォルト値を使用
                self.profile_vel_deg_s: List[float] = [100.0] * len(self.motor_ids)
                self.get_logger().warn('profile_vel_deg_s is empty array, using default 100.0 deg/s')
        except (AttributeError, TypeError):
            # 配列でない場合はスカラー値として読み込む
            try:
                vel_scalar = profile_vel_param_value.double_value
                self.profile_vel_deg_s: List[float] = [vel_scalar] * len(self.motor_ids)
            except (AttributeError, TypeError):
                # どちらでもない場合はデフォルト値を使用
                self.get_logger().warn('profile_vel_deg_s has unexpected type, using default 100.0 deg/s')
                self.profile_vel_deg_s: List[float] = [100.0] * len(self.motor_ids)
        
        profile_acc_param_value = self.get_parameter('profile_acc_deg_ss').get_parameter_value()
        
        # まず配列として試す
        try:
            acc_array = profile_acc_param_value.double_array_value
            if len(acc_array) > 0:
                self.profile_acc_deg_ss: List[float] = list(acc_array)
            else:
                # 空配列の場合はデフォルト値を使用
                self.profile_acc_deg_ss: List[float] = [200.0] * len(self.motor_ids)
                self.get_logger().warn('profile_acc_deg_ss is empty array, using default 200.0 deg/s^2')
        except (AttributeError, TypeError):
            # 配列でない場合はスカラー値として読み込む
            try:
                acc_scalar = profile_acc_param_value.double_value
                self.profile_acc_deg_ss: List[float] = [acc_scalar] * len(self.motor_ids)
            except (AttributeError, TypeError):
                # どちらでもない場合はデフォルト値を使用
                self.get_logger().warn('profile_acc_deg_ss has unexpected type, using default 200.0 deg/s^2')
                self.profile_acc_deg_ss: List[float] = [200.0] * len(self.motor_ids)
        
        # モータ数とパラメータの整合性チェック
        if len(self.motor_ids) != len(self.origin_deg_list):
            self.get_logger().warn(
                f'motor_ids length ({len(self.motor_ids)}) != origin_deg_list length ({len(self.origin_deg_list)}). '
                f'Using first {min(len(self.motor_ids), len(self.origin_deg_list))} values.'
            )
            self.origin_deg_list = self.origin_deg_list[:len(self.motor_ids)]
        
        # 内部状態
        self.origin_deg_list_actual: List[Optional[float]] = [None] * len(self.motor_ids)
        self.current_positions_deg: List[Optional[float]] = [None] * len(self.motor_ids)
        # control_scheme=motor_delta: motorごとの delta_deg
        # control_scheme=fold_tilt: [fold_deg, tilt_deg] を格納
        self.slider_values_deg: List[float] = [0.0] * (len(self.motor_ids) if self.control_scheme == 'motor_delta' else 2)
        self.origin_set_complete: bool = False
        self.last_command_positions_deg: List[Optional[float]] = [None] * len(self.motor_ids)
        self.last_command_motor_delta_deg: List[Optional[float]] = [None] * len(self.motor_ids)
        self._last_print_time_sec: float = 0.0
        
        # スレッドセーフな共有変数用ロック
        self.lock = threading.Lock()
        
        # サブスクライバー
        self.states_subscription = self.create_subscription(
            DxlStates,
            self.state_topic,
            self.states_callback,
            10
        )
        
        # パブリッシャー
        self.command_publisher = self.create_publisher(
            DxlCommandsX,
            self.command_topic,
            10
        )

        # 3Dプレビュー連携: 角度(phi, psi, theta)[deg] を publish
        self.angles_publisher = self.create_publisher(
            Float64MultiArray,
            self.angles_topic,
            10
        )
        self._warned_angles_unavailable = False
        
        # タイマー（定期的にコマンドを送信）
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback
        )

        # 3Dプレビュー連携用: 角度(phi/psi/theta)はモータ制御状態と独立にpublishする
        # （Dynamixel未接続でもプレビューを更新できるようにする）
        self.angles_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.angles_timer_callback
        )
        
        # origin_modeがstartupの場合、初回受信時に原点を設定
        if self.origin_mode == 'startup':
            self.get_logger().info('Origin mode: startup - will set origin from first received states')
        else:
            # fixedモードの場合、origin_deg_listをそのまま使用
            with self.lock:
                self.origin_deg_list_actual = list(self.origin_deg_list)
                self.origin_set_complete = True
            self.get_logger().info(f'Origin mode: fixed - using origin_deg_list={self.origin_deg_list_actual}')
        
        self.get_logger().info(f'MorphDroneSliderNode initialized')
        self.get_logger().info(f'Motor IDs: {self.motor_ids}')
        self.get_logger().info(f'Control scheme: {self.control_scheme}')
        if self.control_scheme == 'fold_tilt':
            self.get_logger().info(f'Fold slider range: [{self.fold_slider_min_deg}, {self.fold_slider_max_deg}] deg')
            self.get_logger().info(f'Tilt slider range: [{self.tilt_slider_min_deg}, {self.tilt_slider_max_deg}] deg')
            self.get_logger().info(
                f'Mechanism: foldgear_rev_per_motor_rev={self.foldgear_rev_per_motor_rev}, '
                f'tilt_gain={self.tilt_gain}, tilt_sign={self.tilt_sign}, tilt_diff_scale={self.tilt_diff_scale}, motor_signs={self.motor_signs}'
            )
        else:
            self.get_logger().info(f'Slider range: [{self.slider_min_deg}, {self.slider_max_deg}] deg')
        self.get_logger().info(f'Publish rate: {self.publish_rate_hz} Hz')
        self.get_logger().info(f'Angles publish topic: {self.angles_topic} (psi_fixed_deg={self.psi_fixed_deg:.2f})')
        
        # matplotlib GUI関連の変数（後で初期化）
        self.fig = None
        self.ax3d = None
        self.sliders = []
        self.gui_initialized = False
        self._debug_text = None
        self._debug_text_last = None

        # 実測（太線）フリッカー対策: 一瞬stateが取れなくても最後の姿勢を保持する
        self.declare_parameter('current_pose_hold_s', 2.0)
        self.current_pose_hold_s: float = float(self.get_parameter('current_pose_hold_s').value)
        self._cached_current_fold_deg: float = 0.0
        self._cached_current_tilt_deg: float = 0.0
        self._cached_current_ok: bool = False
        self._cached_current_time_sec: float = 0.0

        # 3D preview artists
        # target (slider) pose: thin / translucent
        self._preview_target_body_line = None
        self._preview_target_arm_lines = []
        self._preview_target_rotor_lines = []
        self._preview_target_normal_lines = []
        # current (motor-derived) pose: thick / opaque (or gray-fixed when unavailable)
        self._preview_current_body_line = None
        self._preview_current_arm_lines = []
        self._preview_current_rotor_lines = []
        self._preview_current_normal_lines = []
        self._preview_title = None

        # Preview geometry (fixed defaults; matches visualize_morph_drone.py defaults)
        self._pv_cx = 0.035
        self._pv_cy = 0.035
        self._pv_arm_length_m = 0.12
        self._pv_rotor_radius_m = 3.5 * 0.0254
        self._pv_symmetry = "mirror_xy"
        self._pv_rotor_inflow_offset_m = 0.02
        self._pv_view_elev = 12.0
        self._pv_view_azim = 20.0

    def angles_timer_callback(self):
        """3Dプレビュー連携用に (phi, psi, theta)[deg] を定期publish（モータ状態に依存しない）。"""
        if self.control_scheme != 'fold_tilt':
            return
        try:
            with self.lock:
                phi_deg = float(self.slider_values_deg[0]) if len(self.slider_values_deg) >= 1 else 0.0
                theta_deg = float(self.slider_values_deg[1]) if len(self.slider_values_deg) >= 2 else 0.0
                psi_deg = float(self.psi_fixed_deg)
            msg = Float64MultiArray()
            msg.data = [phi_deg, psi_deg, theta_deg]
            self.angles_publisher.publish(msg)
        except Exception:
            # publish失敗は制御に影響させない
            pass
    
    def states_callback(self, msg: DxlStates):
        """Dynamixel状態のコールバック"""
        if not msg.present.id_list:
            return
        
        with self.lock:
            # 各モータの現在位置を更新
            for i, motor_id in enumerate(self.motor_ids):
                if motor_id in msg.present.id_list:
                    idx = msg.present.id_list.index(motor_id)
                    self.current_positions_deg[i] = msg.present.position_deg[idx]
                    
                    # origin_mode=startup かつ未設定の場合、初回受信時に原点を設定
                    if (self.origin_mode == 'startup' and 
                        not self.origin_set_complete and 
                        self.origin_deg_list_actual[i] is None):
                        self.origin_deg_list_actual[i] = msg.present.position_deg[idx]
                        self.get_logger().info(
                            f'Set origin for motor {motor_id}: {self.origin_deg_list_actual[i]:.2f} deg'
                        )
            
            # 全てのモータの原点が設定されたかチェック
            if (self.origin_mode == 'startup' and 
                not self.origin_set_complete and 
                all(o is not None for o in self.origin_deg_list_actual)):
                self.origin_set_complete = True
                self.get_logger().info('All motor origins set from startup positions')
    
    def timer_callback(self):
        """定期的にコマンドを送信"""
        with self.lock:
            # 原点が設定されていない場合は送信しない
            if not self.origin_set_complete:
                return

            # スライダ値から目標位置を計算
            target_positions_deg: List[float] = []
            motor_delta_deg_used: List[float] = [0.0] * len(self.motor_ids)
            if self.control_scheme == 'fold_tilt':
                # slider_values_deg = [fold_deg, tilt_deg]
                if len(self.origin_deg_list_actual) < 2 or any(o is None for o in self.origin_deg_list_actual[:2]):
                    return
                fold_deg = float(self.slider_values_deg[0])
                tilt_deg = float(self.slider_values_deg[1])

                # 仕様（ユーザ提示）:
                # motor rev -> foldgear rev = foldgear_rev_per_motor_rev
                # fold_angle_deg = avg(foldgear_rev)*360
                # tilt_angle_deg = tilt_gain * (foldgear_rev_diff)*360
                fold_rev = fold_deg / 360.0
                diff_rev = (
                    (float(self.tilt_sign) * tilt_deg) / (max(self.tilt_gain, 1e-9) * 360.0)
                ) * max(self.tilt_diff_scale, 0.0)
                g1_rev = fold_rev + 0.5 * diff_rev
                g2_rev = fold_rev - 0.5 * diff_rev
                motor1_rev = g1_rev / max(self.foldgear_rev_per_motor_rev, 1e-12)
                motor2_rev = g2_rev / max(self.foldgear_rev_per_motor_rev, 1e-12)
                motor_delta_deg = [motor1_rev * 360.0, motor2_rev * 360.0]
                motor_delta_deg_used = [motor_delta_deg[0], motor_delta_deg[1]]

                for i in range(2):
                    target_positions_deg.append(
                        float(self.origin_deg_list_actual[i]) + float(self.motor_signs[i]) * motor_delta_deg[i]
                    )
            else:
                # motor_delta: slider_values_deg はモータ毎の delta_deg
                for i in range(len(self.motor_ids)):
                    if self.origin_deg_list_actual[i] is not None:
                        motor_delta_deg_used[i] = float(self.slider_values_deg[i])
                        target_positions_deg.append(float(self.origin_deg_list_actual[i]) + motor_delta_deg_used[i])
                    else:
                        if self.current_positions_deg[i] is not None:
                            target_positions_deg.append(float(self.current_positions_deg[i]))
                        else:
                            return
            
            # コマンドメッセージを作成
            dxl_msg = DxlCommandsX()
            dxl_msg.extended_position_control.id_list = list(self.motor_ids)
            dxl_msg.extended_position_control.position_deg = target_positions_deg
            
            # profile_vel_deg_s と profile_acc_deg_ss を設定
            # 配列の長さがモータ数と一致しない場合は最初の値を繰り返し使用
            vel_list = []
            acc_list = []
            for i in range(len(self.motor_ids)):
                # 空配列チェックを追加
                if len(self.profile_vel_deg_s) > 0:
                    vel_val = self.profile_vel_deg_s[i] if i < len(self.profile_vel_deg_s) else self.profile_vel_deg_s[0]
                else:
                    vel_val = 100.0  # デフォルト値
                vel_list.append(vel_val)
                
                if len(self.profile_acc_deg_ss) > 0:
                    acc_val = self.profile_acc_deg_ss[i] if i < len(self.profile_acc_deg_ss) else self.profile_acc_deg_ss[0]
                else:
                    acc_val = 200.0  # デフォルト値
                acc_list.append(acc_val)
            
            dxl_msg.extended_position_control.profile_vel_deg_s = vel_list
            dxl_msg.extended_position_control.profile_acc_deg_ss = acc_list
            
            self.command_publisher.publish(dxl_msg)

            # デバッグ用に最後のコマンドを保持
            self.last_command_positions_deg = list(target_positions_deg)
            self.last_command_motor_delta_deg = list(motor_delta_deg_used)

            # 端末ログに周期的に出す（過負荷防止）
            if self.debug_print_commands:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if (now_sec - self._last_print_time_sec) >= max(self.debug_print_period_s, 0.05):
                    self._last_print_time_sec = now_sec
                    if self.control_scheme == 'fold_tilt':
                        fold_deg = float(self.slider_values_deg[0])
                        tilt_deg = float(self.slider_values_deg[1])
                        self.get_logger().info(
                            f'cmd fold={fold_deg:.2f}deg tilt={tilt_deg:.2f}deg | '
                            f'motor_delta_deg={motor_delta_deg_used} | '
                            f'target_deg={target_positions_deg}'
                        )
                    else:
                        self.get_logger().info(
                            f'cmd motor_delta_deg={motor_delta_deg_used} | target_deg={target_positions_deg}'
                        )
    
    def init_gui(self):
        """matplotlib GUIの初期化（メインスレッドで実行）"""
        # Ensure 3D projection is registered (Axes3D import side-effect).
        try:
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as e:
            self.get_logger().error(
                "Failed to import mpl_toolkits.mplot3d.Axes3D (3D projection unavailable). "
                "This is often caused by mixed matplotlib installs (e.g., ~/.local vs venv). "
                "If you use venv, try: export PYTHONNOUSERSITE=1. "
                "matplotlib=%s (%s) error=%r",
                getattr(mpl, "__version__", "unknown"),
                getattr(mpl, "__file__", "unknown"),
                e,
            )
            raise

        # matplotlibのバックエンドを設定（GUI環境用）
        plt.ion()  # インタラクティブモード

        # 図の作成（1ウィンドウ: 3Dプレビュー + スライダ）
        self.fig = plt.figure(figsize=(10.5, 8.0))
        # 3D axis occupies top area; sliders occupy bottom area
        self.ax3d = self.fig.add_axes([0.05, 0.30, 0.90, 0.66], projection="3d")
        self.ax3d.set_xlabel("x [m]")
        self.ax3d.set_ylabel("y [m]")
        self.ax3d.set_zlabel("z [m]")
        try:
            self.ax3d.view_init(elev=float(self._pv_view_elev), azim=float(self._pv_view_azim))
        except Exception:
            pass
        
        # スライダの配置（下部）
        slider_height = 0.03
        slider_spacing = 0.04
        bottom_margin = 0.05

        if self.control_scheme == 'fold_tilt':
            # 2 sliders: fold, tilt
            ax_fold = plt.axes([0.15, bottom_margin + 1 * slider_spacing, 0.7, slider_height])
            ax_tilt = plt.axes([0.15, bottom_margin + 0 * slider_spacing, 0.7, slider_height])

            s_fold = Slider(
                ax_fold,
                'Fold [deg]',
                self.fold_slider_min_deg,
                self.fold_slider_max_deg,
                valinit=0.0,
                valstep=0.1,
            )
            s_tilt = Slider(
                ax_tilt,
                'Tilt [deg]',
                self.tilt_slider_min_deg,
                self.tilt_slider_max_deg,
                valinit=0.0,
                valstep=0.1,
            )
            self.sliders = [s_fold, s_tilt]

            def _on_fold(val):
                with self.lock:
                    self.slider_values_deg[0] = float(val)

            def _on_tilt(val):
                with self.lock:
                    self.slider_values_deg[1] = float(val)

            s_fold.on_changed(_on_fold)
            s_tilt.on_changed(_on_tilt)
        else:
            for i, motor_id in enumerate(self.motor_ids):
                y_pos = bottom_margin + (len(self.motor_ids) - 1 - i) * slider_spacing
                ax_slider = plt.axes([0.15, y_pos, 0.7, slider_height])
                slider = Slider(
                    ax_slider,
                    f'Motor {motor_id} [deg]',
                    self.slider_min_deg,
                    self.slider_max_deg,
                    valinit=0.0,
                    valstep=0.1
                )
                self.sliders.append(slider)

                def make_update_callback(idx):
                    def update(val):
                        with self.lock:
                            self.slider_values_deg[idx] = float(val)
                    return update

                slider.on_changed(make_update_callback(i))

        # タイトルと説明（3D上に表示）
        self._preview_title = self.ax3d.set_title('Morph Drone (3D Preview + Control Sliders)', fontsize=12)
        self._debug_text = self.ax3d.text2D(
            0.02,
            0.02,
            'motor_commands: (waiting...)',
            transform=self.ax3d.transAxes,
            ha='left',
            va='bottom',
            fontsize=9,
            family='monospace',
        )
        self._debug_text_last = 'motor_commands: (waiting...)'

        # Initialize 3D preview artists:
        # - target: from sliders (thin/translucent)
        # - current: from motor states (thick/opaque when available, gray-fixed otherwise)
        self._init_preview_artists_pair(psi_deg=float(self.psi_fixed_deg))
        
        self.gui_initialized = True
        self.get_logger().info('Matplotlib GUI initialized')

        # matplotlib側で定期的に表示を更新（テキスト + 3Dプレビュー）
        def _update_gui_text():
            # Update debug text and 3D preview from current slider values (no motor states needed).
            try:
                self._update_preview_from_sliders()
            except Exception:
                pass
            # Update current pose from motor states if available; otherwise keep gray-fixed.
            try:
                self._update_current_preview_from_motors()
            except Exception:
                pass

            with self.lock:
                cmd = self.last_command_positions_deg
                dlt = self.last_command_motor_delta_deg
                if cmd is None or all(v is None for v in cmd):
                    s = 'motor_commands: (waiting for origin/states...)'
                else:
                    pairs = [f'id{mid}:{float(v):.2f}deg' for mid, v in zip(self.motor_ids, cmd)]
                    s = 'motor_commands: ' + ', '.join(pairs)
                    if dlt is not None and any(v is not None for v in dlt):
                        pairs_d = [f'id{mid}Δ:{float(v):.2f}deg' for mid, v in zip(self.motor_ids, dlt)]
                        s += '\\n' + 'motor_delta: ' + ', '.join(pairs_d)
                    if self.control_scheme == 'fold_tilt' and len(self.slider_values_deg) >= 2:
                        s += f'\\nfold={float(self.slider_values_deg[0]):.2f}deg tilt={float(self.slider_values_deg[1]):.2f}deg'
            if self._debug_text is not None:
                # 文字列が変わったときだけ更新（フリッカー/負荷低減）
                if s != self._debug_text_last:
                    self._debug_text_last = s
                    self._debug_text.set_text(s)

            # draw は1回にまとめる（preview更新が複数回 draw_idle するとフリッカーしやすい）
            try:
                self.fig.canvas.draw_idle()
            except Exception:
                pass

        timer = self.fig.canvas.new_timer(interval=250)
        timer.add_callback(_update_gui_text)
        timer.start()

        # Ensure immediate visual response on slider drag (best effort).
        for _s in self.sliders:
            try:
                _s.on_changed(lambda _v: self._update_preview_from_sliders())
            except Exception:
                pass

    def _preview_angles_deg(self) -> tuple[float, float, float]:
        with self.lock:
            if self.control_scheme == 'fold_tilt' and len(self.slider_values_deg) >= 2:
                phi_deg = float(self.slider_values_deg[0])
                theta_deg = float(self.slider_values_deg[1])
            else:
                phi_deg = 0.0
                theta_deg = 0.0
            psi_deg = float(self.psi_fixed_deg)
        return phi_deg, psi_deg, theta_deg

    def _init_preview_artists_pair(self, *, psi_deg: float):
        """Create both target/current artist sets."""
        if self.ax3d is None:
            return

        # Target (slider) initial = neutral
        poses_t = _compute_poses(0.0, psi_deg, 0.0, cx=self._pv_cx, cy=self._pv_cy, symmetry=self._pv_symmetry)
        self._init_preview_target_artists(poses_t)

        # Current initial = gray-fixed neutral (B)
        poses_c = _compute_poses(0.0, psi_deg, 0.0, cx=self._pv_cx, cy=self._pv_cy, symmetry=self._pv_symmetry)
        self._init_preview_current_artists(poses_c, available=False)

        # Use union bounds from target/current
        self._set_preview_limits(poses_t + poses_c)

    def _init_preview_target_artists(self, poses: list[tuple[np.ndarray, np.ndarray, np.ndarray]]):
        hinges = [p[0] for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)
        # Thin + translucent
        (self._preview_target_body_line,) = self.ax3d.plot(
            hs[:, 0], hs[:, 1], hs[:, 2], color="k", alpha=0.22, linewidth=1.0, linestyle="-", zorder=1
        )

        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        self._preview_target_arm_lines = []
        self._preview_target_rotor_lines = []
        self._preview_target_normal_lines = []
        for i, (hinge, arm_dir, rotor_n) in enumerate(poses):
            c = colors[i % len(colors)]
            p0 = hinge
            arm_tip = p0 + float(self._pv_arm_length_m) * arm_dir
            rotor_center = arm_tip + float(self._pv_rotor_inflow_offset_m) * rotor_n

            (arm_ln,) = self.ax3d.plot(
                [p0[0], arm_tip[0]],
                [p0[1], arm_tip[1]],
                [p0[2], arm_tip[2]],
                color=c,
                alpha=0.22,
                linewidth=1.6,
                linestyle="-",
                zorder=1,
            )
            self._preview_target_arm_lines.append(arm_ln)

            circ = _circle_points(center=rotor_center, normal=rotor_n, radius=float(self._pv_rotor_radius_m), n=140)
            (rot_ln,) = self.ax3d.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, alpha=0.22, linewidth=0.9, zorder=1)
            self._preview_target_rotor_lines.append(rot_ln)

            n_scale = float(self._pv_rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * rotor_n
            (n_ln,) = self.ax3d.plot(
                [rotor_center[0], p2[0]],
                [rotor_center[1], p2[1]],
                [rotor_center[2], p2[2]],
                color=c,
                alpha=0.22,
                linewidth=0.9,
                zorder=1,
            )
            self._preview_target_normal_lines.append(n_ln)

    def _init_preview_current_artists(self, poses: list[tuple[np.ndarray, np.ndarray, np.ndarray]], *, available: bool):
        hinges = [p[0] for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)
        # Thick + (opaque if available, gray-fixed if not)
        base_alpha = 1.0 if bool(available) else 0.45
        base_color = "k" if bool(available) else "gray"
        (self._preview_current_body_line,) = self.ax3d.plot(
            hs[:, 0], hs[:, 1], hs[:, 2], color=base_color, alpha=base_alpha, linewidth=1.8, linestyle="-", zorder=3
        )

        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        self._preview_current_arm_lines = []
        self._preview_current_rotor_lines = []
        self._preview_current_normal_lines = []
        for i, (hinge, arm_dir, rotor_n) in enumerate(poses):
            c = (colors[i % len(colors)] if bool(available) else "gray")
            p0 = hinge
            arm_tip = p0 + float(self._pv_arm_length_m) * arm_dir
            rotor_center = arm_tip + float(self._pv_rotor_inflow_offset_m) * rotor_n

            (arm_ln,) = self.ax3d.plot(
                [p0[0], arm_tip[0]],
                [p0[1], arm_tip[1]],
                [p0[2], arm_tip[2]],
                color=c,
                alpha=base_alpha,
                linewidth=3.2,
                linestyle="-",
                zorder=3,
            )
            self._preview_current_arm_lines.append(arm_ln)

            circ = _circle_points(center=rotor_center, normal=rotor_n, radius=float(self._pv_rotor_radius_m), n=140)
            (rot_ln,) = self.ax3d.plot(circ[:, 0], circ[:, 1], circ[:, 2], color=c, alpha=base_alpha, linewidth=1.5, zorder=3)
            self._preview_current_rotor_lines.append(rot_ln)

            n_scale = float(self._pv_rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * rotor_n
            (n_ln,) = self.ax3d.plot(
                [rotor_center[0], p2[0]],
                [rotor_center[1], p2[1]],
                [rotor_center[2], p2[2]],
                color=c,
                alpha=base_alpha,
                linewidth=1.3,
                zorder=3,
            )
            self._preview_current_normal_lines.append(n_ln)

    def _set_preview_limits(self, poses: list[tuple[np.ndarray, np.ndarray, np.ndarray]]):
        if self.ax3d is None:
            return
        pts = []
        for (hinge, arm_dir, rotor_n) in poses:
            p0 = hinge
            arm_tip = p0 + float(self._pv_arm_length_m) * arm_dir
            rotor_center = arm_tip + float(self._pv_rotor_inflow_offset_m) * rotor_n
            pts.append(p0)
            pts.append(arm_tip)
            pts.append(rotor_center)
            pts.append(rotor_center + float(self._pv_rotor_radius_m) * rotor_n)
            pts.append(rotor_center - float(self._pv_rotor_radius_m) * rotor_n)
        P = np.vstack(pts)
        pad = float(self._pv_rotor_radius_m) * 0.8
        x0, x1 = float(P[:, 0].min() - pad), float(P[:, 0].max() + pad)
        y0, y1 = float(P[:, 1].min() - pad), float(P[:, 1].max() + pad)
        z0, z1 = float(P[:, 2].min() - pad), float(P[:, 2].max() + pad)

        xc, yc, zc = 0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)
        half = max((x1 - x0) * 0.5, (y1 - y0) * 0.5, (z1 - z0) * 0.5)
        self.ax3d.set_xlim(xc - half, xc + half)
        self.ax3d.set_ylim(yc - half, yc + half)
        self.ax3d.set_zlim(zc - half, zc + half)

    def _update_preview_from_sliders(self):
        if self.ax3d is None or self._preview_target_body_line is None:
            return
        phi_deg, psi_deg, theta_deg = self._preview_angles_deg()
        poses = _compute_poses(phi_deg, psi_deg, theta_deg, cx=self._pv_cx, cy=self._pv_cy, symmetry=self._pv_symmetry)
        hinges = [p[0] for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)

        self._preview_target_body_line.set_data(hs[:, 0], hs[:, 1])
        self._preview_target_body_line.set_3d_properties(hs[:, 2])

        for i, (hinge, arm_dir, rotor_n) in enumerate(poses):
            p0 = hinge
            arm_tip = p0 + float(self._pv_arm_length_m) * arm_dir
            rotor_center = arm_tip + float(self._pv_rotor_inflow_offset_m) * rotor_n

            self._preview_target_arm_lines[i].set_data([p0[0], arm_tip[0]], [p0[1], arm_tip[1]])
            self._preview_target_arm_lines[i].set_3d_properties([p0[2], arm_tip[2]])

            circ = _circle_points(center=rotor_center, normal=rotor_n, radius=float(self._pv_rotor_radius_m), n=140)
            self._preview_target_rotor_lines[i].set_data(circ[:, 0], circ[:, 1])
            self._preview_target_rotor_lines[i].set_3d_properties(circ[:, 2])

            n_scale = float(self._pv_rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * rotor_n
            self._preview_target_normal_lines[i].set_data([rotor_center[0], p2[0]], [rotor_center[1], p2[1]])
            self._preview_target_normal_lines[i].set_3d_properties([rotor_center[2], p2[2]])

        if self._preview_title is not None:
            try:
                self._preview_title.set_text(
                    f"Morph Drone (3D Preview + Control Sliders) | phi={phi_deg:+.1f}°, psi={psi_deg:+.1f}°, theta={theta_deg:+.1f}°"
                )
            except Exception:
                pass
        # limits are set by union; keep last union update in current updater
        # draw_idle は呼び出し側（GUIタイマ）でまとめて実行する（フリッカー低減）

    def _current_angles_deg_from_motors(self) -> tuple[float, float, bool]:
        """
        Inverse kinematics (fold/tilt) from motor present positions.
        Returns (fold_deg, tilt_deg, available)
        """
        with self.lock:
            if self.control_scheme != 'fold_tilt':
                return 0.0, 0.0, False
            if not self.origin_set_complete:
                return 0.0, 0.0, False
            if len(self.current_positions_deg) < 2 or len(self.origin_deg_list_actual) < 2:
                return 0.0, 0.0, False
            if self.current_positions_deg[0] is None or self.current_positions_deg[1] is None:
                return 0.0, 0.0, False
            if self.origin_deg_list_actual[0] is None or self.origin_deg_list_actual[1] is None:
                return 0.0, 0.0, False
            s0 = float(self.motor_signs[0]) if len(self.motor_signs) > 0 else 1.0
            s1 = float(self.motor_signs[1]) if len(self.motor_signs) > 1 else 1.0
            if abs(s0) < 1e-12 or abs(s1) < 1e-12:
                return 0.0, 0.0, False

            # motor delta deg (signed)
            d0 = (float(self.current_positions_deg[0]) - float(self.origin_deg_list_actual[0])) / s0
            d1 = (float(self.current_positions_deg[1]) - float(self.origin_deg_list_actual[1])) / s1

            # motor rev -> gear rev
            m0_rev = d0 / 360.0
            m1_rev = d1 / 360.0
            g0_rev = m0_rev * float(self.foldgear_rev_per_motor_rev)
            g1_rev = m1_rev * float(self.foldgear_rev_per_motor_rev)

            fold_rev = 0.5 * (g0_rev + g1_rev)
            diff_rev = (g0_rev - g1_rev)
            fold_deg = fold_rev * 360.0
            # Forward mapping uses:
            #   diff_rev = (tilt_deg / (tilt_gain*360)) * tilt_diff_scale
            # Therefore inverse must divide by tilt_diff_scale to match UI tilt.
            tds = float(self.tilt_diff_scale) if hasattr(self, "tilt_diff_scale") else 1.0
            if abs(tds) < 1e-12:
                tds = 1.0
            ts = float(self.tilt_sign) if hasattr(self, "tilt_sign") else 1.0
            if abs(ts) < 1e-12:
                ts = 1.0
            # Forward: tilt enters as (tilt_sign * tilt_deg) -> diff_rev
            # Inverse: divide by tilt_sign
            tilt_deg = ((float(self.tilt_gain) * diff_rev * 360.0) / tds) / ts
            return float(fold_deg), float(tilt_deg), True

    def _set_current_style_available(self, available: bool):
        base_alpha = 1.0 if bool(available) else 0.45
        body_color = "k" if bool(available) else "gray"
        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        if self._preview_current_body_line is not None:
            self._preview_current_body_line.set_alpha(base_alpha)
            self._preview_current_body_line.set_color(body_color)
        for i, ln in enumerate(self._preview_current_arm_lines):
            ln.set_alpha(base_alpha)
            ln.set_color(colors[i % len(colors)] if bool(available) else "gray")
        for i, ln in enumerate(self._preview_current_rotor_lines):
            ln.set_alpha(base_alpha)
            ln.set_color(colors[i % len(colors)] if bool(available) else "gray")
        for i, ln in enumerate(self._preview_current_normal_lines):
            ln.set_alpha(base_alpha)
            ln.set_color(colors[i % len(colors)] if bool(available) else "gray")

    def _update_current_preview_from_motors(self):
        if self.ax3d is None or self._preview_current_body_line is None:
            return

        fold_deg, tilt_deg, ok = self._current_angles_deg_from_motors()
        psi_deg = float(self.psi_fixed_deg)

        # フリッカー対策:
        # - okならキャッシュ更新
        # - okでなくても、一定時間内ならキャッシュ姿勢を使う（太線がニュートラルに跳ねない）
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if ok:
            self._cached_current_fold_deg = float(fold_deg)
            self._cached_current_tilt_deg = float(tilt_deg)
            self._cached_current_ok = True
            self._cached_current_time_sec = float(now_sec)
        else:
            hold_s = max(float(self.current_pose_hold_s), 0.0)
            if self._cached_current_ok and ((now_sec - self._cached_current_time_sec) <= hold_s):
                fold_deg = float(self._cached_current_fold_deg)
                tilt_deg = float(self._cached_current_tilt_deg)
                ok = True  # 表示は継続（必要ならテキスト側でageを出す）
            else:
                # 取得不能が続く場合は「最後の描画のまま」保持し、スタイルだけ unavailable にする
                self._set_current_style_available(False)
                return

        poses = _compute_poses(fold_deg, psi_deg, tilt_deg, cx=self._pv_cx, cy=self._pv_cy, symmetry=self._pv_symmetry)
        hinges = [p[0] for p in poses]
        hs = np.array(hinges + [hinges[0]], dtype=float)

        self._set_current_style_available(ok)
        self._preview_current_body_line.set_data(hs[:, 0], hs[:, 1])
        self._preview_current_body_line.set_3d_properties(hs[:, 2])

        for i, (hinge, arm_dir, rotor_n) in enumerate(poses):
            p0 = hinge
            arm_tip = p0 + float(self._pv_arm_length_m) * arm_dir
            rotor_center = arm_tip + float(self._pv_rotor_inflow_offset_m) * rotor_n

            self._preview_current_arm_lines[i].set_data([p0[0], arm_tip[0]], [p0[1], arm_tip[1]])
            self._preview_current_arm_lines[i].set_3d_properties([p0[2], arm_tip[2]])

            circ = _circle_points(center=rotor_center, normal=rotor_n, radius=float(self._pv_rotor_radius_m), n=140)
            self._preview_current_rotor_lines[i].set_data(circ[:, 0], circ[:, 1])
            self._preview_current_rotor_lines[i].set_3d_properties(circ[:, 2])

            n_scale = float(self._pv_rotor_radius_m) * 0.8
            p2 = rotor_center + n_scale * rotor_n
            self._preview_current_normal_lines[i].set_data([rotor_center[0], p2[0]], [rotor_center[1], p2[1]])
            self._preview_current_normal_lines[i].set_3d_properties([rotor_center[2], p2[2]])

        # Keep limits stable (union of current + target)
        poses_t = _compute_poses(*self._preview_angles_deg(), cx=self._pv_cx, cy=self._pv_cy, symmetry=self._pv_symmetry)
        self._set_preview_limits(poses + poses_t)
    
    def update_slider_value(self, motor_index: int, value: float):
        """スライダ値を更新（外部から呼び出し可能）"""
        with self.lock:
            if 0 <= motor_index < len(self.slider_values_deg):
                self.slider_values_deg[motor_index] = value


def main(args=None):
    rclpy.init(args=args)
    node = MorphDroneSliderNode()
    
    # GUIを初期化（メインスレッドで実行）
    node.init_gui()
    
    # ROS2ノードを別スレッドでspin（GUIがメインスレッドを占有するため）
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    # メインスレッドはGUIのイベントループを実行
    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


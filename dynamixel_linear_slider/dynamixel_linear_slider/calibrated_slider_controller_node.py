#!/usr/bin/env python3

import math
import os
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger

from dynamixel_handler_msgs.msg import DxlStates, DxlCommandsX

import yaml


class CalibratedSliderControllerNode(Node):
    """
    非線形キャリブレーション付きスライダ制御ノード

    機能:
    - DynamixelHandler の状態 (/dynamixel/states) を購読し、指定IDの角度を取得
    - ARベースのスライダ位置 (ar_slider_position) を購読
    - 目標位置コマンド (target_position_command) [m] を購読
    - 「モータ角度[deg] ↔ 物理位置[m]」のピースワイズ線形マッピングを保持
    - マッピングを用いて推定位置 estimated_position[m] を publish
    - マッピングの逆写像で目標角度を計算し、/dynamixel/commands/x に
      extended_position_control でコマンドを送信
    - キャリブレーション点列をサービスで編集し、YAML に保存/読み込み
    """

    def __init__(self) -> None:
        super().__init__("calibrated_slider_controller_node")

        # 基本パラメータ
        self.declare_parameter("motor_id", 1)
        self.declare_parameter("rack_pitch", 0.106214)  # m/rev（初期線形モデル用）
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("control_frequency", 100.0)
        self.declare_parameter("profile_velocity_deg_s", 100.0)
        self.declare_parameter("profile_accel_deg_ss", 20.0)
        # AR マーカー値の平均化に使うサンプル数
        self.declare_parameter("ar_average_window_size", 10)
        self.declare_parameter(
            "calibration_file",
            os.path.join(
                self.get_package_share_directory("dynamixel_linear_slider"),
                "config",
                "calibration.yaml",
            ),
        )

        self.motor_id: int = self.get_parameter("motor_id").value
        self.rack_pitch: float = self.get_parameter("rack_pitch").value
        self.gear_ratio: float = self.get_parameter("gear_ratio").value
        self.control_frequency: float = self.get_parameter("control_frequency").value
        self.profile_velocity_deg_s: float = self.get_parameter(
            "profile_velocity_deg_s"
        ).value
        self.profile_accel_deg_ss: float = self.get_parameter(
            "profile_accel_deg_ss"
        ).value
        self.calibration_file: str = self.get_parameter("calibration_file").value

        # 内部状態
        self.current_motor_position_deg: float = 0.0
        self.current_motor_valid: bool = False

        self.current_ar_position_m: float = 0.0
        self.current_ar_valid: bool = False

        self.target_position_m: float = 0.0
        self.target_position_valid: bool = False

        # キャリブレーション点: List[(theta_deg, x_m)]
        self.calibration_points: List[Tuple[float, float]] = []
        self.calibration_ready: bool = False

        # movement 判定用
        self.is_moving: bool = False

        # サブスクライバ
        self.dxl_states_sub = self.create_subscription(
            DxlStates,
            "/dynamixel/states",
            self.dxl_states_callback,
            10,
        )

        self.ar_position_sub = self.create_subscription(
            Float64,
            "ar_slider_position",
            self.ar_position_callback,
            10,
        )

        self.target_position_sub = self.create_subscription(
            Float64,
            "target_position_command",
            self.target_position_callback,
            10,
        )

        # パブリッシャ
        self.estimated_position_pub = self.create_publisher(
            Float64,
            "estimated_position",
            10,
        )

        self.dxl_command_pub = self.create_publisher(
            DxlCommandsX,
            "/dynamixel/commands/x",
            10,
        )

        self.movement_status_pub = self.create_publisher(
            Bool,
            "movement_status",
            10,
        )

        # サービス
        self.clear_calib_srv = self.create_service(
            Trigger,
            "clear_calibration",
            self.clear_calibration_callback,
        )
        self.add_calib_srv = self.create_service(
            Trigger,
            "add_calibration_point",
            self.add_calibration_point_callback,
        )
        self.finalize_calib_srv = self.create_service(
            Trigger,
            "finalize_calibration",
            self.finalize_calibration_callback,
        )
        self.save_calib_srv = self.create_service(
            Trigger,
            "save_calibration",
            self.save_calibration_callback,
        )
        self.load_calib_srv = self.create_service(
            Trigger,
            "load_calibration",
            self.load_calibration_callback,
        )

        # 制御タイマ
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback,
        )

        # 起動時にキャリブレーションファイルを読み込み
        self.load_calibration_from_file(initial_load=True)

        self.get_logger().info("CalibratedSliderControllerNode initialized")
        self.get_logger().info(f"motor_id={self.motor_id}")
        self.get_logger().info(f"rack_pitch={self.rack_pitch} m/rev, gear_ratio={self.gear_ratio}")
        self.get_logger().info(
            f"profile_vel={self.profile_velocity_deg_s} deg/s, "
            f"profile_acc={self.profile_accel_deg_ss} deg/s^2"
        )
        self.get_logger().info(f"calibration_file={self.calibration_file}")

    # --- ユーティリティ ---

    def get_package_share_directory(self, package_name: str) -> str:
        """
        ament_index_cpp が無い環境でも動くよう、簡易版の share パス解決。
        本パッケージのレイアウトを前提にしている。
        """
        # このファイル: <ws>/src/drone_thrust_analyzer/dynamixel_linear_slider/dynamixel_linear_slider/...
        # share ディレクトリはインストール後に使われるが、
        # ここではソース側の config ディレクトリへの相対パスを返す。
        # calibration_file はユーザが明示的に上書きもできるので、ここでは単純にパッケージルートを返す。
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # current_dir ... dynamixel_linear_slider/dynamixel_linear_slider
        package_root = os.path.dirname(current_dir)
        return package_root

    # --- サブスクライバコールバック ---

    def dxl_states_callback(self, msg: DxlStates) -> None:
        """指定モータIDの現在角度[deg]を取得"""
        if not msg.present.id_list:
            return
        for i, motor_id in enumerate(msg.present.id_list):
            if motor_id == self.motor_id:
                self.current_motor_position_deg = msg.present.position_deg[i]
                self.current_motor_valid = True
                return

    def ar_position_callback(self, msg: Float64) -> None:
        """AR由来のスライダ位置[m]"""
        self.current_ar_position_m = msg.data
        self.current_ar_valid = True

    def target_position_callback(self, msg: Float64) -> None:
        """目標位置[m]"""
        self.target_position_m = msg.data
        self.target_position_valid = True

    # --- キャリブレーション関連 ---

    def clear_calibration_callback(self, request, response):
        self.calibration_points.clear()
        self.calibration_ready = False
        response.success = True
        response.message = "Calibration points cleared"
        self.get_logger().info("Calibration points cleared")
        return response

    def add_calibration_point_callback(self, request, response):
        if not self.current_motor_valid:
            response.success = False
            response.message = "Motor state not available"
            self.get_logger().warn("Failed to add calibration point: motor state invalid")
            return response

        if not self.current_ar_valid:
            response.success = False
            response.message = "AR slider position not available"
            self.get_logger().warn("Failed to add calibration point: AR position invalid")
            return response

        theta = float(self.current_motor_position_deg)
        x = float(self.current_ar_position_m)
        self.calibration_points.append((theta, x))
        self.calibration_ready = len(self.calibration_points) >= 2

        response.success = True
        response.message = f"Added calibration point: theta={theta:.3f} deg, x={x:.6f} m"
        self.get_logger().info(response.message)
        return response

    def finalize_calibration_callback(self, request, response):
        if len(self.calibration_points) < 2:
            response.success = False
            response.message = "Need at least 2 calibration points"
            self.get_logger().warn("Finalize calibration failed: insufficient points")
            return response

        # theta でソート
        self.calibration_points.sort(key=lambda p: p[0])
        self.calibration_ready = True

        response.success = True
        response.message = f"Calibration finalized with {len(self.calibration_points)} points"
        self.get_logger().info(response.message)
        return response

    def save_calibration_callback(self, request, response):
        try:
            self.save_calibration_to_file()
            response.success = True
            response.message = f"Calibration saved to {self.calibration_file}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save calibration: {e}"
            self.get_logger().error(response.message)
        return response

    def load_calibration_callback(self, request, response):
        success, msg = self.load_calibration_from_file(initial_load=False)
        response.success = success
        response.message = msg
        if success:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        return response

    def save_calibration_to_file(self) -> None:
        """現在のキャリブレーション点列を YAML に保存"""
        if not self.calibration_points:
            raise RuntimeError("No calibration points to save")

        data = {
            "calibration": {
                "motor_id": self.motor_id,
                "rack_pitch": float(self.rack_pitch),
                "gear_ratio": float(self.gear_ratio),
                "points": [
                    {"theta_deg": float(theta), "x_m": float(x)}
                    for theta, x in self.calibration_points
                ],
            }
        }

        os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
        with open(self.calibration_file, "w") as f:
            yaml.safe_dump(data, f)

    def load_calibration_from_file(self, initial_load: bool = False) -> Tuple[bool, str]:
        """YAMLからキャリブレーション点列を読み込み"""
        if not self.calibration_file:
            return False, "Calibration file path is empty"

        if not os.path.exists(self.calibration_file):
            msg = f"Calibration file not found: {self.calibration_file}"
            if initial_load:
                # 初回ロード時は情報としてログを出す程度
                self.get_logger().info(msg)
            return False, msg

        try:
            with open(self.calibration_file, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            msg = f"Failed to load calibration file: {e}"
            return False, msg

        calib = data.get("calibration")
        if not calib:
            return False, "No 'calibration' section in file"

        points_raw = calib.get("points", [])
        points: List[Tuple[float, float]] = []
        for p in points_raw:
            try:
                theta = float(p["theta_deg"])
                x = float(p["x_m"])
            except Exception:
                continue
            points.append((theta, x))

        if len(points) < 2:
            return False, "Calibration file does not contain enough valid points"

        points.sort(key=lambda p: p[0])
        self.calibration_points = points
        self.calibration_ready = True

        # motor_id / rack_pitch / gear_ratio はファイルの値を尊重しても良いが、
        # ここではログ表示のみに留め、パラメータからの値を優先する。
        file_motor_id = calib.get("motor_id", self.motor_id)
        file_rack_pitch = calib.get("rack_pitch", self.rack_pitch)
        file_gear_ratio = calib.get("gear_ratio", self.gear_ratio)

        msg = (
            f"Loaded calibration: {len(points)} points "
            f"(file motor_id={file_motor_id}, rack_pitch={file_rack_pitch}, gear_ratio={file_gear_ratio})"
        )
        return True, msg

    # --- マッピング関数 ---

    def theta_to_position(self, theta_deg: float) -> float:
        """
        モータ角度[deg] → 物理位置[m]
        キャリブレーションがあればピースワイズ線形、なければ線形モデル。
        """
        if self.calibration_ready and len(self.calibration_points) >= 2:
            return self._theta_to_position_piecewise(theta_deg)
        # フォールバック: 単純なリードスクリュ線形モデル
        return theta_deg * self.rack_pitch / 360.0 * self.gear_ratio

    def _theta_to_position_piecewise(self, theta_deg: float) -> float:
        pts = self.calibration_points
        # 範囲外は端の線形で外挿
        if theta_deg <= pts[0][0]:
            t0, x0 = pts[0]
            t1, x1 = pts[1]
            return self._lerp(t0, x0, t1, x1, theta_deg)
        if theta_deg >= pts[-1][0]:
            t0, x0 = pts[-2]
            t1, x1 = pts[-1]
            return self._lerp(t0, x0, t1, x1, theta_deg)

        # 中間区間
        for (t0, x0), (t1, x1) in zip(pts[:-1], pts[1:]):
            if t0 <= theta_deg <= t1:
                return self._lerp(t0, x0, t1, x1, theta_deg)

        # 理論上ここには来ないはずだが、保険として線形モデルにフォールバック
        return theta_deg * self.rack_pitch / 360.0 * self.gear_ratio

    def position_to_theta(self, x_m: float) -> float:
        """
        物理位置[m] → モータ角度[deg]
        キャリブレーションがあれば逆写像ピースワイズ線形、なければ線形モデル。
        """
        if self.calibration_ready and len(self.calibration_points) >= 2:
            return self._position_to_theta_piecewise(x_m)
        # フォールバック: 線形モデルの逆
        if self.rack_pitch == 0.0 or self.gear_ratio == 0.0:
            return 0.0
        return x_m * 360.0 / self.rack_pitch / self.gear_ratio

    def _position_to_theta_piecewise(self, x_m: float) -> float:
        pts = self.calibration_points
        xs = [x for _, x in pts]

        # 範囲外は端の線形で外挿
        if x_m <= xs[0]:
            t0, x0 = pts[0]
            t1, x1 = pts[1]
            return self._lerp(x0, t0, x1, t1, x_m)
        if x_m >= xs[-1]:
            t0, x0 = pts[-2]
            t1, x1 = pts[-1]
            return self._lerp(x0, t0, x1, t1, x_m)

        # 中間区間
        for (t0, x0), (t1, x1) in zip(pts[:-1], pts[1:]):
            if x0 <= x_m <= x1 or x1 <= x_m <= x0:
                return self._lerp(x0, t0, x1, t1, x_m)

        # 保険として線形モデルにフォールバック
        if self.rack_pitch == 0.0 or self.gear_ratio == 0.0:
            return 0.0
        return x_m * 360.0 / self.rack_pitch / self.gear_ratio

    @staticmethod
    def _lerp(x0: float, y0: float, x1: float, y1: float, x: float) -> float:
        if x1 == x0:
            return y0
        ratio = (x - x0) / (x1 - x0)
        return y0 + (y1 - y0) * ratio

    # --- 制御タイマ ---

    def control_timer_callback(self) -> None:
        # 推定位置の更新
        if self.current_motor_valid:
            estimated_pos = self.theta_to_position(self.current_motor_position_deg)
            msg = Float64()
            msg.data = estimated_pos
            self.estimated_position_pub.publish(msg)
        else:
            # モータ状態が取れていない場合は制御も行わない
            return

        # 目標位置がまだ来ていない場合は制御なし
        if not self.target_position_valid:
            self.publish_movement_status(False)
            return

        # movement 状態更新
        position_error = abs(self.target_position_m - estimated_pos)
        self.is_moving = position_error > 0.001  # 1mm以上を移動中とみなす
        self.publish_movement_status(self.is_moving)

        # 目標角度を計算してコマンド送信
        target_theta_deg = self.position_to_theta(self.target_position_m)
        self.publish_dxl_command(target_theta_deg)

    def publish_movement_status(self, is_moving: bool) -> None:
        msg = Bool()
        msg.data = is_moving
        self.movement_status_pub.publish(msg)

    def publish_dxl_command(self, target_theta_deg: float) -> None:
        dxl_msg = DxlCommandsX()
        # extended_position_control を使用（単一モータ）
        dxl_msg.extended_position_control.id_list = [self.motor_id]
        dxl_msg.extended_position_control.position_deg = [float(target_theta_deg)]
        dxl_msg.extended_position_control.profile_vel_deg_s = [
            float(self.profile_velocity_deg_s)
        ]
        dxl_msg.extended_position_control.profile_acc_deg_ss = [
            float(self.profile_accel_deg_ss)
        ]

        self.dxl_command_pub.publish(dxl_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibratedSliderControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



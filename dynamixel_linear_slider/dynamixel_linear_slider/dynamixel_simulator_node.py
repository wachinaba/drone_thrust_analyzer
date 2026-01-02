#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from dynamixel_handler_msgs.msg import DxlCommandsX, DxlStates
import math
import time


class DynamixelSimulatorNode(Node):
    """
    XシリーズDynamixelの速度制御シミュレーションノード
    
    機能:
    - 速度指令値の受信
    - 位置・速度のシミュレーション
    - センサーデータの生成
    """
    
    def __init__(self):
        super().__init__('dynamixel_simulator_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_velocity', 65.0)  # XC330-T288-Tの最大速度 (rpm)
        self.declare_parameter('max_acceleration', 130.0)  # rpm/s (2秒で最大速度に到達)
        self.declare_parameter('encoder_resolution', 4096)  # Xシリーズのエンコーダー分解能
        self.declare_parameter('gear_ratio', 1.0)  # ギア比
        self.declare_parameter('simulation_dt', 0.01)  # シミュレーション時間刻み
        self.declare_parameter('stall_torque', 0.92)  # XC330-T288-Tのストールトルク (N·m)
        self.declare_parameter('stall_current', 0.8)  # XC330-T288-Tのストール電流 (A)
        self.declare_parameter('no_load_speed', 65.0)  # XC330-T288-Tの無負荷速度 (rpm)
        self.declare_parameter('no_load_current', 0.05)  # XC330-T288-Tの無負荷電流 (A)
        
        # 抵抗力パラメータ
        self.declare_parameter('resistance_type', 'viscous')  # none, viscous, friction, time_varying
        self.declare_parameter('viscous_coefficient', 0.1)  # 粘性抵抗係数 (N·m·s/rad)
        self.declare_parameter('friction_coefficient', 0.05)  # 摩擦係数 (N·m)
        self.declare_parameter('time_varying_amplitude', 0.2)  # 時間変化抵抗の振幅 (N·m)
        self.declare_parameter('time_varying_frequency', 1.0)  # 時間変化抵抗の周波数 (Hz)
        self.declare_parameter('random_resistance_std', 0.1)  # ランダム抵抗の標準偏差 (N·m)
        
        # スライダ位置パラメータ
        self.declare_parameter('slider_total_length', 2.0)  # スライダ全長 (m)
        self.declare_parameter('slider_initial_position', 0.0)  # スライダ初期位置 (m)
        self.declare_parameter('encoder_to_slider_ratio', 0.106214 / 4096)  # エンコーダー値とスライダ位置の変換比
        self.declare_parameter('slider_position_topic', 'slider_position')  # スライダ位置トピック名
        
        self.motor_id = self.get_parameter('motor_id').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.simulation_dt = self.get_parameter('simulation_dt').value
        self.stall_torque = self.get_parameter('stall_torque').value
        self.stall_current = self.get_parameter('stall_current').value
        self.no_load_speed = self.get_parameter('no_load_speed').value
        self.no_load_current = self.get_parameter('no_load_current').value
        
        # 抵抗力パラメータの取得
        self.resistance_type = self.get_parameter('resistance_type').value
        self.viscous_coefficient = self.get_parameter('viscous_coefficient').value
        self.friction_coefficient = self.get_parameter('friction_coefficient').value
        self.time_varying_amplitude = self.get_parameter('time_varying_amplitude').value
        self.time_varying_frequency = self.get_parameter('time_varying_frequency').value
        self.random_resistance_std = self.get_parameter('random_resistance_std').value
        
        # スライダ位置パラメータの取得
        self.slider_total_length = self.get_parameter('slider_total_length').value
        self.slider_initial_position = self.get_parameter('slider_initial_position').value
        self.encoder_to_slider_ratio = self.get_parameter('encoder_to_slider_ratio').value
        self.slider_position_topic = self.get_parameter('slider_position_topic').value
        
        # シミュレーション状態
        self.current_position = 0.0  # エンコーダー値
        self.current_velocity = 0.0  # rpm
        self.target_velocity = 0.0   # rpm
        self.last_time = time.time()
        self.simulation_start_time = time.time()
        
        # スライダ位置の初期化
        self.current_slider_position = self.slider_initial_position  # mm
        
        # 抵抗力計算用
        import random
        self.random_generator = random.Random()
        
        # パブリッシャーとサブスクライバーの設定
        self.command_subscription = self.create_subscription(
            DxlCommandsX,
            '/dynamixel/commands/x',
            self.command_callback,
            10
        )
        
        # DynamixelHandler形式の状態パブリッシャー
        self.states_publisher = self.create_publisher(
            DxlStates,
            '/dynamixel/states',
            10
        )
        
        # JointStateメッセージのパブリッシャー（標準的なROS2インターフェース）
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # ステータスパブリッシャー
        self.status_publisher = self.create_publisher(
            String,
            'motor_status',
            10
        )
        
        # 抵抗力情報パブリッシャー
        self.resistance_publisher = self.create_publisher(
            Float64,
            'resistance_torque',
            10
        )
        
        # スライダ位置真値パブリッシャー（従来形式との互換性のため）
        self.slider_position_publisher = self.create_publisher(
            Float64,
            self.slider_position_topic,
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback
        )
        
        self.get_logger().info('DynamixelSimulatorNode initialized')
        self.get_logger().info(f'Motor ID: {self.motor_id}')
        self.get_logger().info(f'Max Velocity: {self.max_velocity} rpm (XC330-T288-T spec)')
        self.get_logger().info(f'Max Acceleration: {self.max_acceleration} rpm/s')
        self.get_logger().info(f'Stall Torque: {self.stall_torque} N·m')
        self.get_logger().info(f'No Load Speed: {self.no_load_speed} rpm')
        self.get_logger().info(f'Resistance Type: {self.resistance_type}')
        self.get_logger().info(f'Viscous Coefficient: {self.viscous_coefficient} N·m·s/rad')
        self.get_logger().info(f'Friction Coefficient: {self.friction_coefficient} N·m')
        self.get_logger().info(f'Slider Total Length: {self.slider_total_length} mm')
        self.get_logger().info(f'Slider Initial Position: {self.slider_initial_position} mm')
        self.get_logger().info(f'Encoder to Slider Ratio: {self.encoder_to_slider_ratio}')
    
    def command_callback(self, msg):
        """DynamixelHandler形式のコマンド受信"""
        # 速度制御コマンドの処理
        if msg.velocity_control.id_list:
            for i, id in enumerate(msg.velocity_control.id_list):
                if id == self.motor_id:
                    # deg/s から rpm に変換
                    velocity_deg_s = msg.velocity_control.velocity_deg_s[i]
                    velocity_rpm = velocity_deg_s / 6.0  # deg/s -> rpm
                    
                    # 速度制限の適用
                    target_velocity = max(-self.max_velocity, min(self.max_velocity, velocity_rpm))
                    self.target_velocity = target_velocity
                    
                    self.get_logger().debug(f'Received velocity command: {velocity_deg_s} deg/s ({velocity_rpm} rpm) -> limited to {target_velocity} rpm')
                    break
        
        # 位置制御コマンドの処理（速度制御に変換）
        elif msg.position_control.id_list:
            for i, id in enumerate(msg.position_control.id_list):
                if id == self.motor_id:
                    # 位置制御は現在未実装、速度0に設定
                    self.target_velocity = 0.0
                    self.get_logger().debug(f'Position control received but not implemented, setting velocity to 0')
                    break
        
        # トルク制御の処理
        elif msg.status.id_list:
            for i, id in enumerate(msg.status.id_list):
                if id == self.motor_id:
                    torque_enabled = msg.status.torque[i]
                    if not torque_enabled:
                        self.target_velocity = 0.0
                        self.get_logger().debug(f'Torque disabled, setting velocity to 0')
                    break
    
    def calculate_resistance_torque(self, current_time):
        """抵抗力トルクの計算"""
        resistance_torque = 0.0
        
        if self.resistance_type == 'none':
            return resistance_torque
        
        # 現在の角速度 (rad/s)
        angular_velocity_rad_s = self.current_velocity * 2.0 * math.pi / 60.0
        
        if self.resistance_type == 'viscous':
            # 粘性抵抗: 速度に比例
            resistance_torque = self.viscous_coefficient * angular_velocity_rad_s
        
        elif self.resistance_type == 'friction':
            # 摩擦抵抗: 速度の符号に依存
            if abs(angular_velocity_rad_s) > 0.01:  # 静止摩擦の閾値
                resistance_torque = self.friction_coefficient * (1.0 if angular_velocity_rad_s > 0 else -1.0)
            else:
                resistance_torque = 0.0
        
        elif self.resistance_type == 'time_varying':
            # 時間変化抵抗: 正弦波 + 粘性抵抗
            time_varying_component = self.time_varying_amplitude * math.sin(
                2.0 * math.pi * self.time_varying_frequency * current_time
            )
            viscous_component = self.viscous_coefficient * angular_velocity_rad_s
            resistance_torque = time_varying_component + viscous_component
        
        elif self.resistance_type == 'random':
            # ランダム抵抗 + 粘性抵抗
            random_component = self.random_generator.gauss(0.0, self.random_resistance_std)
            viscous_component = self.viscous_coefficient * angular_velocity_rad_s
            resistance_torque = random_component + viscous_component
        
        elif self.resistance_type == 'combined':
            # 複合抵抗: 摩擦 + 粘性 + 時間変化
            friction_component = 0.0
            if abs(angular_velocity_rad_s) > 0.01:
                friction_component = self.friction_coefficient * (1.0 if angular_velocity_rad_s > 0 else -1.0)
            
            viscous_component = self.viscous_coefficient * angular_velocity_rad_s
            
            time_varying_component = self.time_varying_amplitude * math.sin(
                2.0 * math.pi * self.time_varying_frequency * current_time
            )
            
            resistance_torque = friction_component + viscous_component + time_varying_component
        
        return resistance_torque
    
    def calculate_slider_position(self):
        """エンコーダー値からスライダ位置を計算"""
        # エンコーダー値を角度に変換 (0-4095 → 0-2π)
        angle_rad = (self.current_position / self.encoder_resolution) * 2.0 * math.pi
        
        # 角度からスライダ位置への変換
        # エンコーダー値の変化量をスライダ位置の変化量に変換
        encoder_change = self.current_position - (self.encoder_resolution / 2)  # 中心からの変化量
        slider_change = encoder_change * self.encoder_to_slider_ratio  # mm単位の変化量
        
        # スライダ位置の計算（初期位置からの相対位置）
        slider_position = self.slider_initial_position + slider_change
        
        # スライダ位置を全長範囲内に制限
        slider_position = max(0.0, min(self.slider_total_length, slider_position))
        
        return slider_position
    
    def update_simulation(self):
        """シミュレーション状態の更新"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:
            # 抵抗力トルクの計算
            simulation_time = current_time - self.simulation_start_time
            resistance_torque = self.calculate_resistance_torque(simulation_time)
            
            # 抵抗力による速度減衰の計算
            # トルクから角加速度への変換: τ = I * α → α = τ / I
            # 簡略化のため、慣性モーメントを1.0として計算
            moment_of_inertia = 1.0  # kg·m²
            angular_acceleration = -resistance_torque / moment_of_inertia  # rad/s²
            
            # 角加速度から速度変化への変換
            angular_velocity_change = angular_acceleration * dt  # rad/s
            velocity_change_rpm = angular_velocity_change * 60.0 / (2.0 * math.pi)  # rpm
            
            # 目標速度への追従（加速度制限付き）
            target_velocity_diff = self.target_velocity - self.current_velocity
            max_velocity_change = self.max_acceleration * dt
            
            if abs(target_velocity_diff) > max_velocity_change:
                if target_velocity_diff > 0:
                    target_velocity_change = max_velocity_change
                else:
                    target_velocity_change = -max_velocity_change
            else:
                target_velocity_change = target_velocity_diff
            
            # 抵抗力と目標速度の両方を考慮した速度更新
            total_velocity_change = target_velocity_change + velocity_change_rpm
            self.current_velocity += total_velocity_change
            
            # 速度制限の適用
            self.current_velocity = max(-self.max_velocity, min(self.max_velocity, self.current_velocity))
            
            # 位置の更新（rpm → エンコーダー値/秒）
            # rpmをrad/sに変換: rpm * 2π / 60
            angular_velocity_rad_s = self.current_velocity * 2.0 * math.pi / 60.0
            
            # エンコーダー値の更新
            encoder_velocity = angular_velocity_rad_s * self.encoder_resolution / (2.0 * math.pi) * self.gear_ratio
            position_change = encoder_velocity * dt
            self.current_position += position_change
            
            # エンコーダー値を範囲内に制限（0 ～ encoder_resolution-1）
            self.current_position = self.current_position % self.encoder_resolution
            
            # スライダ位置の更新
            self.current_slider_position = self.calculate_slider_position()
            
            self.last_time = current_time
    
    def control_timer_callback(self):
        """制御タイマーコールバック"""
        # シミュレーション状態の更新
        self.update_simulation()
        
        # 抵抗力トルクの計算
        simulation_time = time.time() - self.simulation_start_time
        resistance_torque = self.calculate_resistance_torque(simulation_time)
        
        # エンコーダー値をdegreeに変換
        position_deg = (self.current_position / self.encoder_resolution) * 360.0
        velocity_deg_s = self.current_velocity * 6.0  # rpm -> deg/s
        
        # 電流の計算（簡略化）
        current_ma = 50.0  # 無負荷電流
        if abs(self.current_velocity) > 0.1:
            current_ma = 50.0 + abs(self.current_velocity) * 10.0  # 負荷に応じた電流
        
        # DynamixelHandler形式の状態メッセージを作成
        states_msg = DxlStates()
        
        # presentデータ
        states_msg.present.id_list = [self.motor_id]
        states_msg.present.position_deg = [position_deg]
        states_msg.present.velocity_deg_s = [velocity_deg_s]
        states_msg.present.current_ma = [current_ma]
        
        # statusデータ
        states_msg.status.id_list = [self.motor_id]
        states_msg.status.torque = [True]  # トルク有効
        states_msg.status.error = [False]  # エラーなし
        states_msg.status.ping = [True]  # 通信正常
        states_msg.status.mode = ['velocity']  # 速度制御モード
        
        # 状態をパブリッシュ
        self.states_publisher.publish(states_msg)
        
        # 抵抗力トルクのパブリッシュ
        resistance_msg = Float64()
        resistance_msg.data = resistance_torque
        self.resistance_publisher.publish(resistance_msg)
        
        # スライダ位置真値のパブリッシュ（従来形式との互換性）
        slider_position_msg = Float64()
        slider_position_msg.data = self.current_slider_position
        self.slider_position_publisher.publish(slider_position_msg)
        
        # JointStateメッセージのパブリッシュ
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'dynamixel_{self.motor_id}']
        joint_state_msg.position = [self.current_position / self.encoder_resolution * 2.0 * math.pi]  # rad
        joint_state_msg.velocity = [self.current_velocity * 2.0 * math.pi / 60.0]  # rad/s
        joint_state_msg.effort = [resistance_torque]  # 抵抗力トルクをeffortとして使用
        self.joint_state_publisher.publish(joint_state_msg)
        
        # ステータスメッセージのパブリッシュ
        status_msg = String()
        status_msg.data = f'Position: {position_deg:.2f}°, Velocity: {velocity_deg_s:.2f} deg/s, Target: {self.target_velocity * 6.0:.2f} deg/s, Resistance: {resistance_torque:.3f} N·m, Slider: {self.current_slider_position:.3f} m'
        self.status_publisher.publish(status_msg)
    
    def get_simulation_info(self):
        """シミュレーション情報の取得"""
        simulation_time = time.time() - self.simulation_start_time
        resistance_torque = self.calculate_resistance_torque(simulation_time)
        
        return {
            'motor_id': self.motor_id,
            'current_position': self.current_position,
            'current_velocity': self.current_velocity,
            'target_velocity': self.target_velocity,
            'max_velocity': self.max_velocity,
            'max_acceleration': self.max_acceleration,
            'encoder_resolution': self.encoder_resolution,
            'stall_torque': self.stall_torque,
            'stall_current': self.stall_current,
            'no_load_speed': self.no_load_speed,
            'no_load_current': self.no_load_current,
            'resistance_type': self.resistance_type,
            'current_resistance_torque': resistance_torque,
            'viscous_coefficient': self.viscous_coefficient,
            'friction_coefficient': self.friction_coefficient,
            'time_varying_amplitude': self.time_varying_amplitude,
            'time_varying_frequency': self.time_varying_frequency,
            'current_slider_position': self.current_slider_position,
            'slider_total_length': self.slider_total_length,
            'slider_initial_position': self.slider_initial_position,
            'encoder_to_slider_ratio': self.encoder_to_slider_ratio
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
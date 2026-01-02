#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import math
import time


class PositionControllerNode(Node):
    """
    システム全体の制御を統括するメインノード
    
    機能:
    - センサーフュージョン結果の受信
    - 目標位置と推定位置の偏差計算
    - PID制御演算
    - 最終的な速度指令値の生成と送信
    """
    
    def __init__(self):
        super().__init__('position_controller_node')
        
        # パラメータの取得
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_velocity', 100.0)
        self.declare_parameter('min_velocity', -100.0)
        
        # PID制御パラメータ
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.25)
        self.declare_parameter('Kd', 0.05)
        self.declare_parameter('integral_limit', 10.0)
        self.declare_parameter('max_pid_output', 100.0)
        self.declare_parameter('min_pid_output', -100.0)
        
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_velocity = self.get_parameter('min_velocity').value
        
        # PID制御パラメータ
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.max_pid_output = self.get_parameter('max_pid_output').value
        self.min_pid_output = self.get_parameter('min_pid_output').value
        
        # 制御用の変数
        self.estimated_position = 0.0
        self.target_position = 0.0
        self.feedforward_velocity = 0.0
        
        # PID制御用の変数
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_update_time = None
        
        # データの有効性フラグ
        self.estimated_position_valid = False
        self.target_position_valid = False
        self.feedforward_velocity_valid = False
        
        # サブスクライバーの設定
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position',
            self.target_position_callback,
            10
        )
        
        self.feedforward_velocity_subscription = self.create_subscription(
            Float64,
            'feedforward_velocity',
            self.feedforward_velocity_callback,
            10
        )
        
        # パブリッシャーの設定
        self.velocity_command_publisher = self.create_publisher(
            Float64,
            'velocity_command',
            10
        )
        
        self.control_status_publisher = self.create_publisher(
            String,
            'control_status',
            10
        )
        
        # PID出力のパブリッシャー（デバッグ用）
        self.pid_output_publisher = self.create_publisher(
            Float64,
            'pid_output',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_timer_callback
        )
        
        self.get_logger().info('PositionControllerNode initialized with integrated PID control')
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.estimated_position = msg.data
        self.estimated_position_valid = True
        self.get_logger().debug(f'Estimated position: {msg.data}')
    
    def target_position_callback(self, msg):
        """目標位置のコールバック"""
        self.target_position = msg.data
        self.target_position_valid = True
        self.get_logger().debug(f'Target position: {msg.data}')
    
    def feedforward_velocity_callback(self, msg):
        """フィードフォワード速度のコールバック"""
        self.feedforward_velocity = msg.data
        self.feedforward_velocity_valid = True
        self.get_logger().debug(f'Feedforward velocity: {msg.data}')
    
    def calculate_position_error(self):
        """位置偏差を計算"""
        if not self.estimated_position_valid or not self.target_position_valid:
            return 0.0
        
        error = self.target_position - self.estimated_position
        return error
    
    def calculate_pid_output(self, error):
        """PID制御量を計算"""
        current_time = time.time()
        
        # 初回実行時の初期化
        if self.last_update_time is None:
            self.last_update_time = current_time
            self.prev_error = error
            return 0.0
        
        # 時間間隔の計算
        dt = current_time - self.last_update_time
        if dt <= 0:
            return 0.0
        
        # 積分ゲインの計算
        # フィードフォワード速度が大きいほど積分ゲインを小さくする
        adaptive_i_gain = 1.0 / (1.0 + abs(self.feedforward_velocity) * 50)
        self.get_logger().info(f'Adaptive i gain: {adaptive_i_gain}')
        
        # 積分項の計算
        self.integral_error += error * dt
        self.integral_error *= adaptive_i_gain
        
        # 積分項の制限（アンチワインドアップ）
        if self.integral_error > self.integral_limit:
            self.integral_error = self.integral_limit
        elif self.integral_error < -self.integral_limit:
            self.integral_error = -self.integral_limit
        
        # 微分項の計算
        derivative_error = (error - self.prev_error) / dt
        
        # PID制御量の計算
        # v_feedback = Kp * e + Ki * ∫e dt + Kd * de/dt
        pid_output = (self.Kp * error + 
                     self.Ki * self.integral_error * adaptive_i_gain + 
                     self.Kd * derivative_error)
        
        # 出力制限の適用
        pid_output = max(self.min_pid_output, min(self.max_pid_output, pid_output))
        
        # 状態の更新
        self.prev_error = error
        self.last_update_time = current_time
        
        # デバッグ情報の出力
        self.get_logger().info(
            f'PID: error={error:.4f}, '
            f'integral={self.integral_error:.4f}, '
            f'derivative={derivative_error:.4f}, '
            f'output={pid_output:.4f}'
        )
        
        return pid_output
    
    def calculate_velocity_command(self):
        """最終的な速度指令値を計算"""
        # 位置偏差を計算
        position_error = self.calculate_position_error()
        
        # PID制御量を計算
        feedback_velocity = self.calculate_pid_output(position_error)
        
        # フィードフォワード速度を取得
        feedforward_velocity = self.feedforward_velocity if self.feedforward_velocity_valid else 0.0
        
        # 最終的な速度指令値を計算
        # v_command = v_feedback + v_feedforward
        velocity_command = feedback_velocity + feedforward_velocity
        
        # 速度制限の適用
        velocity_command = max(self.min_velocity, min(self.max_velocity, velocity_command))
        
        return velocity_command, feedback_velocity
    
    def control_timer_callback(self):
        """制御タイマーコールバック"""
        # 最終的な速度指令値を計算
        velocity_command, pid_output = self.calculate_velocity_command()
        
        # 位置偏差を計算（デバッグ用）
        position_error = self.calculate_position_error()
        
        # 速度指令値をパブリッシュ（m/s単位）
        velocity_msg = Float64()
        velocity_msg.data = velocity_command
        self.velocity_command_publisher.publish(velocity_msg)
        
        # PID出力をパブリッシュ（デバッグ用）
        pid_msg = Float64()
        pid_msg.data = pid_output
        self.pid_output_publisher.publish(pid_msg)
        
        # 制御ステータスをパブリッシュ
        status_msg = String()
        status_msg.data = (f'Target: {self.target_position:.4f}, '
                          f'Estimated: {self.estimated_position:.4f}, '
                          f'Error: {position_error:.4f}, '
                          f'PID: {pid_output:.4f}, '
                          f'Command: {velocity_command:.4f}')
        self.control_status_publisher.publish(status_msg)
        
        # デバッグ情報の出力
        if (self.estimated_position_valid and self.target_position_valid):
            """
            self.get_logger().debug(
                f'Control: target={self.target_position:.4f}, '
                f'estimated={self.estimated_position:.4f}, '
                f'error={position_error:.4f}, '
                f'pid={pid_output:.4f}, '
                f'ff={self.feedforward_velocity:.4f}, '
                f'command={velocity_command:.4f}'
            )
            """
    
    def reset_integral(self):
        """積分項をリセット"""
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_update_time = None
        self.get_logger().info('PID integral term reset')
    
    def set_pid_gains(self, Kp, Ki, Kd):
        """PIDゲインを設定"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.get_logger().info(f'PID gains updated: Kp={Kp}, Ki={Ki}, Kd={Kd}')
    
    def get_pid_gains(self):
        """PIDゲインを取得"""
        return self.Kp, self.Ki, self.Kd
    
    def set_velocity_limits(self, max_velocity, min_velocity):
        """速度制限を設定"""
        self.max_velocity = max_velocity
        self.min_velocity = min_velocity
        self.get_logger().info(f'Velocity limits updated: max={max_velocity}, min={min_velocity}')
    
    def set_pid_output_limits(self, max_output, min_output):
        """PID出力制限を設定"""
        self.max_pid_output = max_output
        self.min_pid_output = min_output
        self.get_logger().info(f'PID output limits updated: max={max_output}, min={min_output}')
    
    def get_control_status(self):
        """制御ステータスを取得"""
        return {
            'target_position': self.target_position,
            'estimated_position': self.estimated_position,
            'position_error': self.calculate_position_error(),
            'integral_error': self.integral_error,
            'pid_gains': (self.Kp, self.Ki, self.Kd)
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = PositionControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
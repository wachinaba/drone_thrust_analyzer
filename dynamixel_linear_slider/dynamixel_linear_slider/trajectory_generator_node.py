#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class TrajectoryGeneratorNode(Node):
    """
    目標軌道の生成を行うノード
    
    機能:
    - 目標位置の設定
    - 軌道生成（直線補間、スプライン補間等）
    - フィードフォワード速度の計算
    - リアルタイム軌道修正（現在位置フィードバック統合）
    - 速度ベース軌道生成（現在位置に基づく適応的速度制御）
    """
    
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # パラメータの取得
        self.declare_parameter('target_position', 0.1)  # m（初期値）
        self.declare_parameter('max_velocity', 0.03)  # m/s
        self.declare_parameter('acceleration', 0.01)  # m/s²
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('trajectory_type', 'linear')  # linear, sine, step, velocity_based
        self.declare_parameter('feedback_gain', 0.1)  # フィードバックゲイン
        self.declare_parameter('max_position_error', 0.01)  # 最大位置偏差（m）
        
        # 速度ベース制御用パラメータ
        self.declare_parameter('velocity_control_gain', 2.0)  # 速度制御ゲイン
        self.declare_parameter('deceleration_distance', 0.02)  # 減速開始距離（m）
        self.declare_parameter('min_velocity', 0.001)  # 最小速度（m/s）
        self.declare_parameter('velocity_smoothing_factor', 0.1)  # 速度平滑化係数
        
        self.target_position = self.get_parameter('target_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.acceleration = self.get_parameter('acceleration').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.feedback_gain = self.get_parameter('feedback_gain').value
        self.max_position_error = self.get_parameter('max_position_error').value
        
        # 速度ベース制御用パラメータ
        self.velocity_control_gain = self.get_parameter('velocity_control_gain').value
        self.deceleration_distance = self.get_parameter('deceleration_distance').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.velocity_smoothing_factor = self.get_parameter('velocity_smoothing_factor').value
        
        # 軌道生成用の変数
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.start_time = None
        self.trajectory_start_time = None
        self.is_trajectory_active = False
        self.initial_position = 0.0  # 軌道開始時の位置
        
        # フィードバック制御用の変数
        self.position_error = 0.0  # 位置偏差
        self.last_position_error = 0.0  # 前回の位置偏差
        self.integrated_error = 0.0  # 積分偏差
        self.has_received_position = False  # 位置情報を受信したかどうか
        
        # 速度ベース制御用の変数
        self.target_velocity = 0.0  # 目標速度
        self.last_target_velocity = 0.0  # 前回の目標速度
        self.velocity_integral = 0.0  # 速度積分（位置計算用）
        self.last_velocity_integral = 0.0  # 前回の速度積分
        
        # サブスクライバーの設定
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position_command',
            self.target_position_callback,
            10
        )
        
        # 現在位置の受信
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        # パブリッシャーの設定
        self.target_position_publisher = self.create_publisher(
            Float64,
            'target_position',
            10
        )
        
        self.feedforward_velocity_publisher = self.create_publisher(
            Float64,
            'feedforward_velocity',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.trajectory_timer_callback
        )
        
        self.get_logger().info('TrajectoryGeneratorNode initialized with velocity-based control')
    
    def target_position_callback(self, msg):
        """目標位置コマンドのコールバック"""
        new_target_position = msg.data
        if new_target_position != self.target_position:
            self.target_position = new_target_position
            self.initial_position = self.current_position  # 現在位置を初期位置として記録
            self.trajectory_start_time = time.time()  # 軌道をリスタート
            self.integrated_error = 0.0  # 積分偏差をリセット
            self.velocity_integral = 0.0  # 速度積分をリセット
            self.get_logger().info(f'Target position updated to: {new_target_position}')
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.current_position = msg.data
        self.has_received_position = True
    
    def calculate_position_error(self, target_pos):
        """位置偏差の計算"""
        if self.has_received_position:
            self.last_position_error = self.position_error
            self.position_error = target_pos - self.current_position
            # 積分偏差の更新（ウィンドウ制限付き）
            self.integrated_error += self.position_error / self.control_frequency
            # 積分偏差の飽和
            max_integral = 0.1  # 最大積分値
            self.integrated_error = max(-max_integral, min(max_integral, self.integrated_error))
        else:
            self.position_error = 0.0
            self.integrated_error = 0.0
    
    def apply_feedback_correction(self, target_pos, feedforward_vel):
        """フィードバック補正の適用"""
        if not self.has_received_position:
            return target_pos, feedforward_vel
        
        # フィードバック補正の計算
        feedback_correction = (self.feedback_gain * self.position_error + 
                             0.1 * self.feedback_gain * self.integrated_error)
        
        # 補正された目標位置
        corrected_target_pos = target_pos + feedback_correction
        
        # 位置偏差が大きい場合は軌道を再調整
        if abs(self.position_error) > self.max_position_error:
            self.get_logger().warn(f'Large position error detected: {self.position_error:.4f} m')
        
        return corrected_target_pos, feedforward_vel
    
    def calculate_adaptive_velocity(self, current_pos, target_pos):
        """適応的速度計算（距離に基づく）"""
        if not self.has_received_position:
            return 0.0
        
        # 現在位置から目標位置までの距離
        distance = abs(target_pos - current_pos)
        
        # 距離に基づく速度計算
        if distance <= self.deceleration_distance:
            # 減速領域：距離に比例した速度
            velocity = self.max_velocity * (distance / self.deceleration_distance)
            velocity = max(velocity, self.min_velocity)  # 最小速度制限
        else:
            # 定速領域：最大速度
            velocity = self.max_velocity
        
        # 移動方向の判定
        direction = 1.0 if target_pos > current_pos else -1.0
        
        return velocity * direction
    
    def apply_velocity_feedback(self, base_velocity):
        """速度フィードバック補正の適用"""
        if not self.has_received_position:
            return base_velocity
        
        # 位置偏差に基づく速度補正
        velocity_correction = self.velocity_control_gain * self.position_error
        
        # 補正された速度
        corrected_velocity = base_velocity + velocity_correction
        
        # 速度制限
        corrected_velocity = max(-self.max_velocity, min(self.max_velocity, corrected_velocity))
        
        return corrected_velocity
    
    def smooth_velocity(self, target_velocity):
        """速度の平滑化"""
        # 指数移動平均による速度平滑化
        smoothed_velocity = (self.velocity_smoothing_factor * target_velocity + 
                           (1.0 - self.velocity_smoothing_factor) * self.last_target_velocity)
        
        self.last_target_velocity = smoothed_velocity
        return smoothed_velocity
    
    def generate_velocity_based_trajectory(self, current_time):
        """速度ベース軌道の生成"""
        if self.trajectory_start_time is None or not self.has_received_position:
            return self.current_position, 0.0
        
        # 適応的速度の計算
        base_velocity = self.calculate_adaptive_velocity(self.current_position, self.target_position)
        
        # 速度フィードバック補正の適用
        corrected_velocity = self.apply_velocity_feedback(base_velocity)
        
        # 速度の平滑化
        smoothed_velocity = self.smooth_velocity(corrected_velocity)
        
        # 速度積分による位置計算
        dt = 1.0 / self.control_frequency
        self.velocity_integral += smoothed_velocity * dt
        
        # 目標位置の計算（初期位置 + 速度積分）
        target_position = self.initial_position + self.velocity_integral
        
        return target_position, smoothed_velocity
    
    def generate_linear_trajectory(self, current_time):
        """直線軌道の生成（現在位置考慮版）"""
        if self.trajectory_start_time is None:
            return self.current_position, 0.0
        
        elapsed_time = current_time - self.trajectory_start_time
        
        # 等加速度運動による軌道生成
        # 加速 → 等速 → 減速の3段階
        
        # 加速時間の計算
        acceleration_time = self.max_velocity / self.acceleration
        
        # 加速距離の計算
        acceleration_distance = 0.5 * self.acceleration * acceleration_time ** 2
        
        # 目標位置までの総距離（初期位置からの相対距離）
        total_distance = abs(self.target_position - self.initial_position)
        
        # 移動方向の判定
        direction = 1.0 if self.target_position > self.initial_position else -1.0
        
        if total_distance <= 2 * acceleration_distance:
            # 三角形軌道（加速と減速のみ）
            acceleration_time = math.sqrt(total_distance / self.acceleration)
            max_velocity = self.acceleration * acceleration_time * direction
            deceleration_time = acceleration_time
            constant_velocity_time = 0.0  # 三角形軌道では等速時間は0
        else:
            # 台形軌道（加速 → 等速 → 減速）
            max_velocity = self.max_velocity * direction
            deceleration_time = acceleration_time
            constant_velocity_time = (total_distance - 2 * acceleration_distance) / abs(max_velocity)
        
        # 軌道の計算
        if elapsed_time <= acceleration_time:
            # 加速段階
            position = self.initial_position + 0.5 * self.acceleration * elapsed_time ** 2 * direction
            velocity = self.acceleration * elapsed_time * direction
        elif elapsed_time <= acceleration_time + constant_velocity_time:
            # 等速段階
            position = (self.initial_position + acceleration_distance * direction + 
                       max_velocity * (elapsed_time - acceleration_time))
            velocity = max_velocity
        elif elapsed_time <= acceleration_time + constant_velocity_time + deceleration_time:
            # 減速段階
            decel_time = elapsed_time - acceleration_time - constant_velocity_time
            position = (self.initial_position + acceleration_distance * direction + 
                       max_velocity * constant_velocity_time + 
                       max_velocity * decel_time - 0.5 * self.acceleration * decel_time ** 2 * direction)
            velocity = max_velocity - self.acceleration * decel_time * direction
        else:
            # 軌道完了
            position = self.target_position
            velocity = 0.0
        
        return position, velocity
    
    def generate_sine_trajectory(self, current_time):
        """正弦波軌道の生成（現在位置考慮版）"""
        if self.trajectory_start_time is None:
            return self.current_position, 0.0
        
        elapsed_time = current_time - self.trajectory_start_time
        
        # 正弦波の周期（秒）
        period = 10.0  # 10秒周期
        
        # 振幅（m）
        amplitude = 0.05  # 5cm振幅
        
        # 現在位置をオフセットとして使用
        offset = self.current_position if self.has_received_position else 0.0
        
        # 位置と速度の計算
        omega = 2 * math.pi / period
        position = offset + amplitude * math.sin(omega * elapsed_time)
        velocity = amplitude * omega * math.cos(omega * elapsed_time)
        
        return position, velocity
    
    def generate_step_trajectory(self, current_time):
        """ステップ軌道の生成（現在位置考慮版）"""
        if self.trajectory_start_time is None:
            return self.current_position, 0.0
        
        elapsed_time = current_time - self.trajectory_start_time
        
        # ステップ間隔（秒）
        step_interval = 5.0
        
        # 現在位置を基準とした相対的なステップ位置
        base_offset = self.current_position if self.has_received_position else 0.0
        relative_step_positions = [0.0, 0.05, 0.1, 0.05, 0.0]
        
        # 現在のステップを決定
        step_index = int(elapsed_time / step_interval) % len(relative_step_positions)
        
        position = base_offset + relative_step_positions[step_index]
        velocity = 0.0  # ステップ軌道では速度は0
        
        return position, velocity
    
    def trajectory_timer_callback(self):
        """軌道生成タイマーコールバック（速度ベース制御統合版）"""
        current_time = time.time()
        
        # 初回実行時の初期化
        if self.start_time is None:
            self.start_time = current_time
            self.trajectory_start_time = current_time
        
        # 軌道タイプに応じた位置と速度の計算
        if self.trajectory_type == 'linear':
            target_pos, feedforward_vel = self.generate_linear_trajectory(current_time)
        elif self.trajectory_type == 'sine':
            target_pos, feedforward_vel = self.generate_sine_trajectory(current_time)
        elif self.trajectory_type == 'step':
            target_pos, feedforward_vel = self.generate_step_trajectory(current_time)
        elif self.trajectory_type == 'velocity_based':
            target_pos, feedforward_vel = self.generate_velocity_based_trajectory(current_time)
        else:
            target_pos, feedforward_vel = 0.0, 0.0
        
        # 位置偏差の計算
        self.calculate_position_error(target_pos)
        
        # フィードバック補正の適用（velocity_based以外）
        if self.trajectory_type != 'velocity_based':
            corrected_target_pos, corrected_feedforward_vel = self.apply_feedback_correction(
                target_pos, feedforward_vel
            )
        else:
            # 速度ベース制御では位置補正は不要（速度で制御）
            corrected_target_pos, corrected_feedforward_vel = target_pos, feedforward_vel
        
        # 目標位置をパブリッシュ
        target_msg = Float64()
        target_msg.data = corrected_target_pos
        self.target_position_publisher.publish(target_msg)
        
        # フィードフォワード速度をパブリッシュ
        feedforward_msg = Float64()
        feedforward_msg.data = corrected_feedforward_vel
        self.feedforward_velocity_publisher.publish(feedforward_msg)
        
        # デバッグ情報の出力
        self.get_logger().debug(
            f'Trajectory: type={self.trajectory_type}, '
            f'target={corrected_target_pos:.4f}, '
            f'feedforward={corrected_feedforward_vel:.4f}, '
            f'error={self.position_error:.4f}, '
            f'current_pos={self.current_position:.4f}'
        )
    
    def set_target_position(self, target_position):
        """目標位置を設定（内部メソッド）"""
        self.target_position = target_position
        self.trajectory_start_time = time.time()
        self.integrated_error = 0.0  # 積分偏差をリセット
        self.velocity_integral = 0.0  # 速度積分をリセット
        self.get_logger().info(f'Target position updated to: {target_position}')
    
    def set_trajectory_type(self, trajectory_type):
        """軌道タイプを設定"""
        if trajectory_type in ['linear', 'sine', 'step', 'velocity_based']:
            self.trajectory_type = trajectory_type
            self.trajectory_start_time = time.time()
            self.integrated_error = 0.0  # 積分偏差をリセット
            self.velocity_integral = 0.0  # 速度積分をリセット
            self.get_logger().info(f'Trajectory type updated to: {trajectory_type}')
        else:
            self.get_logger().warn(f'Invalid trajectory type: {trajectory_type}')
    
    def set_velocity_limits(self, max_velocity, acceleration):
        """速度制限を設定"""
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.get_logger().info(f'Velocity limits updated: max_vel={max_velocity}, accel={acceleration}')
    
    def set_feedback_gain(self, feedback_gain):
        """フィードバックゲインを設定"""
        self.feedback_gain = feedback_gain
        self.get_logger().info(f'Feedback gain updated to: {feedback_gain}')
    
    def set_velocity_control_parameters(self, velocity_control_gain, deceleration_distance, min_velocity):
        """速度制御パラメータを設定"""
        self.velocity_control_gain = velocity_control_gain
        self.deceleration_distance = deceleration_distance
        self.min_velocity = min_velocity
        self.get_logger().info(f'Velocity control parameters updated: gain={velocity_control_gain}, decel_dist={deceleration_distance}, min_vel={min_velocity}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
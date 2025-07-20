#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class TrajectoryGeneratorNode(Node):
    """
    台形制御による軌道生成ノード
    
    機能:
    - 目標位置コマンドの受信
    - 台形制御による軌道生成
    - フィードフォワード速度の計算
    """
    
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        # パラメータの取得
        self.declare_parameter('max_velocity', 0.1)  # m/s
        self.declare_parameter('acceleration', 0.1)  # m/s²
        self.declare_parameter('control_frequency', 100.0)
        
        self.max_velocity = self.get_parameter('max_velocity').value
        self.acceleration = self.get_parameter('acceleration').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # 軌道生成用の変数
        self.target_position = 0.0
        self.current_position = 0.0
        self.trajectory_start_time = None
        self.initial_position = 0.0  # 軌道開始時の位置
        self.has_received_position = False  # 位置情報を受信したかどうか
        
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
        
        self.get_logger().info('TrajectoryGeneratorNode initialized with trapezoidal control')
    
    def target_position_callback(self, msg):
        """目標位置コマンドのコールバック"""
        new_target_position = msg.data
        if new_target_position != self.target_position:
            self.target_position = new_target_position
            # 現在位置が有効な場合のみ初期位置を更新
            if self.has_received_position:
                self.initial_position = self.current_position
            self.trajectory_start_time = time.time()  # 軌道をリスタート
            self.get_logger().info(f'Target position updated to: {new_target_position}')
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.current_position = msg.data
        self.has_received_position = True
    
    def generate_trapezoidal_trajectory(self, current_time):
        """台形軌道の生成（修正版）"""
        if self.trajectory_start_time is None:
            return self.current_position, 0.0
        
        elapsed_time = current_time - self.trajectory_start_time
        
        # 台形軌道を前提とした基本パラメータを計算
        base_acceleration_time = self.max_velocity / self.acceleration
        acceleration_distance = 0.5 * self.acceleration * base_acceleration_time ** 2
        
        # 目標位置までの総距離
        total_distance = abs(self.target_position - self.initial_position)
        
        # 移動方向の判定
        direction = 1.0 if self.target_position > self.initial_position else -1.0
        
        # total_distanceが非常に小さい場合は即座に終了
        if total_distance < 1e-6:
            return self.target_position, 0.0

        # 軌道タイプの判定とパラメータの調整
        if total_distance < 2 * acceleration_distance:
            # 三角形軌道：最高速度に達する前に減速が始まる
            # ★★★バグ修正の核心部分★★★
            # 実際の移動距離に合わせて加速時間を再計算する
            acceleration_time = math.sqrt(total_distance / self.acceleration)
            max_reached_velocity = self.acceleration * acceleration_time
            constant_velocity_time = 0.0
            trajectory_duration = 2 * acceleration_time
        else:
            # 台形軌道：パラメータ通りの加速・等速・減速を行う
            acceleration_time = base_acceleration_time
            max_reached_velocity = self.max_velocity
            constant_velocity_distance = total_distance - 2 * acceleration_distance
            constant_velocity_time = constant_velocity_distance / max_reached_velocity
            trajectory_duration = 2 * acceleration_time + constant_velocity_time
        
        # その時点での最高速度（方向を含む）
        max_velocity_with_direction = max_reached_velocity * direction

        # 軌道完了後の処理
        if elapsed_time >= trajectory_duration:
            position = self.target_position
            velocity = 0.0
        else:
            # 軌道の計算
            if elapsed_time < acceleration_time:
                # 加速段階
                position = self.initial_position + 0.5 * self.acceleration * direction * elapsed_time ** 2
                velocity = self.acceleration * direction * elapsed_time
            elif elapsed_time < acceleration_time + constant_velocity_time:
                # 等速段階
                # 加速完了時点での移動距離（絶対値）
                distance_at_accel_end = 0.5 * self.acceleration * acceleration_time ** 2
                time_in_const_vel = elapsed_time - acceleration_time
                position = (self.initial_position + distance_at_accel_end * direction + 
                            max_velocity_with_direction * time_in_const_vel)
                velocity = max_velocity_with_direction
            else:
                # 減速段階
                # 等速区間が終了する時点の情報を計算
                distance_at_accel_end = 0.5 * self.acceleration * acceleration_time ** 2
                constant_velocity_end_time = acceleration_time + constant_velocity_time
                constant_velocity_travel = max_velocity_with_direction * constant_velocity_time
                position_at_decel_start = self.initial_position + distance_at_accel_end * direction + constant_velocity_travel
                
                # 減速が始まってからの経過時間
                time_in_deceleration = elapsed_time - constant_velocity_end_time
                
                # 速度を計算 v(t) = v_start - a*t
                velocity = max_velocity_with_direction - self.acceleration * direction * time_in_deceleration
                # 位置を計算 x(t) = x_start + v_start*t - 0.5*a*t^2
                position = position_at_decel_start + (max_velocity_with_direction * time_in_deceleration - 0.5 * self.acceleration * direction * time_in_deceleration**2)

        return position, velocity
    

    
    def trajectory_timer_callback(self):
        if not self.has_received_position:
            return

        """軌道生成タイマーコールバック（台形制御版）"""
        current_time = time.time()
        
        # 初回実行時の初期化
        if self.trajectory_start_time is None:
            self.trajectory_start_time = current_time
        
        # 台形軌道の生成
        target_pos, feedforward_vel = self.generate_trapezoidal_trajectory(current_time)
        
        # 目標位置をパブリッシュ
        target_msg = Float64()
        target_msg.data = target_pos
        self.target_position_publisher.publish(target_msg)
        
        # フィードフォワード速度をパブリッシュ
        feedforward_msg = Float64()
        feedforward_msg.data = feedforward_vel
        self.feedforward_velocity_publisher.publish(feedforward_msg)
        
        # デバッグ情報の出力
        if self.trajectory_start_time is not None:
            elapsed_time = current_time - self.trajectory_start_time
            self.get_logger().debug(
                f'Trajectory: target={target_pos:.4f}, '
                f'feedforward={feedforward_vel:.4f}, '
                f'current_pos={self.current_position:.4f}, '
                f'initial_pos={self.initial_position:.4f}, '
                f'elapsed_time={elapsed_time:.2f}s, '
                f'final_target={self.target_position:.4f}'
            )
    
    def set_velocity_limits(self, max_velocity, acceleration):
        """速度制限を設定"""
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.get_logger().info(f'Velocity limits updated: max_vel={max_velocity}, accel={acceleration}')


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
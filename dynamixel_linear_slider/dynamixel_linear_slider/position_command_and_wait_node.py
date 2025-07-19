#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time


class PositionCommandAndWaitNode(Node):
    """
    位置コマンドを送信し、移動完了を待って終了するノード
    
    機能:
    - パラメータで指定された位置に移動
    - target_positionトピックの監視
    - 移動状態の遷移確認
    - 移動完了（停止）を待って終了
    - タイムアウト機能
    """
    
    def __init__(self):
        super().__init__('position_command_and_wait_node')
        
        # パラメータの取得
        self.declare_parameter('target_position', 0.05)  # 目標位置 (m)
        self.declare_parameter('wait_timeout', 30.0)     # 待機タイムアウト (秒)
        
        self.target_position = self.get_parameter('target_position').value
        self.wait_timeout = self.get_parameter('wait_timeout').value
        
        # パブリッシャーの設定
        self.target_position_publisher = self.create_publisher(
            Float64,
            'target_position_command',
            10
        )
        
        # サブスクライバーの設定
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position',
            self.target_position_callback,
            10
        )
        
        self.movement_status_subscription = self.create_subscription(
            Bool,
            'movement_status',
            self.movement_status_callback,
            10
        )
        
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        # 状態管理
        self.is_moving = False
        self.estimated_position = 0.0
        self.movement_status_valid = False
        self.estimated_position_valid = False
        self.target_position_valid = False
        self.command_sent = False
        self.node_start_time = time.time()
        
        # 移動状態遷移の管理
        self.has_movement_started = False  # 移動開始を確認したか
        self.movement_start_time = None    # 移動開始時刻
        
        # 終了フラグ
        self.should_exit = False
        
        # コマンド送信管理
        self.last_command_time = 0.0
        self.command_interval = 0.1  # 100ms間隔でコマンド送信
        
        # タイマーの設定（100Hzで監視）
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('PositionCommandAndWaitNode initialized')
        self.get_logger().info(f'Target position: {self.target_position} m')
        self.get_logger().info(f'Wait timeout: {self.wait_timeout} s')
    
    def target_position_callback(self, msg):
        """目標位置のコールバック"""
        received_position = msg.data
        
        if self.target_position_valid:
            return
        
        # 同じ位置へのリクエストの場合、即座に終了
        if abs(received_position - self.target_position) < 0.001:  # 1mm以下の差は同じとみなす
            self.get_logger().info(f'Same position requested: {received_position} m, exiting immediately')
            self.should_exit = True

        self.target_position_valid = True
    
    def movement_status_callback(self, msg):
        """移動状態のコールバック"""
        previous_moving = self.is_moving
        self.is_moving = msg.data
        self.movement_status_valid = True
        
        # 移動開始の検出
        if not previous_moving and self.is_moving:
            self.has_movement_started = True
            self.movement_start_time = time.time()
            self.get_logger().info('Movement started')
        
        # 移動終了の検出
        elif previous_moving and not self.is_moving:
            self.get_logger().info('Movement stopped')
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.estimated_position = msg.data
        self.estimated_position_valid = True
    
    def send_position_command(self):
        """位置コマンドを送信"""
        current_time = time.time()
        
        # 移動開始していない場合は定期的にコマンドを送信
        if not self.has_movement_started:
            if current_time - self.last_command_time >= self.command_interval:
                position_msg = Float64()
                position_msg.data = self.target_position
                self.target_position_publisher.publish(position_msg)
                
                self.last_command_time = current_time
                self.get_logger().debug(f'Position command sent: {self.target_position} m (waiting for movement start)')
            return
        
        # 移動開始後は初回のみコマンドを送信
        if not self.command_sent:
            position_msg = Float64()
            position_msg.data = self.target_position
            self.target_position_publisher.publish(position_msg)
            
            self.command_sent = True
            self.get_logger().info(f'Position command sent: {self.target_position} m (movement started)')
    
    def check_movement_completion(self):
        """移動完了の判定"""
        if not self.movement_status_valid:
            return False
        
        # 移動開始を確認していない場合は完了判定しない
        if not self.has_movement_started:
            return False
        
        # 移動状態がFalse（停止中）の場合、完了と判定
        if not self.is_moving:
            self.get_logger().info('Movement completed successfully')
            return True
        
        return False
    
    def check_timeout(self):
        """タイムアウトの確認"""
        current_time = time.time()
        elapsed_time = current_time - self.node_start_time
        
        if elapsed_time >= self.wait_timeout:
            self.get_logger().warn(f'Timeout reached: {elapsed_time:.1f}s')
            return True
        
        return False
    
    def timer_callback(self):
        """タイマーコールバック"""
        # 終了フラグが設定されている場合は終了
        if self.should_exit:
            self.get_logger().info('Exiting due to same position request')
            rclpy.try_shutdown()
            return
        
        # 位置コマンドの送信（移動開始まで継続）
        self.send_position_command()
        
        # タイムアウトチェック
        if self.check_timeout():
            self.get_logger().error('Node terminating due to timeout')
            rclpy.try_shutdown()
            return
        
        # 移動完了チェック
        if self.check_movement_completion():
            rclpy.try_shutdown()
            return
        
        # デバッグ情報の出力（1秒ごと）
        current_time = time.time()
        if int(current_time) % 1 == 0 and int(current_time) != getattr(self, '_last_debug_time', -1):
            self._last_debug_time = int(current_time)
            elapsed_time = current_time - self.node_start_time
            
            if self.movement_status_valid and self.estimated_position_valid:
                position_error = abs(self.target_position - self.estimated_position)
                movement_status_text = "Moving" if self.is_moving else "Stopped"
                movement_started_text = "Started" if self.has_movement_started else "Not Started"
                
                self.get_logger().info(
                    f'Time: {elapsed_time:.1f}s, '
                    f'Target: {self.target_position:.4f}m, '
                    f'Estimated: {self.estimated_position:.4f}m, '
                    f'Error: {position_error:.6f}m, '
                    f'Status: {movement_status_text}, '
                    f'Movement: {movement_started_text}'
                )


def main(args=None):
    rclpy.init(args=args)
    
    node = PositionCommandAndWaitNode()
    
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 終了条件のチェック
            if node.check_timeout():
                node.get_logger().error('Node terminating due to timeout')
                break
            elif node.check_movement_completion():
                break
            elif node.should_exit:
                break
                
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
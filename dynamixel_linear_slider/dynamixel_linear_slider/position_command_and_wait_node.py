#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Float64, Bool
import time
import sys


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
        self.declare_parameter('tolerance_mm', 2.0)      # 到達判定の許容誤差 (mm)
        self.declare_parameter('stable_delta_mm', 0.2)   # 推定位置の差分安定判定閾値 (mm)
        
        self.target_position = self.get_parameter('target_position').value
        self.wait_timeout = self.get_parameter('wait_timeout').value
        self.tolerance_mm = self.get_parameter('tolerance_mm').value
        self.stable_delta_mm = self.get_parameter('stable_delta_mm').value
        
        # パブリッシャーの設定
        self.target_position_publisher = self.create_publisher(
            Float64,
            'target_position_command',
            10
        )
        
        # サブスクライバーの設定（目標と推定位置のみを使用）
        self.target_position_subscription = self.create_subscription(
            Float64,
            'target_position',
            self.target_position_callback,
            10
        )
        self.estimated_position_subscription = self.create_subscription(
            Float64,
            'estimated_position',
            self.estimated_position_callback,
            10
        )
        
        # 状態管理
        self.estimated_position = 0.0
        self.estimated_position_valid = False
        self.target_position_valid = False
        self.command_sent = False
        self.node_start_time = time.time()
        self.done_future: Future = Future()
        self.exit_reason = None
        # 差分安定判定用
        self.last_estimated_position = None
        self.last_delta_abs_m = None
        
        # 到達の安定判定（連続サンプルで堅牢化）
        self.stable_hit_count = 0
        self.stable_required = 3  # 3連続でtolerance内なら到達
        
        # 終了フラグ
        self.should_exit = False
        
        # コマンド送信管理
        self.last_command_time = 0.0
        self.command_interval = 0.1  # 100ms間隔でコマンド送信（常時）
        
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
    
    def estimated_position_callback(self, msg):
        """推定位置のコールバック"""
        self.estimated_position = msg.data
        # 直近との差分（絶対値, m）を記録
        if self.last_estimated_position is not None:
            try:
                self.last_delta_abs_m = abs(self.estimated_position - self.last_estimated_position)
            except Exception:
                self.last_delta_abs_m = None
        self.last_estimated_position = self.estimated_position
        self.estimated_position_valid = True
    
    def send_position_command(self):
        """位置コマンドを送信（常時、一定間隔）"""
        current_time = time.time()
        
        if current_time - self.last_command_time >= self.command_interval:
            position_msg = Float64()
            position_msg.data = self.target_position
            self.target_position_publisher.publish(position_msg)
            
            self.last_command_time = current_time
            self.get_logger().debug(f'Position command sent: {self.target_position} m')
    
    def check_arrival(self):
        """到達の判定（推定位置ベース、安定判定あり）"""
        if not self.estimated_position_valid or not self.target_position_valid:
            return False
        error = abs(self.target_position - self.estimated_position)
        # 差分安定（停止傾向）の判定
        delta_ok = (self.last_delta_abs_m is not None) and (self.last_delta_abs_m <= self.stable_delta_mm / 1000.0)
        # 誤差閾値と差分安定の両方を満たした場合のみヒットカウント
        if error <= self.tolerance_mm / 1000.0 and delta_ok:
            self.stable_hit_count += 1
        else:
            self.stable_hit_count = 0
        if self.stable_hit_count >= self.stable_required:
            self.get_logger().info('Movement completed successfully (stable arrival)')
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
            # タイマー停止と終了処理
            try:
                self.timer.cancel()
            except Exception:
                pass
            self.exit_reason = 'same_position'
            if not self.done_future.done():
                self.done_future.set_result(True)
            return
        
        # 位置コマンドの送信（常時・一定間隔）
        self.send_position_command()
        
        # タイムアウトチェック
        if self.check_timeout():
            self.get_logger().error('Node terminating due to timeout')
            # フラグを立ててタイマー停止し、確実に終了
            self.should_exit = True
            try:
                self.timer.cancel()
            except Exception:
                pass
            self.exit_reason = 'timeout'
            if not self.done_future.done():
                self.done_future.set_result(False)
            return
        
        # 到達チェック（安定判定付き）
        if self.check_arrival():
            # フラグを立ててタイマー停止し、確実に終了
            self.should_exit = True
            try:
                self.timer.cancel()
            except Exception:
                pass
            self.exit_reason = 'arrival'
            if not self.done_future.done():
                self.done_future.set_result(True)
            return
        
        # デバッグ情報の出力（1秒ごと）
        current_time = time.time()
        if int(current_time) % 1 == 0 and int(current_time) != getattr(self, '_last_debug_time', -1):
            self._last_debug_time = int(current_time)
            elapsed_time = current_time - self.node_start_time
            
            if self.estimated_position_valid:
                position_error = abs(self.target_position - self.estimated_position)
                self.get_logger().info(
                    f'Time: {elapsed_time:.1f}s, '
                    f'Target: {self.target_position:.4f}m, '
                    f'Estimated: {self.estimated_position:.4f}m, '
                    f'Error: {position_error:.6f}m, '
                    f'Delta: {(self.last_delta_abs_m if self.last_delta_abs_m is not None else float("nan")):.6f}m, '
                    f'StableCount: {self.stable_hit_count}/{self.stable_required}'
                )


def main(args=None):
    rclpy.init(args=args)
    
    node = PositionCommandAndWaitNode()
    
    try:
        rclpy.spin_until_future_complete(node, node.done_future)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        node.get_logger().info(f'Shutting down rclpy [reason: {node.exit_reason}]')
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main() 
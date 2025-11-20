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
        self.declare_parameter('tolerance_mm', 2.0)          # 到達判定の許容誤差 (mm)
        self.declare_parameter('stable_velocity_mm_s', 1.0)  # 到達判定用の速度閾値 (mm/s)
        
        self.target_position = self.get_parameter('target_position').value
        self.wait_timeout = self.get_parameter('wait_timeout').value
        self.tolerance_mm = self.get_parameter('tolerance_mm').value
        self.stable_velocity_mm_s = self.get_parameter('stable_velocity_mm_s').value
        
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
        # 差分・速度安定判定用
        self.last_estimated_position = None
        self.last_delta_abs_m = None
        self.last_estimated_time = None
        self.estimated_velocity_m_s = None
        
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
        current_time = time.time()
        self.estimated_position = msg.data

        # 直近との差分（絶対値, m）および速度[m/s]を記録
        if self.last_estimated_position is not None and self.last_estimated_time is not None:
            try:
                delta_pos = self.estimated_position - self.last_estimated_position
                delta_t = current_time - self.last_estimated_time
                if delta_t > 0.0:
                    self.last_delta_abs_m = abs(delta_pos)
                    self.estimated_velocity_m_s = delta_pos / delta_t
            except Exception:
                self.last_delta_abs_m = None
                self.estimated_velocity_m_s = None

        self.last_estimated_position = self.estimated_position
        self.last_estimated_time = current_time
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
        """到達の判定（推定位置と推定速度ベース、安定判定あり）"""
        # 新アーキテクチャでは target_position トピックが存在しない場合があるため、
        # target_position_valid は到達判定の必須条件から外し、推定位置と推定速度のみで判定する。
        if not self.estimated_position_valid:
            return False

        error = abs(self.target_position - self.estimated_position)
        # 推定速度[m/s]が得られていない場合はまだ判定しない
        if self.estimated_velocity_m_s is None:
            return False

        vel_abs = abs(self.estimated_velocity_m_s)
        vel_threshold_m_s = self.stable_velocity_mm_s / 1000.0

        # 誤差が小さく、かつ速度も十分に小さい場合のみヒットカウント
        if (error <= self.tolerance_mm / 1000.0) and (vel_abs <= vel_threshold_m_s):
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
                vel_log = (self.estimated_velocity_m_s if self.estimated_velocity_m_s is not None else float("nan"))
                self.get_logger().info(
                    f'Time: {elapsed_time:.1f}s, '
                    f'Target: {self.target_position:.4f}m, '
                    f'Estimated: {self.estimated_position:.4f}m, '
                    f'Error: {position_error:.6f}m, '
                    f'Vel: {vel_log:.6f}m/s, '
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
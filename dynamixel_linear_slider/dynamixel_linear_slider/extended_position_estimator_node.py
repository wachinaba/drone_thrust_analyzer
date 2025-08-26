#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_handler_msgs.msg import DxlStates
import math
import time
from collections import deque


class ExtendedPositionEstimatorNode(Node):
    """
    DynamixelHandlerのExtendedPositionをベースにした推定位置ノード
    
    機能:
    - DynamixelHandlerのExtendedPosition（/dynamixel/states）を受信
    - ARマーカー位置（/ar_marker_position）で原点リセット
    - モータ速度が0でない場合は原点リセットしない
    - 1秒間のARマーカーポジション平均値で原点リセット
    - 原点リセット前は/estimated_positionを発行しない
    """
    
    def __init__(self):
        super().__init__('extended_position_estimator_node')
        
        # パラメータの取得
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('rack_pitch', 0.106214)  # m/rev
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('velocity_zero_threshold', 0.01)  # deg/s
        self.declare_parameter('ar_marker_average_duration', 1.0)  # 秒
        self.declare_parameter('ar_marker_std_threshold', 0.002)  # m（標準偏差の閾値）
        
        self.motor_id = self.get_parameter('motor_id').value
        self.rack_pitch = self.get_parameter('rack_pitch').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.velocity_zero_threshold = self.get_parameter('velocity_zero_threshold').value
        self.ar_marker_average_duration = self.get_parameter('ar_marker_average_duration').value
        self.ar_marker_std_threshold = self.get_parameter('ar_marker_std_threshold').value
        
        # 推定用の変数
        self.current_position_deg = 0.0  # 現在の位置（度）
        self.current_velocity_deg_s = 0.0  # 現在の速度（度/秒）
        self.origin_offset_deg = 0.0  # 原点オフセット（度）
        self.estimated_position_m = 0.0  # 推定位置（メートル）
        self.is_origin_set = False  # 原点が設定されているか
        
        # ARマーカー平均化用の変数
        self.ar_marker_buffer = deque()  # ARマーカーポジションのバッファ
        self.ar_marker_timestamps = deque()  # タイムスタンプのバッファ
        self.ar_marker_valid = False
        
        # データの有効性フラグ
        self.dynamixel_states_valid = False
        
        # サブスクライバーの設定
        self.dynamixel_states_subscription = self.create_subscription(
            DxlStates,
            '/dynamixel/states',
            self.dynamixel_states_callback,
            10
        )
        
        self.ar_marker_subscription = self.create_subscription(
            Float64,
            'ar_marker_position',
            self.ar_marker_callback,
            10
        )
        
        # パブリッシャーの設定
        self.estimated_position_publisher = self.create_publisher(
            Float64,
            'estimated_position',
            10
        )
        
        # デバッグ用パブリッシャー
        self.origin_offset_publisher = self.create_publisher(
            Float64,
            'origin_offset_deg',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.estimation_timer_callback
        )
        
        self.get_logger().info('ExtendedPositionEstimatorNode initialized')
        self.get_logger().info(f'Motor ID: {self.motor_id}')
        self.get_logger().info(f'Rack pitch: {self.rack_pitch} m/rev')
        self.get_logger().info(f'Gear ratio: {self.gear_ratio}')
        self.get_logger().info(f'Velocity zero threshold: {self.velocity_zero_threshold} deg/s')
        self.get_logger().info(f'AR marker average duration: {self.ar_marker_average_duration} s')
        self.get_logger().info(f'AR marker std threshold: {self.ar_marker_std_threshold} m')
    
    def dynamixel_states_callback(self, msg):
        """DynamixelHandlerの状態を受信"""
        # 指定されたモーターIDのデータを探す
        target_id = self.motor_id
        
        # presentデータから位置と速度を取得
        if msg.present.id_list:
            for i, id in enumerate(msg.present.id_list):
                if id == target_id:
                    self.current_position_deg = msg.present.position_deg[i]
                    self.current_velocity_deg_s = msg.present.velocity_deg_s[i]
                    self.dynamixel_states_valid = True
                    
                    self.get_logger().debug(
                        f'Motor {target_id}: position_deg={self.current_position_deg:.2f}, '
                        f'velocity_deg_s={self.current_velocity_deg_s:.2f}'
                    )
                    break
    
    def ar_marker_callback(self, msg):
        """ARマーカー位置のコールバック"""
        current_time = time.time()
        
        # バッファに追加
        self.ar_marker_buffer.append(msg.data)
        self.ar_marker_timestamps.append(current_time)
        
        # 古いデータを削除（指定時間より古いもの）
        while (self.ar_marker_timestamps and 
               current_time - self.ar_marker_timestamps[0] > self.ar_marker_average_duration):
            self.ar_marker_buffer.popleft()
            self.ar_marker_timestamps.popleft()
        
        self.ar_marker_valid = True
        self.get_logger().debug(f'AR marker position: {msg.data:.4f} m')
    
    def calculate_ar_marker_average(self):
        """ARマーカーポジションの平均値を計算"""
        if not self.ar_marker_buffer:
            return None, None
        
        # 平均値と標準偏差を計算
        positions = list(self.ar_marker_buffer)
        mean_position = sum(positions) / len(positions)
        
        # 標準偏差を計算
        variance = sum((x - mean_position) ** 2 for x in positions) / len(positions)
        std_deviation = math.sqrt(variance)
        
        return mean_position, std_deviation
    
    def check_origin_reset_conditions(self):
        """原点リセットの条件をチェック"""
        # DynamixelHandlerの状態が有効でない場合は原点リセットしない
        if not self.dynamixel_states_valid:
            self.get_logger().debug('Dynamixel states not valid')
            return False

        # モータ速度が0でない場合は警告を出力
        if abs(self.current_velocity_deg_s) > self.velocity_zero_threshold:
            self.get_logger().debug(
                f'Motor velocity not zero: {self.current_velocity_deg_s:.2f} deg/s, '
                f'threshold: {self.velocity_zero_threshold} deg/s'
            )
        
        # ARマーカーデータが不足している場合
        if not self.ar_marker_valid or len(self.ar_marker_buffer) < 5:  # 最低5個のデータが必要
            self.get_logger().debug('AR marker data insufficient for origin reset')
            return False
        
        # ARマーカーポジションの平均値と標準偏差を計算
        mean_position, std_deviation = self.calculate_ar_marker_average()
        if mean_position is None:
            return False
        
        # 標準偏差が閾値を超えている場合は原点リセットしない
        if std_deviation > self.ar_marker_std_threshold:
            self.get_logger().debug(
                f'AR marker std deviation too high: {std_deviation:.4f} m, '
                f'threshold: {self.ar_marker_std_threshold} m'
            )
            return False
        
        return True
    
    def perform_origin_reset(self):
        """原点リセットを実行"""
        mean_position, _ = self.calculate_ar_marker_average()
        
        # 現在の位置（度）をメートルに変換
        current_position_m = self.current_position_deg * self.rack_pitch / 360.0 * self.gear_ratio
        
        # 原点オフセットを計算（ARマーカー位置を0にするためのオフセット）
        self.origin_offset_deg = (current_position_m - mean_position) * 360.0 / self.rack_pitch / self.gear_ratio
        
        self.is_origin_set = True
        
        # 原点オフセットをパブリッシュ
        offset_msg = Float64()
        offset_msg.data = self.origin_offset_deg
        self.origin_offset_publisher.publish(offset_msg)
        
        self.get_logger().info(
            f'Origin reset performed: AR marker mean={mean_position:.4f} m, '
            f'current position={current_position_m:.4f} m, '
            f'origin offset={self.origin_offset_deg:.2f} deg'
        )
    
    def calculate_estimated_position(self):
        """推定位置を計算"""
        if not self.dynamixel_states_valid:
            return None
        
        # 原点が設定されていない場合は計算しない
        if not self.is_origin_set:
            return None
        
        # 位置をメートルに変換（原点オフセットを適用）
        adjusted_position_deg = self.current_position_deg - self.origin_offset_deg
        estimated_position_m = adjusted_position_deg * self.rack_pitch / 360.0 * self.gear_ratio
        
        return estimated_position_m
    
    def estimation_timer_callback(self):
        """推定タイマーコールバック"""
        # 原点リセットの条件をチェック
        if self.check_origin_reset_conditions() and not self.is_origin_set:
            self.perform_origin_reset()
        
        # 推定位置を計算
        estimated_position = self.calculate_estimated_position()
        
        # 推定位置をパブリッシュ
        if estimated_position is not None:
            estimated_msg = Float64()
            estimated_msg.data = estimated_position
            self.estimated_position_publisher.publish(estimated_msg)
            
            self.get_logger().info(
                f'Estimated position: {estimated_position:.4f} m, '
                f'Origin set: {self.is_origin_set}'
            )
        else:
            if not self.is_origin_set:
                self.get_logger().warn('Waiting for origin reset...')
            else:
                self.get_logger().warn('Dynamixel states not valid')


def main(args=None):
    rclpy.init(args=args)
    
    node = ExtendedPositionEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
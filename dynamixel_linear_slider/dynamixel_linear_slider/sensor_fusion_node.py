#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class SensorFusionNode(Node):
    """
    ARマーカーとオドメトリの相補フィルター処理を行うノード
    
    機能:
    - ARマーカー位置情報の受信
    - オドメトリ情報の受信
    - 相補フィルターによる位置推定
    - 推定位置の出力
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # パラメータの取得
        self.declare_parameter('alpha', 0.97)
        self.declare_parameter('drift_bias', 0.00)  # m/s
        self.declare_parameter('control_frequency', 100.0)
        
        self.alpha = self.get_parameter('alpha').value
        self.drift_bias = self.get_parameter('drift_bias').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # 相補フィルター用の変数
        self.estimated_position = 0.0  # 推定位置
        self.prev_estimated_position = 0.0  # 前回の推定位置
        self.last_ar_marker_position = None  # 最新のARマーカー位置
        self.last_odometry_position = None  # 最新のオドメトリ位置
        self.last_odometry_velocity = 0.0  # 最新のオドメトリ速度
        self.last_update_time = None  # 最後の更新時刻
        
        # データの有効性フラグ
        self.ar_marker_valid = False
        self.odometry_valid = False
        
        # サブスクライバーの設定
        self.ar_marker_subscription = self.create_subscription(
            Float64,
            'ar_marker_position',
            self.ar_marker_callback,
            10
        )
        
        self.odometry_position_subscription = self.create_subscription(
            Float64,
            'odometry_position',
            self.odometry_position_callback,
            10
        )
        
        self.odometry_velocity_subscription = self.create_subscription(
            Float64,
            'odometry_velocity',
            self.odometry_velocity_callback,
            10
        )
        
        # パブリッシャーの設定
        self.estimated_position_publisher = self.create_publisher(
            Float64,
            'estimated_position',
            10
        )
        
        # タイマーの設定
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.fusion_timer_callback
        )
        
        self.get_logger().info('SensorFusionNode initialized')
    
    def ar_marker_callback(self, msg):
        """ARマーカー位置のコールバック"""
        self.last_ar_marker_position = msg.data
        self.ar_marker_valid = True
        self.get_logger().debug(f'AR marker position: {msg.data}')
    
    def odometry_position_callback(self, msg):
        """オドメトリ位置のコールバック"""
        self.last_odometry_position = msg.data
        self.odometry_valid = True
        self.get_logger().debug(f'Odometry position: {msg.data}')
    
    def odometry_velocity_callback(self, msg):
        """オドメトリ速度のコールバック"""
        self.last_odometry_velocity = msg.data
        self.get_logger().debug(f'Odometry velocity: {msg.data}')
    
    def complementary_filter(self):
        """相補フィルターによる位置推定"""
        current_time = time.time()
        
        # 初回実行時の初期化
        if self.last_update_time is None:
            self.last_update_time = current_time
            if self.ar_marker_valid:
                self.estimated_position = self.last_ar_marker_position
                self.prev_estimated_position = self.estimated_position
            elif self.odometry_valid:
                self.estimated_position = self.last_odometry_position
                self.prev_estimated_position = self.estimated_position
            return
        
        dt = current_time - self.last_update_time
        
        # 予測ステップ（オドメトリベース）
        if self.odometry_valid:
            # オドメトリの位置変化を計算
            if self.last_odometry_position is not None:
                position_change = self.last_odometry_position - self.prev_estimated_position
            else:
                position_change = 0.0
            
            # 速度ベースの予測（ドリフトバイアスを含む）
            velocity_prediction = (self.last_odometry_velocity + self.drift_bias) * dt
            
            # 予測位置 = 前回の推定位置 + 位置変化 + 速度予測
            predicted_position = self.prev_estimated_position + velocity_prediction
        else:
            # オドメトリが利用できない場合は前回の推定位置を使用
            predicted_position = self.prev_estimated_position
        
        # 補正ステップ（ARマーカーベース）
        if self.ar_marker_valid and self.last_ar_marker_position is not None:
            # 相補フィルター: α × 予測位置 + (1-α) × ARマーカー位置
            self.estimated_position = (self.alpha * predicted_position + 
                                    (1.0 - self.alpha) * self.last_ar_marker_position)
        else:
            # ARマーカーが利用できない場合は予測位置を使用
            self.estimated_position = predicted_position
        
        # 状態の更新
        self.prev_estimated_position = self.estimated_position
        self.last_update_time = current_time
        
        # デバッグ情報の出力
        if self.ar_marker_valid and self.odometry_valid:
            self.get_logger().debug(
                f'Fusion: AR={self.last_ar_marker_position:.4f}, '
                f'ODO={self.last_odometry_position:.4f}, '
                f'EST={self.estimated_position:.4f}, '
                f'α={self.alpha}'
            )
    
    def fusion_timer_callback(self):
        """センサーフュージョンタイマーコールバック"""
        # 相補フィルターの実行
        self.complementary_filter()
        
        # 推定位置をパブリッシュ
        estimated_msg = Float64()
        estimated_msg.data = self.estimated_position
        self.estimated_position_publisher.publish(estimated_msg)
        
        # データの有効性を定期的にリセット（データが古くなった場合の対策）
        current_time = time.time()
        if self.last_update_time is not None:
            if current_time - self.last_update_time > 1.0:  # 1秒以上データがない場合
                self.ar_marker_valid = False
                self.odometry_valid = False
    
    def get_parameter_alpha(self):
        """αパラメータを取得"""
        return self.alpha
    
    def set_parameter_alpha(self, alpha):
        """αパラメータを設定"""
        if 0.5 <= alpha <= 1.0:
            self.alpha = alpha
            self.get_logger().info(f'Alpha parameter updated to: {alpha}')
        else:
            self.get_logger().warn(f'Invalid alpha value: {alpha}. Must be between 0.8 and 1.0')


def main(args=None):
    rclpy.init(args=args)
    
    node = SensorFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
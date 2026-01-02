import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class FlowSensor(Node):
    def __init__(self):
        super().__init__('flow_sensor_subscriber')
        
        # パラメータの宣言
        self.declare_parameter('seven_segment_topic', '/seven_segment/values')
        self.declare_parameter('seven_segment_timeout', 1.0)
        self.declare_parameter('enable_seven_segment', True)
        self.declare_parameter('max_elements', 4)  # 最大要素数（デフォルト4）
        
        # パラメータの取得
        self.seven_segment_topic = self.get_parameter('seven_segment_topic').value
        self.seven_segment_timeout = self.get_parameter('seven_segment_timeout').value
        self.enable_seven_segment = self.get_parameter('enable_seven_segment').value
        self.max_elements = self.get_parameter('max_elements').value
        
        # サブスクライバーの作成
        if self.enable_seven_segment:
            self.seven_segment_subscriber = self.create_subscription(
                Float64MultiArray, 
                self.seven_segment_topic, 
                self.seven_segment_callback, 
                10
            )
            self.get_logger().info(f"フローセンサーのサブスクライバーを開始: {self.seven_segment_topic}")
            self.get_logger().info(f"最大要素数: {self.max_elements}")
        else:
            self.seven_segment_subscriber = None
            self.get_logger().info("フローセンサーは無効です")
        
        # データ保存用変数
        self.latest_data = None
        self.timestamp = None
        self.valid_count = 0
        
        # コールバック関数
        self.on_sensor_update_callback = None

    def seven_segment_callback(self, msg: Float64MultiArray):
        """フローセンサーのデータを受信するコールバック"""
        if not self.enable_seven_segment:
            return
            
        # データを保存
        self.latest_data = list(msg.data)
        self.timestamp = self.get_clock().now().nanoseconds / 1e9
        self.valid_count = len([x for x in msg.data if not np.isnan(x)])
        
        # コールバック関数を呼び出し
        if self.on_sensor_update_callback:
            self.on_sensor_update_callback(self.latest_data, self.timestamp, self.valid_count)
        
        self.get_logger().debug(f"フローセンサーデータ受信: {self.valid_count}個の有効値")

    def set_on_sensor_update(self, callback):
        """センサー更新時のコールバック関数を設定"""
        self.on_sensor_update_callback = callback

    def get_latest_data(self):
        """最新のフローセンサーデータを取得"""
        return self.latest_data

    def get_timestamp(self):
        """最新データのタイムスタンプを取得"""
        return self.timestamp

    def get_valid_count(self):
        """有効な測定値の数を取得"""
        return self.valid_count

    def is_data_fresh(self):
        """データが新鮮かどうかをチェック（タイムアウト内か）"""
        if not self.enable_seven_segment or self.timestamp is None:
            return False
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        return (current_time - self.timestamp) <= self.seven_segment_timeout

    def get_wind_speed_data(self):
        """フローセンサーデータを辞書形式で取得"""
        if not self.enable_seven_segment or not self.is_data_fresh():
            # データが無効または古い場合はNaNで埋める
            return {
                'seven_segment_count': 0,
                'seven_segment_timestamp': float('nan'),
                **{f'seven_segment_value_{i}': float('nan') for i in range(self.max_elements)}
            }
        
        # 有効なデータを返す
        result = {
            'seven_segment_count': self.valid_count,
            'seven_segment_timestamp': self.timestamp,
        }
        
        # 個別の値を追加（max_elements個まで）
        for i in range(min(len(self.latest_data), self.max_elements)):
            result[f'seven_segment_value_{i}'] = self.latest_data[i]
        
        # 不足分をNaNで埋める
        for i in range(len(self.latest_data), self.max_elements):
            result[f'seven_segment_value_{i}'] = float('nan')
        
        return result

    def get_named_wind_speed_data(self, sensor_reversed: bool = False):
        """名前付き（front/rear + in/out）でフローセンサー値を返す
        
        非反転時の対応: index順に [front_in, front_out, rear_out, rear_in]
        反転時の対応:   index順に [rear_in, rear_out, front_out, front_in]
        """
        named_keys_normal = ['front_in', 'front_out', 'rear_out', 'rear_in']
        named_keys_reversed = ['rear_in', 'rear_out', 'front_out', 'front_in']
        
        if not self.enable_seven_segment or not self.is_data_fresh():
            return {
                'seven_segment_count': 0,
                'seven_segment_timestamp': float('nan'),
                'front_in': float('nan'),
                'front_out': float('nan'),
                'rear_out': float('nan'),
                'rear_in': float('nan'),
            }
        
        keys_order = named_keys_reversed if sensor_reversed else named_keys_normal
        result = {
            'seven_segment_count': self.valid_count,
            'seven_segment_timestamp': self.timestamp,
        }
        
        # indexに応じて名前付きキーへ割り当て（不足分はNaN）
        values = self.latest_data if self.latest_data is not None else []
        for i in range(4):
            value = values[i] if i < len(values) else float('nan')
            result[keys_order[i]] = value
        
        # 念のためすべてのキーを含める
        for k in ['front_in', 'front_out', 'rear_out', 'rear_in']:
            if k not in result:
                result[k] = float('nan')
        
        return result

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point
import cv2
import numpy as np
import requests
import base64
import json
from datetime import datetime
import logging
from inference.models.utils import get_roboflow_model
import threading
import time


class SevenSegmentReaderNode(Node):
    """7セグメントディスプレイ読み取りROS2ノード"""
    
    def __init__(self):
        super().__init__('seven_segment_reader_node')
        
        # パラメータの宣言（YAMLファイルから読み込み）
        self.declare_parameter('server_url', 'http://127.0.0.1:5000')
        self.declare_parameter('model_name', '7-segment-display-gxhnj')
        self.declare_parameter('model_version', '2')
        self.declare_parameter('api_key', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('overlap_threshold', 0.7)
        self.declare_parameter('processing_interval', 0.1)  # 秒
        self.declare_parameter('doi_regions', '')  # DOI領域の座標リスト（文字列）
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('log_file_prefix', 'seven_segment_detection')
        
        # パラメータの取得（型変換を明示的に実行）
        self.server_url = self.get_parameter('server_url').value
        self.model_name = self.get_parameter('model_name').value
        # model_versionを直接取得（型チェックを回避）
        try:
            model_version_param = self.get_parameter('model_version').value
            self.model_version = str(model_version_param) if model_version_param is not None else '2'
        except:
            self.model_version = '2'  # デフォルト値
        
        # APIキーをパラメータから取得
        self.api_key = self.get_parameter('api_key').value
        
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.overlap_threshold = float(self.get_parameter('overlap_threshold').value)
        self.processing_interval = float(self.get_parameter('processing_interval').value)
        # DOI領域の取得（文字列として取得して解析）
        try:
            doi_regions_str = self.get_parameter('doi_regions').value
            self.get_logger().info(f"DOI領域文字列: {doi_regions_str}")
            self.doi_regions = []
            
            if doi_regions_str and doi_regions_str.strip():
                # カンマ区切りで分割して整数に変換
                coords = [int(x.strip()) for x in doi_regions_str.split(',')]
                
                # 4要素ずつに分割して2次元配列に変換
                for i in range(0, len(coords), 4):
                    if i + 3 < len(coords):
                        region = [coords[i], coords[i+1], coords[i+2], coords[i+3]]
                        self.doi_regions.append(region)
        except Exception as e:
            self.get_logger().error(f"DOI領域の取得に失敗: {e}")
            self.doi_regions = []
        
        # DOI領域の読み込み状況をログ出力
        self.get_logger().info(f"DOI領域数: {len(self.doi_regions)}")
        if self.doi_regions:
            for i, region in enumerate(self.doi_regions):
                self.get_logger().info(f"領域 {i+1}: {region}")
        else:
            self.get_logger().warn("DOI領域が設定されていません")
        
        # Roboflowモデルの初期化
        try:
            self.get_logger().info("api_key: " + self.api_key)
            self.model = get_roboflow_model(
                model_id=f"{self.model_name}/{self.model_version}",
                api_key=self.api_key
            )
            self.get_logger().info(f"Roboflowモデル '{self.model_name}/{self.model_version}' を初期化しました")
        except Exception as e:
            self.get_logger().error(f"Roboflowモデルの初期化に失敗しました: {e}")
            self.model = None
        
        # パブリッシャーの作成
        self.detection_pub = self.create_publisher(String, 'seven_segment_detection', 10)
        self.numeric_value_pub = self.create_publisher(Float64MultiArray, 'seven_segment_values', 10)
        self.timestamp_pub = self.create_publisher(String, 'detection_timestamp', 10)
        
        # タイマーの作成
        self.timer = self.create_timer(self.processing_interval, self.process_frame)
        
        # 状態変数
        self.frame_count = 0
        self.last_detection_time = None
        
        # ログ設定
        self.setup_logging()
        
        self.get_logger().info("7セグメントディスプレイ読み取りノードが開始されました")
        self.get_logger().info(f"サーバーURL: {self.server_url}")
        self.get_logger().info(f"処理間隔: {self.processing_interval}秒")
        
    def setup_logging(self):
        """ログ設定の初期化（ファイル出力なし）"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler()  # コンソールのみ出力
            ]
        )
        self.logger = logging.getLogger(__name__)
        self.logger.info("7セグメント表示読み取りを開始します")
        
    def get_frame_with_timestamp(self):
        """Webカメラサーバーから画像とタイムスタンプを取得"""
        try:
            response = requests.get(f"{self.server_url}/frame_with_timestamp", timeout=5.0)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"フレーム取得エラー: {e}")
            return None
    
    def decode_image_from_base64(self, img_base64):
        """Base64エンコードされた画像をデコード"""
        try:
            img_data = base64.b64decode(img_base64)
            nparr = np.frombuffer(img_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            self.get_logger().error(f"画像デコードエラー: {e}")
            return None
    
    def calculate_iou(self, box1, box2):
        """2つのバウンディングボックスのIoUを計算"""
        x1_1, y1_1, w1, h1 = box1
        x1_2, y1_2, w2, h2 = box2
        
        x2_1, y2_1 = x1_1 + w1, y1_1 + h1
        x2_2, y2_2 = x1_2 + w2, y1_2 + h2
        
        x_left = max(x1_1, x1_2)
        y_top = max(y1_1, y1_2)
        x_right = min(x2_1, x2_2)
        y_bottom = min(y2_1, y2_2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - intersection_area
        
        iou = intersection_area / union_area if union_area > 0 else 0.0
        return iou
    
    def filter_overlapping_predictions(self, predictions):
        """重複するバウンディングボックスを信頼度でフィルタリング"""
        if len(predictions) <= 1:
            return predictions
        
        sorted_predictions = sorted(predictions, key=lambda x: x.confidence, reverse=True)
        filtered_predictions = []
        
        for i, pred1 in enumerate(sorted_predictions):
            should_keep = True
            
            for pred2 in filtered_predictions:
                box1 = (pred1.x - pred1.width/2, pred1.y - pred1.height/2, pred1.width, pred1.height)
                box2 = (pred2.x - pred2.width/2, pred2.y - pred2.height/2, pred2.width, pred2.height)
                
                iou = self.calculate_iou(box1, box2)
                
                if iou >= self.overlap_threshold:
                    should_keep = False
                    break
            
            if should_keep:
                filtered_predictions.append(pred1)
        
        return filtered_predictions
    
    def extract_region(self, frame, region):
        """指定された領域を抽出"""
        start_point = region[0]
        end_point = region[1]
        x1, y1 = min(start_point[0], end_point[0]), min(start_point[1], end_point[1])
        x2, y2 = max(start_point[0], end_point[0]), max(start_point[1], end_point[1])
        return frame[y1:y2, x1:x2]
    
    def read_seven_segment(self, frame, region, region_index):
        """指定された領域の7セグメント表示を読み取る"""
        if self.model is None:
            return "MODEL_NOT_LOADED"
        
        # 領域を抽出
        roi = self.extract_region(frame, region)
        if roi.size == 0:
            self.logger.warning(f"領域 {region_index}: ROIサイズが0です")
            return "ERROR"
        
        try:
            # 推論実行
            results = self.model.infer(
                image=roi,
                confidence=self.confidence_threshold,
                iou_threshold=self.iou_threshold
            )
            
            # 重複するバウンディングボックスをフィルタリング
            filtered_predictions = self.filter_overlapping_predictions(results[0].predictions)
            
            # 検出された数字を取得（小数点は除外）
            detected_digits = []
            for prediction in filtered_predictions:
                if prediction.class_name.lower() in ['dot', 'decimal', 'point', '.']:
                    continue
                detected_digits.append(prediction.class_name)
            
            # 複数の数字が検出された場合は、x座標でソート
            if len(detected_digits) > 1:
                predictions_with_x = []
                for prediction in filtered_predictions:
                    if prediction.class_name.lower() in ['dot', 'decimal', 'point', '.']:
                        continue
                    predictions_with_x.append((prediction.x, prediction.class_name))
                predictions_with_x.sort(key=lambda x: x[0])
                detected_digits = [digit for _, digit in predictions_with_x]
            
            result = ''.join(detected_digits) if detected_digits else "NO_DETECTION"
            self.logger.info(f"領域 {region_index}: 検出結果: {result}")
            
            # 検出結果を画像に描画
            self.draw_detection_results(frame, region, filtered_predictions, result, region_index)
            
            return result
            
        except Exception as e:
            self.logger.error(f"領域 {region_index}: 推論エラー: {e}")
            return "ERROR"
    
    def draw_detection_results(self, frame, region, predictions, result, region_index):
        """検出結果を画像に描画"""
        try:
            # 領域の座標を取得
            start_point = region[0]
            end_point = region[1]
            x1, y1 = min(start_point[0], end_point[0]), min(start_point[1], end_point[1])
            x2, y2 = max(start_point[0], end_point[0]), max(start_point[1], end_point[1])
            
            # 領域の境界線を描画
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 領域番号を描画
            cv2.putText(frame, f"Region {region_index}", (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 検出結果を描画
            cv2.putText(frame, f"Result: {result}", (x1, y2 + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 各検出結果のバウンディングボックスを描画
            for prediction in predictions:
                # バウンディングボックスの座標を計算
                bbox_x = int(prediction.x - prediction.width / 2)
                bbox_y = int(prediction.y - prediction.height / 2)
                bbox_w = int(prediction.width)
                bbox_h = int(prediction.height)
                
                # グローバル座標に変換
                global_x1 = x1 + bbox_x
                global_y1 = y1 + bbox_y
                global_x2 = global_x1 + bbox_w
                global_y2 = global_y1 + bbox_h
                
                # バウンディングボックスを描画
                cv2.rectangle(frame, (global_x1, global_y1), (global_x2, global_y2), (255, 0, 0), 1)
                
                # クラス名と信頼度を描画
                label = f"{prediction.class_name}: {prediction.confidence:.2f}"
                cv2.putText(frame, label, (global_x1, global_y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                
        except Exception as e:
            self.logger.error(f"描画エラー: {e}")
    
    def draw_frame_info(self, frame, server_time, delay_ms, frame_count):
        """フレーム情報を画像に描画"""
        try:
            # フレーム情報を描画
            info_text = f"Frame: {frame_count} | Delay: {delay_ms:.1f}ms"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # サーバータイムスタンプを描画
            timestamp_text = f"Server: {server_time.strftime('%H:%M:%S.%f')[:-3]}"
            cv2.putText(frame, timestamp_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
        except Exception as e:
            self.logger.error(f"フレーム情報描画エラー: {e}")
    
    def draw_doi_regions(self, frame):
        """DOI領域を画像に描画"""
        try:
            for i, region_coords in enumerate(self.doi_regions):
                # 座標を取得
                x1, y1, x2, y2 = region_coords
                
                # DOI領域の境界線を描画（緑色）
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 領域番号を描画
                cv2.putText(frame, f"DOI Region {i+1}", (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 座標情報を描画
                coord_text = f"({x1},{y1})-({x2},{y2})"
                cv2.putText(frame, coord_text, (x1, y2 + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
        except Exception as e:
            self.logger.error(f"DOI領域描画エラー: {e}")
    
    def format_seven_segment_number(self, digit_string):
        """7セグメント数字を適切な形式に変換（小数点を挿入）"""
        if digit_string in ["NO_DETECTION", "ERROR", "MODEL_NOT_LOADED"]:
            return digit_string
        
        # 数字のみを抽出
        digits_only = ''.join([c for c in digit_string if c.isdigit()])
        
        # 桁数チェック
        if len(digits_only) < 3 or len(digits_only) > 4:
            return f"INVALID_LENGTH_{len(digits_only)}"
        
        # 3桁の場合は1桁目と2桁目の間に小数点を挿入（例：147 -> 1.47）
        if len(digits_only) == 3:
            formatted = f"{digits_only[0]}.{digits_only[1]}{digits_only[2]}"
        # 4桁の場合は2桁目と3桁目の間に小数点を挿入（例：1483 -> 14.83）
        elif len(digits_only) == 4:
            formatted = f"{digits_only[0]}{digits_only[1]}.{digits_only[2]}{digits_only[3]}"
        else:
            formatted = digit_string
        
        return formatted
    
    
    
    def process_frame(self):
        """フレーム処理のメイン関数"""
        # フレーム取得
        data = self.get_frame_with_timestamp()
        if not data:
            return
        
        # 画像デコード
        frame = self.decode_image_from_base64(data["image"])
        if frame is None:
            return
        
        self.frame_count += 1
        
        # タイムスタンプ処理
        current_time = datetime.now()
        try:
            server_time = datetime.strptime(data["timestamp"], "%Y-%m-%d %H:%M:%S:%f")
        except ValueError:
            try:
                server_time = datetime.strptime(data["timestamp"], "%Y-%m-%d %H:%M:%S")
            except ValueError:
                server_time = current_time
        
        delay_ms = (current_time - server_time).total_seconds() * 1000
        
        # DOI領域が設定されていない場合はスキップ
        if not self.doi_regions:
            self.get_logger().warn("DOI領域が設定されていません。パラメータで設定してください。")
            return
        
        # 表示用フレームのコピーを作成
        display_frame = frame.copy()
        
        # フレーム情報を描画
        self.draw_frame_info(display_frame, server_time, delay_ms, self.frame_count)
        
        # DOI領域を描画
        self.draw_doi_regions(display_frame)
        
        # 各領域の7セグメント表示を読み取り
        detection_results = []
        numeric_values = []
        
        for i, region_coords in enumerate(self.doi_regions):
            # 座標をタプルに変換
            region = [(region_coords[0], region_coords[1]), (region_coords[2], region_coords[3])]
            
            # 7セグメント読み取り
            digit = self.read_seven_segment(frame, region, i+1)
            formatted_digit = self.format_seven_segment_number(digit)
            detection_results.append(formatted_digit)
            
            # 数値変換
            try:
                if formatted_digit not in ["NO_DETECTION", "ERROR", "MODEL_NOT_LOADED"] and not formatted_digit.startswith("INVALID_LENGTH"):
                    numeric_value = float(formatted_digit)
                    numeric_values.append(numeric_value)
                else:
                    numeric_values.append(float('nan'))
            except ValueError:
                numeric_values.append(float('nan'))
            
            # 検出結果をフレームに描画（新しい描画システムを使用）
            # self.draw_detection_on_frame(display_frame, region, i+1, formatted_digit)
        
        # 画像を表示
        cv2.imshow('7-Segment Detection', display_frame)
        cv2.waitKey(1)
        
        # ROSメッセージのパブリッシュ
        # 検出結果メッセージ
        detection_msg = String()
        detection_msg.data = json.dumps({
            'detections': detection_results,
            'frame_count': self.frame_count,
            'server_timestamp': data['timestamp'],
            'client_timestamp': current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'delay_ms': delay_ms
        })
        self.detection_pub.publish(detection_msg)
        
        # 数値メッセージ
        numeric_msg = Float64MultiArray()
        numeric_msg.data = numeric_values
        self.numeric_value_pub.publish(numeric_msg)
        
        # タイムスタンプメッセージ
        timestamp_msg = String()
        timestamp_msg.data = data['timestamp']
        self.timestamp_pub.publish(timestamp_msg)
        
        self.last_detection_time = current_time
        
        # ログ出力
        if self.frame_count % 10 == 0:  # 10フレームごとにログ出力
            self.get_logger().info(f"フレーム {self.frame_count}: 検出結果 = {detection_results}, 遅延 = {delay_ms:.1f}ms")


def main(args=None):
    rclpy.init(args=args)
    
    node = SevenSegmentReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ノードが終了されました")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

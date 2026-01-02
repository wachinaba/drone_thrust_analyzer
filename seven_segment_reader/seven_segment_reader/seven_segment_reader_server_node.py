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
import threading
import time
import subprocess
import os


class SevenSegmentReaderServerNode(Node):
    """7セグメントディスプレイ読み取りROS2ノード（Inference Server版）"""
    
    def __init__(self):
        super().__init__('seven_segment_reader_server_node')
        
        # パラメータの宣言（YAMLファイルから読み込み）
        self.declare_parameter('server_url', 'http://127.0.0.1:5000')
        self.declare_parameter('inference_server_url', 'http://localhost:9001')
        self.declare_parameter('model_name', '7-segment-display-gxhnj')
        self.declare_parameter('model_version', '2')
        self.declare_parameter('api_key', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('overlap_threshold', 0.7)
        self.declare_parameter('processing_interval', 0.1)  # 秒
        self.declare_parameter('doi_count', 1)  # DOI領域の数
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('log_file_prefix', 'seven_segment_detection')
        self.declare_parameter('use_gpu', True)  # GPU使用フラグ
        self.declare_parameter('gpu_device_id', 0)  # GPUデバイスID
        self.declare_parameter('show_debug_window', True)  # デバッグウィンドウ表示
        self.declare_parameter('show_bounding_boxes', True)  # バウンディングボックス表示
        
        # パラメータの取得（型変換を明示的に実行）
        self.server_url = self.get_parameter('server_url').value
        self.inference_server_url = self.get_parameter('inference_server_url').value
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
        self.doi_count = int(self.get_parameter('doi_count').value)
        self.use_gpu = bool(self.get_parameter('use_gpu').value)
        self.gpu_device_id = int(self.get_parameter('gpu_device_id').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.show_bounding_boxes = bool(self.get_parameter('show_bounding_boxes').value)
        
        # GPU設定の確認とログ出力
        self.device_info = self.setup_device()
        self.get_logger().info(f"DOI領域数: {self.doi_count}, GPU: {self.device_info['gpu_available']}, Server: {self.inference_server_url}")
        
        # APIキーの検証
        self.validate_api_key()
        
        # Inference Serverの接続確認
        self.check_inference_server()
        
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
        
        self.get_logger().info("7セグメントディスプレイ読み取りノード（Inference Server版）が開始されました")
        self.get_logger().info(f"サーバーURL: {self.server_url}")
        self.get_logger().info(f"処理間隔: {self.processing_interval}秒")
        
    def setup_device(self):
        """GPUデバイスの設定（torchを使わない方法）"""
        device_info = {
            'gpu_available': False,
            'device': 'cpu',
            'gpu_name': None,
            'gpu_memory': None
        }
        
        if self.use_gpu:
            # nvidia-smiコマンドでGPU情報を取得
            try:
                result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader,nounits'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0 and result.stdout.strip():
                    lines = result.stdout.strip().split('\n')
                    if self.gpu_device_id < len(lines):
                        gpu_info = lines[self.gpu_device_id].split(', ')
                        if len(gpu_info) >= 2:
                            device_info['gpu_available'] = True
                            device_info['device'] = f"cuda:{self.gpu_device_id}"
                            device_info['gpu_name'] = gpu_info[0].strip()
                            device_info['gpu_memory'] = f"{int(gpu_info[1]) / 1024:.1f}GB"
                            
                            self.get_logger().info(f"GPU検出: {device_info['gpu_name']}")
                            self.get_logger().info(f"GPUメモリ: {device_info['gpu_memory']}")
                        else:
                            self.get_logger().warn(f"GPU {self.gpu_device_id} の情報が不完全です")
                    else:
                        self.get_logger().warn(f"GPU {self.gpu_device_id} が見つかりません")
                else:
                    self.get_logger().warn("nvidia-smiコマンドの実行に失敗しました")
            except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.SubprocessError) as e:
                self.get_logger().warn(f"GPU検出エラー: {e}")
            
            if not device_info['gpu_available']:
                self.get_logger().warn("GPU使用が要求されましたが、GPUが利用できません。CPUを使用します。")
        else:
            self.get_logger().info("CPU使用")
        
        return device_info
    
    def validate_api_key(self):
        """APIキーの検証"""
        if not self.api_key or self.api_key.strip() == "":
            self.get_logger().error("APIキーが設定されていません。config/detector_params_server.yamlでapi_keyを設定してください。")
            return False
        
        # APIキーの形式をチェック（RoboflowのAPIキーは通常24文字）
        if len(self.api_key) < 10:
            self.get_logger().warn(f"APIキーの長さが短すぎます: {len(self.api_key)}文字")
        
        self.get_logger().info(f"APIキー設定済み: {self.api_key[:8]}...")
        return True
    
    def check_inference_server(self):
        """Inference Serverの接続確認"""
        try:
            # ヘルスチェック
            response = requests.get(f"{self.inference_server_url}/health", timeout=5.0)
            if response.status_code == 200:
                self.get_logger().info("Inference Server接続OK")
                return True
            else:
                self.get_logger().error(f"Inference Serverの応答が異常です: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Inference Serverに接続できません: {e}")
            self.get_logger().error("inference server start コマンドでサーバーを起動してください")
            return False
        
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
    
    def encode_image_to_base64(self, img):
        """画像をBase64エンコード"""
        try:
            _, buffer = cv2.imencode('.jpg', img)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            return img_base64
        except Exception as e:
            self.get_logger().error(f"画像エンコードエラー: {e}")
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
        
        # 予測結果の形式を確認してソート
        def get_confidence(pred):
            if isinstance(pred, dict):
                return pred.get('confidence', 0) or pred.get('score', 0)
            return 0
        
        sorted_predictions = sorted(predictions, key=get_confidence, reverse=True)
        filtered_predictions = []
        
        for i, pred1 in enumerate(sorted_predictions):
            if not isinstance(pred1, dict):
                continue
                
            should_keep = True
            
            # バウンディングボックスの座標を取得
            x1 = pred1.get('x', 0) or pred1.get('x_center', 0)
            y1 = pred1.get('y', 0) or pred1.get('y_center', 0)
            w1 = pred1.get('width', 0) or pred1.get('w', 0)
            h1 = pred1.get('height', 0) or pred1.get('h', 0)
            
            for pred2 in filtered_predictions:
                if not isinstance(pred2, dict):
                    continue
                    
                x2 = pred2.get('x', 0) or pred2.get('x_center', 0)
                y2 = pred2.get('y', 0) or pred2.get('y_center', 0)
                w2 = pred2.get('width', 0) or pred2.get('w', 0)
                h2 = pred2.get('height', 0) or pred2.get('h', 0)
                
                box1 = (x1 - w1/2, y1 - h1/2, w1, h1)
                box2 = (x2 - w2/2, y2 - h2/2, w2, h2)
                
                iou = self.calculate_iou(box1, box2)
                
                if iou >= self.overlap_threshold:
                    should_keep = False
                    break
            
            if should_keep:
                filtered_predictions.append(pred1)
        
        return filtered_predictions
    
    def split_processed_image(self, processed_image):
        """処理済みの複数DOI画像を個別のDOI画像に分割"""
        if processed_image is None:
            return []
        
        h, w = processed_image.shape[:2]
        
        # 正方形画像のサイズを計算（すべてのDOI画像は同じサイズ）
        doi_size = h  # 高さ=幅（正方形）
        
        # DOI領域数に基づいて分割
        doi_images = []
        for i in range(self.doi_count):
            x_start = i * doi_size
            x_end = (i + 1) * doi_size
            
            if x_end <= w:
                doi_image = processed_image[:, x_start:x_end]
                doi_images.append(doi_image)
            else:
                self.get_logger().warn(f"DOI領域 {i+1} の分割に失敗: 画像幅が不足")
                break
        
        return doi_images
    
    def infer_with_server(self, doi_image, region_index):
        """Inference Serverを使用してDOI画像の7セグメント表示を読み取る"""
        if doi_image is None or doi_image.size == 0:
            self.logger.warning(f"領域 {region_index}: DOI画像が無効です")
            return "ERROR"
        
        try:
            # 画像をBase64エンコード
            img_base64 = self.encode_image_to_base64(doi_image)
            if img_base64 is None:
                return "ERROR"
            
            # Inference Serverにリクエストを送信（V2 Route）
            payload = {
                "model_id": f"{self.model_name}/{self.model_version}",
                "image": [
                    {
                        "type": "base64",
                        "value": img_base64
                    }
                ],
                "confidence": self.confidence_threshold,
                "iou_threshold": self.iou_threshold
            }
            
            if self.api_key:
                payload["api_key"] = self.api_key
            
            # V2 Routeを使用（object_detectionタスク）
            endpoint = f"{self.inference_server_url}/infer/object_detection"
            headers = {"Content-Type": "application/json"}
            
            response = requests.post(
                endpoint,
                json=payload,
                headers=headers,
                timeout=10.0
            )
            
            # V2 Routeが失敗した場合はV1 Routeを試行
            if response.status_code != 200:
                
                # V1 Route用のペイロード
                v1_payload = {
                    "api_key": self.api_key,
                    "confidence": self.confidence_threshold,
                    "overlap": self.iou_threshold
                }
                
                # V1 Route
                v1_endpoint = f"{self.inference_server_url}/{self.model_name}/{self.model_version}"
                
                response = requests.post(
                    v1_endpoint,
                    json=img_base64,
                    headers={"Content-Type": "application/json"},
                    params=v1_payload,
                    timeout=10.0
                )
            
            if response.status_code != 200:
                self.logger.error(f"領域 {region_index}: Inference Serverエラー: {response.status_code}")
                self.logger.error(f"レスポンス内容: {response.text}")
                
                # APIキーエラーの場合の特別な処理
                if response.status_code == 401:
                    self.logger.error("APIキーが無効です。RoboflowのAPIキーを確認してください。")
                    self.logger.error("https://docs.roboflow.com/api-reference/authentication#retrieve-an-api-key")
                
                return "ERROR"
            
            result = response.json()
            
            # 予測結果を処理（レスポンス形式に応じて処理）
            predictions = []
            if isinstance(result, dict):
                # V2 Routeの場合
                predictions = result.get('predictions', [])
            elif isinstance(result, list):
                # リストの場合、最初の要素からpredictionsを取得
                if len(result) > 0 and isinstance(result[0], dict):
                    predictions = result[0].get('predictions', [])
                else:
                    # 直接予測結果のリストの場合
                    predictions = result
            else:
                self.logger.error(f"予期しないレスポンス形式: {type(result)}")
                return "ERROR"
            
            # 重複するバウンディングボックスをフィルタリング
            filtered_predictions = self.filter_overlapping_predictions(predictions)
            
            # 検出された数字を取得（小数点は除外）
            detected_digits = []
            for prediction in filtered_predictions:
                # 予測結果の形式を確認
                if isinstance(prediction, dict):
                    class_name = prediction.get('class', '') or prediction.get('class_name', '')
                else:
                    self.logger.warn(f"予期しない予測結果形式: {type(prediction)}")
                    continue
                
                if class_name.lower() in ['dot', 'decimal', 'point', '.']:
                    continue
                detected_digits.append(class_name)
            
            # 複数の数字が検出された場合は、x座標でソート
            if len(detected_digits) > 1:
                predictions_with_x = []
                for prediction in filtered_predictions:
                    if isinstance(prediction, dict):
                        class_name = prediction.get('class', '') or prediction.get('class_name', '')
                        if class_name.lower() in ['dot', 'decimal', 'point', '.']:
                            continue
                        x_coord = prediction.get('x', 0) or prediction.get('x_center', 0)
                        predictions_with_x.append((x_coord, class_name))
                predictions_with_x.sort(key=lambda x: x[0])
                detected_digits = [digit for _, digit in predictions_with_x]
            
            result_str = ''.join(detected_digits) if detected_digits else "NO_DETECTION"
            
            # デバッグ表示用の情報を返す
            debug_info = {
                'result': result_str,
                'predictions': filtered_predictions,
                'detected_digits': detected_digits
            }
            
            return debug_info
            
        except Exception as e:
            self.logger.error(f"領域 {region_index}: 推論エラー: {e}")
            return {'result': 'ERROR', 'predictions': [], 'detected_digits': []}
    
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
    
    def draw_bounding_boxes(self, image, predictions, region_index):
        """バウンディングボックスと検出結果を描画"""
        if not self.show_bounding_boxes or not predictions:
            return image
        
        # 色の設定（BGR形式）
        colors = [
            (0, 255, 0),    # 緑
            (255, 0, 0),    # 青
            (0, 0, 255),    # 赤
            (255, 255, 0),  # シアン
            (255, 0, 255),  # マゼンタ
            (0, 255, 255),  # イエロー
        ]
        
        for i, prediction in enumerate(predictions):
            if not isinstance(prediction, dict):
                continue
            
            # バウンディングボックスの座標を取得
            x = int(prediction.get('x', 0) or prediction.get('x_center', 0))
            y = int(prediction.get('y', 0) or prediction.get('y_center', 0))
            w = int(prediction.get('width', 0) or prediction.get('w', 0))
            h = int(prediction.get('height', 0) or prediction.get('h', 0))
            confidence = prediction.get('confidence', 0) or prediction.get('score', 0)
            class_name = prediction.get('class', '') or prediction.get('class_name', '')
            
            # バウンディングボックスを描画
            color = colors[i % len(colors)]
            cv2.rectangle(image, (x - w//2, y - h//2), (x + w//2, y + h//2), color, 2)
            
            # ラベルを描画
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.rectangle(image, (x - w//2, y - h//2 - label_size[1] - 5), 
                         (x - w//2 + label_size[0], y - h//2), color, -1)
            cv2.putText(image, label, (x - w//2, y - h//2 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image
    
    def process_frame(self):
        """フレーム処理のメイン関数"""
        # フレーム取得
        data = self.get_frame_with_timestamp()
        if not data:
            return
        
        # 画像デコード
        processed_frame = self.decode_image_from_base64(data["image"])
        if processed_frame is None:
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
        
        # DOI領域数が設定されていない場合はスキップ
        if self.doi_count <= 0:
            self.get_logger().warn("DOI領域数が設定されていません。パラメータで設定してください。")
            return
        
        # 処理済み画像を個別のDOI画像に分割
        doi_images = self.split_processed_image(processed_frame)
        
        if len(doi_images) != self.doi_count:
            self.get_logger().warn(f"DOI画像の分割に失敗: 期待値={self.doi_count}, 実際={len(doi_images)}")
            return
        
        # 各DOI画像の7セグメント表示を読み取り
        detection_results = []
        numeric_values = []
        debug_images = []
        
        for i, doi_image in enumerate(doi_images):
            # 7セグメント読み取り（Inference Server使用）
            debug_info = self.infer_with_server(doi_image, i+1)
            
            if isinstance(debug_info, dict):
                digit = debug_info['result']
                predictions = debug_info['predictions']
                
                # バウンディングボックスを描画
                if self.show_bounding_boxes:
                    debug_image = doi_image.copy()
                    debug_image = self.draw_bounding_boxes(debug_image, predictions, i+1)
                    
                    # 最終的な検出数値を画像に表示
                    formatted_digit = self.format_seven_segment_number(digit)
                    cv2.putText(debug_image, f"Region {i+1}: {formatted_digit}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    debug_images.append(debug_image)
                else:
                    debug_images.append(doi_image)
            else:
                digit = debug_info
                debug_images.append(doi_image)
            
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
        
        # デバッグウィンドウを表示
        if self.show_debug_window and debug_images:
            # 個別のDOI画像を横に並べて表示
            combined_image = np.hstack(debug_images)
            cv2.imshow('7-Segment Detection Results', combined_image)
            cv2.waitKey(1)
        
        # ROSメッセージのパブリッシュ
        # 検出結果メッセージ
        detection_msg = String()
        detection_msg.data = json.dumps({
            'detections': detection_results,
            'frame_count': self.frame_count,
            'server_timestamp': data['timestamp'],
            'client_timestamp': current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'delay_ms': delay_ms,
            'doi_count': self.doi_count,
            'inference_method': 'server'
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
        
        # ログ出力（50フレームごと）
        if self.frame_count % 50 == 0:
            self.get_logger().info(f"フレーム {self.frame_count}: 検出結果 = {detection_results}")


def main(args=None):
    rclpy.init(args=args)
    
    node = SevenSegmentReaderServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ノードが終了されました")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

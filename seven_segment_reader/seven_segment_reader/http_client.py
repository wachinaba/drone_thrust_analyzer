#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import cv2
import requests
import base64
import json
from datetime import datetime
import logging
import math
import numpy as np
from collections import deque


class SevenSegmentReaderServerNode(Node):
    """7セグメントディスプレイ読み取りROS2ノード（HTTP取得版）"""
    
    def __init__(self):
        super().__init__('seven_segment_reader_server_node')
        
        # パラメータの宣言（YAMLファイルから読み込み）
        self.declare_parameter('doi_server_url', 'http://127.0.0.1:5001')
        self.declare_parameter('processing_interval', 0.1)  # 秒
        self.declare_parameter('log_level', 'DEBUG')
        self.declare_parameter('show_debug_window', True)  # デバッグウィンドウ表示（画像取得）
        self.declare_parameter('median_window', 7)  # 中央値フィルタ窓サイズ
        
        # パラメータの取得
        self.doi_server_url = self.get_parameter('doi_server_url').value
        self.processing_interval = float(self.get_parameter('processing_interval').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.log_level_param = str(self.get_parameter('log_level').value).upper()
        self.median_window = int(self.get_parameter('median_window').value)
        
        # サーバー接続確認
        self.check_doi_server()
        
        # パブリッシャーの作成
        self.detection_pub = self.create_publisher(String, 'seven_segment_detection', 10)
        self.numeric_value_pub = self.create_publisher(Float64MultiArray, 'seven_segment_values', 10)
        self.timestamp_pub = self.create_publisher(String, 'detection_timestamp', 10)
        
        # タイマーの作成
        self.timer = self.create_timer(self.processing_interval, self.process_frame)
        
        # 状態変数
        self.frame_count = 0
        self.last_detection_time = None
        self.median_buffers = []  # 各DOIごとの履歴バッファ（deque of float）
        
        # ログ設定
        self.setup_logging()
        
        self.get_logger().info("7セグメントディスプレイ読み取りノード（HTTP取得版）が開始されました")
        self.get_logger().info(f"DOIサーバーURL: {self.doi_server_url}")
        self.get_logger().info(f"処理間隔: {self.processing_interval}秒")
        
    def check_doi_server(self):
        """DOIサーバーのヘルスチェック"""
        try:
            response = requests.get(f"{self.doi_server_url}/health", timeout=5.0)
            if response.status_code == 200:
                info = response.json() if response.headers.get('Content-Type','').startswith('application/json') else {}
                self.get_logger().info(f"DOIサーバー接続OK（DOI数: {info.get('doi_regions', 'unknown')}）")
            else:
                self.get_logger().warn(f"DOIサーバー応答: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"DOIサーバーに接続できません: {e}")
        
    def setup_logging(self):
        """ログ設定の初期化（コンソール出力、パラメータでレベル制御）"""
        lvl_str = getattr(self, 'log_level_param', 'INFO')
        level = {
            'DEBUG': logging.DEBUG,
            'INFO': logging.INFO,
            'WARNING': logging.WARNING,
            'ERROR': logging.ERROR,
            'CRITICAL': logging.CRITICAL,
        }.get(str(lvl_str).upper(), logging.INFO)
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(level)
        self.logger.info(f"7セグメント表示読み取りを開始します（log_level={lvl_str}）")
        self.logger.info(f"median_window={getattr(self, 'median_window', 7)}")
        
    def decode_image_from_base64(self, img_base64):
        """Base64エンコードされた画像をデコード"""
        try:
            img_data = base64.b64decode(img_base64)
            # OpenCVはnp.asarrayを内部で使うため、np経由でなくてもよいが互換のためimdecodeを使用
            import numpy as _np
            nparr = _np.frombuffer(img_data, _np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            self.get_logger().error(f"画像デコードエラー: {e}")
            return None
    
    def normalize_detected_string(self, raw_string: str) -> str:
        """検出文字列の正規化（ハイフン除去・エラー判定・小数点付与）。"""
        if raw_string is None:
            return "NO_DETECTION"
        # ハイフンや空白など非数字を排除（小数点は学習で出ないため自前で付与）
        digits_only = ''.join([c for c in str(raw_string) if c.isdigit()])
        if len(digits_only) == 0:
            return "NO_DETECTION"
        if len(digits_only) not in (3, 4):
            return f"INVALID_LENGTH_{len(digits_only)}"
        if len(digits_only) == 3:
            return f"{digits_only[0]}.{digits_only[1]}{digits_only[2]}"
        return f"{digits_only[0]}{digits_only[1]}.{digits_only[2]}{digits_only[3]}"

    def _bbox_to_xyxy(self, item):
        # サポートする形式:
        # - item['bbox'] が [x, y, w, h] or [x1, y1, x2, y2] or dict
        # - item 直下に x,y,w,h もしくは x1,y1,x2,y2
        def as_xyxy_from_xywh(x, y, w, h):
            return [float(x), float(y), float(x) + float(w), float(y) + float(h)]
        def as_xyxy_from_xyxy(x1, y1, x2, y2):
            return [float(x1), float(y1), float(x2), float(y2)]

        b = item.get('bbox')
        if isinstance(b, dict):
            if all(k in b for k in ('x1', 'y1', 'x2', 'y2')):
                return as_xyxy_from_xyxy(b['x1'], b['y1'], b['x2'], b['y2'])
            if all(k in b for k in ('x', 'y', 'w', 'h')):
                return as_xyxy_from_xywh(b['x'], b['y'], b['w'], b['h'])
        elif isinstance(b, (list, tuple)) and len(b) == 4:
            x1, y1, x2_or_w, y2_or_h = b
            if x2_or_w > 0 and y2_or_h > 0:
                # 一般的な [x, y, w, h]
                return as_xyxy_from_xywh(x1, y1, x2_or_w, y2_or_h)
            return as_xyxy_from_xyxy(x1, y1, x2_or_w, y2_or_h)

        # トップレベルにあるパターン
        if all(k in item for k in ('x1', 'y1', 'x2', 'y2')):
            return as_xyxy_from_xyxy(item['x1'], item['y1'], item['x2'], item['y2'])
        if all(k in item for k in ('x', 'y', 'w', 'h')):
            return as_xyxy_from_xywh(item['x'], item['y'], item['w'], item['h'])
        return None

    def _compute_iou(self, a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        if ax2 < ax1:
            ax1, ax2 = ax2, ax1
        if ay2 < ay1:
            ay1, ay2 = ay2, ay1
        if bx2 < bx1:
            bx1, bx2 = bx2, bx1
        if by2 < by1:
            by1, by2 = by2, by1

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        iw = max(0.0, inter_x2 - inter_x1)
        ih = max(0.0, inter_y2 - inter_y1)
        inter = iw * ih
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter
        if union <= 0.0:
            return 0.0
        return inter / union

    def _nms_by_iou(self, detections, iou_thresh=0.5):
        # detections: [{string, score, bbox(=xyxy or None)}]
        sorted_dets = sorted(detections, key=lambda d: float(d.get('score', 1.0)), reverse=True)
        kept = []
        for d in sorted_dets:
            db = d.get('bbox')
            if db is None:
                kept.append(d)
                continue
            overlapped = False
            for k in kept:
                kb = k.get('bbox')
                if kb is None:
                    continue
                if self._compute_iou(db, kb) > iou_thresh:
                    overlapped = True
                    break
            if not overlapped:
                kept.append(d)
        return kept

    def _extract_detections(self, data):
        # regions があれば bbox/score を伴う候補を抽出。無ければ空配列を返しフォールバックへ。
        out = []
        regions = data.get('regions')
        if isinstance(regions, list):
            for r in regions:
                s = r.get('detected_string') or r.get('string') or r.get('text') or ''
                score = r.get('probability', r.get('score', r.get('confidence', 1.0)))
                bbox = self._bbox_to_xyxy(r)
                out.append({'string': s, 'score': float(score) if score is not None else 1.0, 'bbox': bbox})
        return out

    def _ensure_median_buffers(self, size):
        # 必要数のバッファを確保
        while len(self.median_buffers) < size:
            self.median_buffers.append(deque(maxlen=self.median_window))

    def _apply_median_filter(self, numeric_values):
        # 各DOIごとに履歴からNaNを無視した中央値を計算
        self._ensure_median_buffers(len(numeric_values))
        filtered = []
        for i, val in enumerate(numeric_values):
            self.median_buffers[i].append(val)
            arr = np.array(self.median_buffers[i], dtype=float)
            if arr.size == 0:
                filtered.append(float('nan'))
                continue
            try:
                med = float(np.nanmedian(arr))
            except Exception:
                med = float('nan')
            if np.isnan(med):
                filtered.append(float('nan'))
            else:
                filtered.append(med)
        return filtered
    def fetch_inference(self):
        """DOIサーバーから推論結果を取得。デバッグ時は画像付きエンドポイント。"""
        endpoint = "/inference_result" if self.show_debug_window else "/inference_only"
        try:
            self.logger.debug(f"HTTP GET {self.doi_server_url}{endpoint} を要求")
            response = requests.get(f"{self.doi_server_url}{endpoint}", timeout=5.0)
            self.logger.debug(f"HTTP 応答 {response.status_code}, Content-Type={response.headers.get('Content-Type')}")
            if response.status_code != 200:
                self.get_logger().warn(f"推論取得エラー: HTTP {response.status_code}")
                return None
            data = response.json()
            # 軽量な要約情報のみDEBUG出力
            keys = list(data.keys()) if isinstance(data, dict) else []
            self.logger.debug(f"推論JSONキー: {keys}")
            return data
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"推論取得例外: {e}")
            return None
    
    def process_frame(self):
        """HTTPで推論結果を取得し、正規化してトピック配信"""
        self.logger.debug("process_frame tick 開始")
        data = self.fetch_inference()
        if not data:
            self.logger.debug("推論データなし（fetch_inference が None/空）")
            return
        
        self.frame_count += 1
        current_time = datetime.now()
        # サーバータイムスタンプ
        server_ts = data.get('timestamp', '')
        server_time = current_time
        if isinstance(server_ts, str):
            for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
                try:
                    server_time = datetime.strptime(server_ts, fmt)
                    break
                except ValueError:
                    continue
        delay_ms = (current_time - server_time).total_seconds() * 1000
        self.logger.debug(f"server_ts={server_ts}, delay_ms={delay_ms:.2f}")
        
        # 検出候補の抽出 + NMS で重複排除（IoU > 0.5 を同一とみなす）
        detections = self._extract_detections(data)
        if detections:
            self.logger.debug(f"抽出検出候補: {len(detections)} 件")
            for i, d in enumerate(detections):
                self.logger.debug(f"  cand[{i}]: string='{d.get('string','')}', score={d.get('score')}, bbox={d.get('bbox')}")
        if detections and any(d.get('bbox') is not None for d in detections):
            deduped = self._nms_by_iou(detections, iou_thresh=0.5)
            self.logger.debug(f"NMS後: {len(deduped)} 件を採用")
            detected_strings = [d.get('string', '') for d in deduped]
        else:
            # フォールバック（従来のレスポンス形式）
            self.logger.debug("フォールバック経路で detected_strings を抽出")
            detected_strings = []
            if isinstance(data.get('detected_strings'), list) and data['detected_strings']:
                detected_strings = [item.get('string', '') for item in data['detected_strings']]
            elif isinstance(data.get('regions'), list):
                detected_strings = [r.get('detected_string', '') for r in data['regions']]

        # 正規化
        detection_results = [self.normalize_detected_string(s) for s in detected_strings]
        if detected_strings:
            for raw, norm in zip(detected_strings, detection_results):
                self.logger.debug(f"normalize: '{raw}' -> '{norm}'")

        # 数値配列
        numeric_values = []
        for s in detection_results:
            try:
                if s.startswith('INVALID_LENGTH') or s in ("NO_DETECTION", "ERROR"):
                    numeric_values.append(float('nan'))
                    self.logger.debug(f"toFloat: '{s}' -> NaN")
                else:
                    numeric_values.append(float(s))
                    self.logger.debug(f"toFloat: '{s}' -> {numeric_values[-1]}")
            except Exception:
                numeric_values.append(float('nan'))
                self.logger.debug(f"toFloat: '{s}' 変換例外 -> NaN")

        # 中央値フィルタ適用（NaNを無視）
        if len(numeric_values) > 0:
            prev = list(numeric_values)
            numeric_values = self._apply_median_filter(numeric_values)
            self.logger.debug(f"median(window={self.median_window}): before={prev} after={numeric_values}")
        
        # デバッグウィンドウ（画像がなくてもフォールバック表示）
        if self.show_debug_window:
            img = None
            if 'image' in data:
                img = self.decode_image_from_base64(data['image'])
            
            # DOI数と画像サイズからDOIサイズを推定
            doi_count = len(numeric_values) if numeric_values else int(data.get('doi_count', 0) or 0)
            img_width = img.shape[1] if img is not None else 480
            img_height = img.shape[0] if img is not None else 200
            
            # DOIサイズ推定（横並びと仮定）
            doi_width = img_width // max(1, doi_count) if doi_count > 0 else img_width
            doi_height = img_height
            
            self.logger.debug(f"DOI推定: count={doi_count}, img_size=({img_width}x{img_height}), doi_size=({doi_width}x{doi_height})")
            
            # 表示テキストを準備（複数行対応）
            display_texts = []
            if numeric_values:
                for i, value in enumerate(numeric_values):
                    raw_val = prev[i] if i < len(prev) else float('nan')
                    raw_text = "n/a" if (raw_val is None or math.isnan(raw_val)) else f"{raw_val:.2f}"
                    med_text = "n/a" if (value is None or math.isnan(value)) else f"{value:.2f}"
                    display_texts.append([
                        f"DOI {i+1}:",
                        f"raw={raw_text}",
                        f"med={med_text} m/s"
                    ])
            else:
                for i in range(doi_count):
                    display_texts.append([f"DOI {i+1}:", "n/a", ""])

            # 画像が無ければ黒背景のキャンバスを生成
            if img is None:
                height = 20 + 3 * 20 + 10  # 3行分の高さ
                width = max(480, doi_count * doi_width)  # DOI数×推定DOI幅
                img = np.zeros((height, width, 3), dtype=np.uint8)
            
            # オーバーレイ描画（DOIサイズに基づく配置）
            try:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                text_thickness = 1
                outline_thickness = 2
                line_height = 20
                
                for doi_idx, doi_texts in enumerate(display_texts):
                    # DOI位置計算（横並び）
                    x_offset = doi_idx * doi_width + 5  # DOI左端 + マージン
                    y_pos = 30  # 上端マージン
                    
                    # 画像境界チェック
                    if x_offset + doi_width > img.shape[1]:
                        x_offset = max(5, img.shape[1] - doi_width - 5)
                    
                    for text_line in doi_texts:
                        if text_line:  # 空文字列はスキップ
                            (text_size, baseline) = cv2.getTextSize(text_line, font, font_scale, text_thickness)
                            org = (x_offset, y_pos)
                            # 白の太枠 → 黒文字の順で描画
                            cv2.putText(img, text_line, org, font, font_scale, (255, 255, 255), outline_thickness, cv2.LINE_AA)
                            cv2.putText(img, text_line, org, font, font_scale, (0, 0, 0), text_thickness, cv2.LINE_AA)
                        y_pos += line_height
            except Exception:
                pass

            cv2.imshow('7-Segment Detection Results', img)
            cv2.waitKey(1)
        
        # ROSメッセージのパブリッシュ
        detection_msg = String()
        detection_msg.data = json.dumps({
            'detections': detection_results,
            'frame_count': self.frame_count,
            'server_timestamp': server_ts,
            'client_timestamp': current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'delay_ms': delay_ms,
            'doi_count': data.get('doi_count', len(detected_strings) or 0),
            'inference_method': 'http_server'
        })
        self.logger.debug(f"publish detection_msg bytes={len(detection_msg.data)} 数値数={len(numeric_values)}")
        self.detection_pub.publish(detection_msg)
        
        numeric_msg = Float64MultiArray()
        numeric_msg.data = numeric_values
        self.numeric_value_pub.publish(numeric_msg)
        
        timestamp_msg = String()
        timestamp_msg.data = server_ts if isinstance(server_ts, str) else current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.timestamp_pub.publish(timestamp_msg)
        
        self.last_detection_time = current_time
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

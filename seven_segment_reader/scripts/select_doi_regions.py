#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import argparse
import requests
import base64
from datetime import datetime
import os


class DOIRegionSelector:
    """DOI領域選択クラス"""
    
    def __init__(self, server_url="http://127.0.0.1:5000"):
        self.server_url = server_url
        self.doi_regions = []  # DOI領域を保存
        self.click_count = 0
        self.current_region = []
        self.frame = None
        
    def get_frame_from_server(self):
        """サーバーからフレームを取得"""
        try:
            response = requests.get(f"{self.server_url}/frame_with_timestamp", timeout=5.0)
            response.raise_for_status()
            data = response.json()
            
            # Base64データをデコード
            img_data = base64.b64decode(data["image"])
            nparr = np.frombuffer(img_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            print(f"フレーム取得エラー: {e}")
            return None
    
    def mouse_callback(self, event, x, y, flags, param):
        """マウスクリックでDOI領域を指定するコールバック関数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.click_count == 0:
                # 最初のクリック：領域の開始点
                self.current_region = [(x, y)]
                self.click_count = 1
                print(f"領域 {len(self.doi_regions) + 1} の開始点: ({x}, {y})")
            elif self.click_count == 1:
                # 2番目のクリック：領域の終了点
                self.current_region.append((x, y))
                self.doi_regions.append(self.current_region.copy())
                self.click_count = 0
                print(f"領域 {len(self.doi_regions)} の終了点: ({x}, {y})")
                print(f"領域 {len(self.doi_regions)} が設定されました")
    
    def draw_regions(self, frame):
        """選択された領域をフレームに描画"""
        display_frame = frame.copy()
        
        # 既に選択された領域を描画
        for i, region in enumerate(self.doi_regions):
            start_point = region[0]
            end_point = region[1]
            cv2.rectangle(display_frame, start_point, end_point, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Region {i+1}", 
                       (start_point[0], start_point[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 現在選択中の領域を描画
        if self.click_count == 1 and self.current_region:
            cv2.circle(display_frame, self.current_region[0], 5, (0, 0, 255), -1)
            cv2.putText(display_frame, "終了点をクリックしてください", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return display_frame
    
    def select_regions(self):
        """DOI領域を選択する"""
        print("DOI領域を選択してください。")
        print("各領域について、左上と右下の点をクリックしてください。")
        print("すべての領域を選択したら、'Enter'キーを押してください。")
        print("選択をやり直す場合は、'r'キーを押してください。")
        print("終了する場合は、'q'キーを押してください。")
        
        # フレーム取得
        self.frame = self.get_frame_from_server()
        if self.frame is None:
            print("フレームを取得できませんでした。サーバーが起動しているか確認してください。")
            return False
        
        # ウィンドウの作成とマウスコールバックの設定
        cv2.namedWindow('DOI Region Selection', cv2.WINDOW_NORMAL)
        
        # ウィンドウサイズを大きく設定
        height, width = self.frame.shape[:2]
        max_width = min(1920, width * 2)
        max_height = min(1080, height * 2)
        cv2.resizeWindow('DOI Region Selection', max_width, max_height)
        
        cv2.setMouseCallback('DOI Region Selection', self.mouse_callback)
        
        while True:
            # フレームを描画
            display_frame = self.draw_regions(self.frame)
            
            # 操作説明を表示
            cv2.putText(display_frame, "Controls: Click to select regions, 'r'=reset, 'Enter'=save, 'q'=quit", 
                       (10, display_frame.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('DOI Region Selection', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                return False
            elif key == ord('r'):
                # 選択をリセット
                self.doi_regions = []
                self.click_count = 0
                self.current_region = []
                print("選択をリセットしました。")
            elif key == 13:  # Enter key
                if self.doi_regions:
                    cv2.destroyAllWindows()
                    print(f"{len(self.doi_regions)}個の領域が選択されました。")
                    return True
                else:
                    print("少なくとも1つの領域を選択してください。")
        
        cv2.destroyAllWindows()
        return False
    
    def save_regions_to_config(self, config_file):
        """選択された領域を設定ファイルに保存"""
        if not self.doi_regions:
            print("保存する領域がありません。")
            return False
        
        # 設定ファイルを読み込み（存在する場合）
        config = {}
        if os.path.exists(config_file):
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f) or {}
        
        # DOI領域を文字列として保存（ROS2パラメータ形式）
        doi_regions_str_parts = []
        for region in self.doi_regions:
            start_point = region[0]
            end_point = region[1]
            # [左上x, 左上y, 右下x, 右下y]の形式で文字列に追加
            doi_regions_str_parts.extend([
                str(min(start_point[0], end_point[0])),  # 左上x
                str(min(start_point[1], end_point[1])),  # 左上y
                str(max(start_point[0], end_point[0])),  # 右下x
                str(max(start_point[1], end_point[1]))   # 右下y
            ])
        
        doi_regions_str = ','.join(doi_regions_str_parts)
        
        # ROS2パラメータ形式で保存
        ros_config = {
            'seven_segment_reader_node': {
                'ros__parameters': config
            }
        }
        ros_config['seven_segment_reader_node']['ros__parameters']['doi_regions'] = doi_regions_str
        
        # 設定ファイルに保存
        with open(config_file, 'w', encoding='utf-8') as f:
            yaml.dump(ros_config, f, default_flow_style=False, allow_unicode=True)
        
        print(f"DOI領域を {config_file} に保存しました:")
        coords = [int(x) for x in doi_regions_str.split(',')]
        for i in range(0, len(coords), 4):
            if i + 3 < len(coords):
                region = coords[i:i+4]
                print(f"  領域 {i//4+1}: [{region[0]}, {region[1]}, {region[2]}, {region[3]}]")
        
        return True
    
    def load_regions_from_config(self, config_file):
        """設定ファイルからDOI領域を読み込み"""
        if not os.path.exists(config_file):
            print(f"設定ファイル {config_file} が見つかりません。")
            return False
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            if 'doi_regions' in config:
                self.doi_regions = []
                for region in config['doi_regions']:
                    # [左上x, 左上y, 右下x, 右下y]から[(x1,y1), (x2,y2)]に変換
                    self.doi_regions.append([
                        (region[0], region[1]),  # 左上
                        (region[2], region[3])   # 右下
                    ])
                
                print(f"設定ファイルから {len(self.doi_regions)} 個の領域を読み込みました:")
                for i, region in enumerate(self.doi_regions):
                    print(f"  領域 {i+1}: {region[0]} -> {region[1]}")
                return True
            else:
                print("設定ファイルにDOI領域が設定されていません。")
                return False
                
        except Exception as e:
            print(f"設定ファイルの読み込みエラー: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='DOI領域選択ツール')
    parser.add_argument('--server-url', default='http://127.0.0.1:5000',
                       help='IPカメラサーバーのURL (デフォルト: http://127.0.0.1:5000)')
    parser.add_argument('--config-file', default='config/detector_params.yaml',
                       help='設定ファイルのパス (デフォルト: config/detector_params.yaml)')
    parser.add_argument('--load-existing', action='store_true',
                       help='既存の設定ファイルから領域を読み込む')
    
    args = parser.parse_args()
    
    # DOI領域選択クラスの初期化
    selector = DOIRegionSelector(args.server_url)
    
    # 既存の設定を読み込む場合
    if args.load_existing:
        if not selector.load_regions_from_config(args.config_file):
            print("既存の設定を読み込めませんでした。新しく領域を選択してください。")
    
    # DOI領域を選択
    if selector.select_regions():
        # 設定ファイルに保存
        selector.save_regions_to_config(args.config_file)
        print("DOI領域の設定が完了しました。")
    else:
        print("DOI領域の設定がキャンセルされました。")


if __name__ == "__main__":
    main()

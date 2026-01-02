#!/usr/bin/env python3

"""
ARマーカー一括生成スクリプト

設定ファイルからベース側とスライダ側のマーカーを一括生成し、
印刷用の最適化された画像と設定ファイルを出力する。
"""

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import sys
import yaml
import json
from pathlib import Path

# 他のスクリプトの関数をインポート
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from generate_markers import generate_marker
from generate_board import generate_base_board, generate_slider_board


def load_config(config_path: str) -> dict:
    """
    設定ファイルを読み込む
    
    Args:
        config_path: 設定ファイルのパス
    
    Returns:
        設定辞書
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def mm_to_pixels(mm: float, dpi: int = 300) -> int:
    """
    ミリメートルをピクセルに変換
    
    Args:
        mm: ミリメートル
        dpi: 解像度
    
    Returns:
        ピクセル数
    """
    return int(mm * dpi / 25.4)


def generate_config_file(marker_ids: list, marker_size_mm: float, 
                        spacing_mm: float, layout: str, output_path: str) -> None:
    """
    設定ファイルを生成する
    
    Args:
        marker_ids: マーカーIDのリスト
        marker_size_mm: マーカーサイズ（mm）
        spacing_mm: マーカー間隔（mm）
        layout: レイアウト
        output_path: 出力パス
    """
    config = {
        "marker_size_mm": marker_size_mm,
        "markers": []
    }
    
    if layout == 'linear':
        # 直線配置
        for i, marker_id in enumerate(marker_ids):
            x = i * spacing_mm
            config["markers"].append({
                "id": marker_id,
                "translation": [x, 0.0, 0.0]
            })
    elif layout == 'grid':
        # グリッド配置
        cols = int(np.ceil(np.sqrt(len(marker_ids))))
        for i, marker_id in enumerate(marker_ids):
            row = i // cols
            col = i % cols
            x = col * spacing_mm
            y = row * spacing_mm
            config["markers"].append({
                "id": marker_id,
                "translation": [x, y, 0.0]
            })
    
    # JSONファイルとして保存
    with open(output_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"設定ファイルを {output_path} に保存しました")


def generate_all_markers(config_path: str, output_dir: str = "output") -> None:
    """
    設定ファイルから全てのマーカーを生成する
    
    Args:
        config_path: 設定ファイルのパス
        output_dir: 出力ディレクトリ
    """
    # 設定ファイルを読み込み
    config = load_config(config_path)
    
    # 出力ディレクトリを作成
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, "markers", "base"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "markers", "slider"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "boards"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "configs"), exist_ok=True)
    
    # 基本設定
    dictionary = config.get('dictionary', 'DICT_4X4_100')
    marker_size_mm = config.get('marker_size_mm', 50.0)
    dpi = config.get('dpi', 300)
    marker_size_pixels = mm_to_pixels(marker_size_mm, dpi)
    margin = config.get('margin', 0)  # マージン設定（デフォルト0で余白なし）
    
    # ベース側マーカーの生成
    base_config = config.get('base_markers', {})
    base_start_id = base_config.get('start_id', 0)
    base_end_id = base_config.get('end_id', 4)
    base_spacing_mm = base_config.get('spacing_mm', 200.0)
    base_layout = base_config.get('layout', 'linear')
    base_spacing_pixels = mm_to_pixels(base_spacing_mm, dpi)
    
    # スライダ側マーカーの生成
    slider_config = config.get('slider_markers', {})
    slider_start_id = slider_config.get('start_id', 50)
    slider_end_id = slider_config.get('end_id', 53)
    slider_spacing_mm = slider_config.get('spacing_mm', 100.0)
    slider_layout = slider_config.get('layout', 'grid')
    slider_spacing_pixels = mm_to_pixels(slider_spacing_mm, dpi)
    
    output_config = config.get('output', {})
    
    # 個別マーカーの生成
    if output_config.get('individual_markers', True):
        print("個別マーカーを生成中...")
        
        # ベース側マーカー
        for marker_id in range(base_start_id, base_end_id + 1):
            output_path = os.path.join(output_dir, "markers", "base", f"marker_{marker_id}.png")
            generate_marker(marker_id, dictionary, marker_size_pixels, output_path, margin)
        
        # スライダ側マーカー
        for marker_id in range(slider_start_id, slider_end_id + 1):
            output_path = os.path.join(output_dir, "markers", "slider", f"marker_{marker_id}.png")
            generate_marker(marker_id, dictionary, marker_size_pixels, output_path, margin)
    
    # マーカーボードの生成
    if output_config.get('board_layout', True):
        print("マーカーボードを生成中...")
        
        # ベース側ボード
        base_board_path = os.path.join(output_dir, "boards", "base_board.png")
        generate_base_board(base_start_id, base_end_id, dictionary,
                          marker_size_pixels, base_spacing_pixels, base_board_path)
        
        # スライダ側ボード
        slider_board_path = os.path.join(output_dir, "boards", "slider_board.png")
        generate_slider_board(slider_start_id, slider_end_id, dictionary,
                            marker_size_pixels, slider_spacing_pixels, slider_board_path)
    
    # 設定ファイルの生成
    if output_config.get('config_files', True):
        print("設定ファイルを生成中...")
        
        # ベース側設定ファイル
        base_config_path = os.path.join(output_dir, "configs", "base_board_config.json")
        base_marker_ids = list(range(base_start_id, base_end_id + 1))
        generate_config_file(base_marker_ids, marker_size_mm, base_spacing_mm, 
                           base_layout, base_config_path)
        
        # スライダ側設定ファイル
        slider_config_path = os.path.join(output_dir, "configs", "slider_board_config.json")
        slider_marker_ids = list(range(slider_start_id, slider_end_id + 1))
        generate_config_file(slider_marker_ids, marker_size_mm, slider_spacing_mm, 
                           slider_layout, slider_config_path)
    
    print(f"全てのファイルを {output_dir} に生成しました")


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='ArUcoマーカー一括生成スクリプト')
    parser.add_argument('--config', type=str, required=True,
                       help='設定ファイルのパス')
    parser.add_argument('--output-dir', type=str, default='output',
                       help='出力ディレクトリ')
    
    args = parser.parse_args()
    
    # 設定ファイルの存在確認
    if not os.path.exists(args.config):
        print(f"エラー: 設定ファイル {args.config} が見つかりません")
        sys.exit(1)
    
    # 一括生成を実行
    generate_all_markers(args.config, args.output_dir)


if __name__ == '__main__':
    main() 
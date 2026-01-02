#!/usr/bin/env python3

"""
ARマーカーボード生成スクリプト

複数のArUcoマーカーを1枚の画像に配置し、印刷用に最適化されたマーカーボードを生成する。
"""

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import sys
from pathlib import Path


def generate_marker_board(marker_ids: list, dictionary_name: str = 'DICT_4X4_100',
                         marker_size_pixels: int = 200, spacing_pixels: int = 100,
                         layout: str = 'linear', output_path: str = None) -> np.ndarray:
    """
    マーカーボードを生成する
    
    Args:
        marker_ids: マーカーIDのリスト
        dictionary_name: 辞書名
        marker_size_pixels: マーカーサイズ（ピクセル）
        spacing_pixels: マーカー間隔（ピクセル）
        layout: レイアウト（'linear' または 'grid'）
        output_path: 出力パス
    
    Returns:
        生成されたマーカーボード画像
    """
    # 辞書の取得
    dictionary_name_attr = getattr(aruco, dictionary_name)
    dictionary = aruco.getPredefinedDictionary(dictionary_name_attr)
    
    # マーカーを生成
    markers = []
    for marker_id in marker_ids:
        marker_image = aruco.generateImageMarker(dictionary, marker_id, marker_size_pixels)
        markers.append(marker_image)
    
    # レイアウトに応じてボードサイズを計算
    if layout == 'linear':
        # 直線配置
        board_width = len(markers) * marker_size_pixels + (len(markers) - 1) * spacing_pixels
        board_height = marker_size_pixels
        cols = len(markers)
        rows = 1
    elif layout == 'grid':
        # グリッド配置
        cols = int(np.ceil(np.sqrt(len(markers))))
        rows = int(np.ceil(len(markers) / cols))
        board_width = cols * marker_size_pixels + (cols - 1) * spacing_pixels
        board_height = rows * marker_size_pixels + (rows - 1) * spacing_pixels
    else:
        raise ValueError(f"サポートされていないレイアウト: {layout}")
    
    # マージンを追加
    margin = 100
    final_width = board_width + 2 * margin
    final_height = board_height + 2 * margin
    
    # 白い背景の画像を作成
    board_image = np.ones((final_height, final_width), dtype=np.uint8) * 255
    
    # マーカーを配置
    for i, marker_image in enumerate(markers):
        if layout == 'linear':
            row = 0
            col = i
        else:  # grid
            row = i // cols
            col = i % cols
        
        # マーカーの位置を計算
        x = margin + col * (marker_size_pixels + spacing_pixels)
        y = margin + row * (marker_size_pixels + spacing_pixels)
        
        # マーカーを配置
        board_image[y:y+marker_size_pixels, x:x+marker_size_pixels] = marker_image
    
    # ファイルに保存
    if output_path:
        cv2.imwrite(output_path, board_image)
        print(f"マーカーボードを {output_path} に保存しました")
        print(f"配置されたマーカーID: {marker_ids}")
    
    return board_image


def generate_base_board(start_id: int, end_id: int, dictionary_name: str = 'DICT_4X4_100',
                       marker_size_pixels: int = 200, spacing_pixels: int = 100,
                       output_path: str = None) -> np.ndarray:
    """
    ベース側マーカーボードを生成する
    
    Args:
        start_id: 開始ID
        end_id: 終了ID
        dictionary_name: 辞書名
        marker_size_pixels: マーカーサイズ（ピクセル）
        spacing_pixels: マーカー間隔（ピクセル）
        output_path: 出力パス
    
    Returns:
        生成されたベース側マーカーボード画像
    """
    marker_ids = list(range(start_id, end_id + 1))
    return generate_marker_board(marker_ids, dictionary_name, marker_size_pixels,
                               spacing_pixels, 'linear', output_path)


def generate_slider_board(start_id: int, end_id: int, dictionary_name: str = 'DICT_4X4_100',
                         marker_size_pixels: int = 200, spacing_pixels: int = 100,
                         output_path: str = None) -> np.ndarray:
    """
    スライダ側マーカーボードを生成する
    
    Args:
        start_id: 開始ID
        end_id: 終了ID
        dictionary_name: 辞書名
        marker_size_pixels: マーカーサイズ（ピクセル）
        spacing_pixels: マーカー間隔（ピクセル）
        output_path: 出力パス
    
    Returns:
        生成されたスライダ側マーカーボード画像
    """
    marker_ids = list(range(start_id, end_id + 1))
    return generate_marker_board(marker_ids, dictionary_name, marker_size_pixels,
                               spacing_pixels, 'grid', output_path)


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='ArUcoマーカーボード生成スクリプト')
    parser.add_argument('--type', type=str, required=True, choices=['base', 'slider'],
                       help='ボードタイプ（base または slider）')
    parser.add_argument('--start-id', type=int, required=True, help='開始ID')
    parser.add_argument('--end-id', type=int, required=True, help='終了ID')
    parser.add_argument('--size', type=int, default=200, help='マーカーサイズ（ピクセル）')
    parser.add_argument('--spacing', type=int, default=100, help='マーカー間隔（ピクセル）')
    parser.add_argument('--dictionary', type=str, default='DICT_4X4_100', 
                       help='使用する辞書名')
    parser.add_argument('--output', type=str, help='出力ファイルパス')
    
    args = parser.parse_args()
    
    # 出力パスの設定
    if args.output is None:
        if args.type == 'base':
            args.output = 'base_board.png'
        else:
            args.output = 'slider_board.png'
    
    # ボードタイプに応じて生成
    if args.type == 'base':
        generate_base_board(args.start_id, args.end_id, args.dictionary,
                          args.size, args.spacing, args.output)
    else:  # slider
        generate_slider_board(args.start_id, args.end_id, args.dictionary,
                            args.size, args.spacing, args.output)


if __name__ == '__main__':
    main() 
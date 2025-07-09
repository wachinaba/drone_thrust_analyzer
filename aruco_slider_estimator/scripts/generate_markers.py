#!/usr/bin/env python3

"""
ARマーカー生成スクリプト

指定したIDのArUcoマーカーを生成し、印刷用に最適化された画像を出力する。
"""

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import sys
from pathlib import Path


def generate_marker(marker_id: int, dictionary_name: str = 'DICT_4X4_100', 
                   marker_size_pixels: int = 200, output_path: str = None) -> np.ndarray:
    """
    指定したIDのArUcoマーカーを生成する
    
    Args:
        marker_id: マーカーID
        dictionary_name: 辞書名
        marker_size_pixels: マーカーサイズ（ピクセル）
        output_path: 出力パス（Noneの場合は画像を返すのみ）
    
    Returns:
        生成されたマーカー画像
    """
    # 辞書の取得
    dictionary_name_attr = getattr(aruco, dictionary_name)
    dictionary = aruco.getPredefinedDictionary(dictionary_name_attr)
    
    # マーカーの生成
    marker_image = aruco.generateImageMarker(dictionary, marker_id, marker_size_pixels)
    
    # 白い背景を追加（印刷用）
    margin = 50  # マージン（ピクセル）
    final_size = marker_size_pixels + 2 * margin
    final_image = np.ones((final_size, final_size), dtype=np.uint8) * 255
    final_image[margin:margin+marker_size_pixels, margin:margin+marker_size_pixels] = marker_image
    
    # ファイルに保存
    if output_path:
        cv2.imwrite(output_path, final_image)
        print(f"マーカーID {marker_id} を {output_path} に保存しました")
    
    return final_image


def generate_markers_batch(start_id: int, end_id: int, dictionary_name: str = 'DICT_4X4_100',
                          marker_size_pixels: int = 200, output_dir: str = None) -> None:
    """
    指定した範囲のマーカーを一括生成する
    
    Args:
        start_id: 開始ID
        end_id: 終了ID
        dictionary_name: 辞書名
        marker_size_pixels: マーカーサイズ（ピクセル）
        output_dir: 出力ディレクトリ
    """
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    
    for marker_id in range(start_id, end_id + 1):
        if output_dir:
            output_path = os.path.join(output_dir, f"marker_{marker_id}.png")
        else:
            output_path = None
        
        generate_marker(marker_id, dictionary_name, marker_size_pixels, output_path)


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='ArUcoマーカー生成スクリプト')
    parser.add_argument('--id', type=int, help='生成するマーカーID')
    parser.add_argument('--start-id', type=int, help='開始ID（一括生成時）')
    parser.add_argument('--end-id', type=int, help='終了ID（一括生成時）')
    parser.add_argument('--size', type=int, default=200, help='マーカーサイズ（ピクセル）')
    parser.add_argument('--dictionary', type=str, default='DICT_4X4_100', 
                       help='使用する辞書名')
    parser.add_argument('--output', type=str, help='出力ファイルパス（単一マーカー時）')
    parser.add_argument('--output-dir', type=str, help='出力ディレクトリ（一括生成時）')
    
    args = parser.parse_args()
    
    # 引数の検証
    if args.id is not None:
        # 単一マーカー生成
        if args.output is None:
            args.output = f"marker_{args.id}.png"
        
        generate_marker(args.id, args.dictionary, args.size, args.output)
        
    elif args.start_id is not None and args.end_id is not None:
        # 一括生成
        if args.output_dir is None:
            args.output_dir = "markers"
        
        generate_markers_batch(args.start_id, args.end_id, args.dictionary, 
                             args.size, args.output_dir)
        
    else:
        print("エラー: --id または --start-id と --end-id を指定してください")
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main() 
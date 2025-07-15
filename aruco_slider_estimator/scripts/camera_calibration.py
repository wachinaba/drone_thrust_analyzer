#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import argparse
import os
import sys
from datetime import datetime


class CameraCalibrator:
    def __init__(self, camera_id=0, board_size=(9, 6), square_size=0.02495):
        """
        カメラキャリブレーターの初期化
        
        Args:
            camera_id (int): カメラデバイスID
            board_size (tuple): チェッカーボードの内部コーナー数 (width, height)
            square_size (float): チェッカーボードの正方形サイズ（メートル）
        """
        self.camera_id = camera_id
        self.board_size = board_size
        self.square_size = square_size
        
        # チェッカーボードの3D座標を生成
        self.objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # キャリブレーション用のデータを格納
        self.objpoints = []  # 3D点
        self.imgpoints = []  # 2D点
        
        # 自動キャプチャ用の変数
        self.last_capture_time = 0
        self.capture_interval = 0.1  # 2秒間隔でキャプチャ
        
        # カメラを初期化
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            print(f"エラー: カメラID {camera_id} を開けませんでした。")
            sys.exit(1)
        
        # MJPGフォーマットを設定
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # ウィンドウ名を設定
        self.window_name = "Camera Calibration"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # キャリブレーション結果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
        
    def detect_checkerboard(self, frame):
        """
        チェッカーボードを検出
        
        Args:
            frame: 入力画像
            
        Returns:
            tuple: (検出成功フラグ, コーナー座標)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
        
        if ret:
            # コーナーの精度を向上
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
        return ret, corners
    
    def capture_frame(self, frame):
        """
        キャリブレーション用のフレームをキャプチャ
        
        Args:
            frame: 入力画像
            
        Returns:
            bool: キャプチャ成功フラグ
        """
        ret, corners = self.detect_checkerboard(frame)
        
        if ret:
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners)
            return True
        
        return False
    
    def calibrate_camera(self):
        """
        カメラキャリブレーションを実行
        
        Returns:
            bool: キャリブレーション成功フラグ
        """
        if len(self.objpoints) < 10:
            print("警告: キャリブレーションには最低10枚の画像が必要です。")
            return False
        
        # 画像サイズを取得
        height, width = self.cap.read()[1].shape[:2]
        img_size = (width, height)
        
        # カメラキャリブレーションを実行
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, img_size, None, None
        )
        
        if ret:
            # キャリブレーション誤差を計算
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], 
                                                self.camera_matrix, self.dist_coeffs)
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            
            self.calibration_error = mean_error / len(self.objpoints)
            return True
        
        return False
    
    def save_camera_info(self, output_path):
        """
        カメラ情報をYAMLファイルとして保存
        
        Args:
            output_path (str): 出力ファイルパス
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("エラー: キャリブレーションが実行されていません。")
            return False
        
        # 画像サイズを取得
        height, width = self.cap.read()[1].shape[:2]
        
        # camera_info形式のデータを作成
        camera_info = {
            'image_width': width,
            'image_height': height,
            'camera_name': 'default_cam',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': self.camera_matrix.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': self.dist_coeffs.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': np.eye(3).flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': np.hstack([self.camera_matrix, np.zeros((3, 1))]).flatten().tolist()
            }
        }
        
        # YAMLファイルとして保存
        with open(output_path, 'w') as f:
            yaml.dump(camera_info, f, default_flow_style=False)
        
        print(f"カメラ情報を保存しました: {output_path}")
        return True
    
    def draw_info(self, frame, corners=None):
        """
        フレームに情報を描画
        
        Args:
            frame: 入力画像
            corners: 検出されたコーナー座標
        """
        # 情報テキストを描画
        info_text = [
            f"Captured images: {len(self.objpoints)}",
            "Auto capture: 2s interval",
            "Press 'Space' for manual capture",
            "Press 'Enter' to calibrate",
            "Press 'Esc' to quit"
        ]
        
        if self.calibration_error is not None:
            info_text.append(f"Calibration error: {self.calibration_error:.4f}")
        
        for i, text in enumerate(info_text):
            cv2.putText(frame, text, (10, 30 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # チェッカーボードが検出された場合、コーナーを描画
        if corners is not None:
            cv2.drawChessboardCorners(frame, self.board_size, corners, True)
            # 検出状態を示すテキストを追加
            cv2.putText(frame, "CHECKERBOARD DETECTED - Press SPACE to capture", 
                       (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # 検出されていない場合のテキスト
            cv2.putText(frame, "No checkerboard detected", 
                       (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    def run(self):
        """
        メインループ
        """
        print("カメラキャリブレーションを開始します...")
        print("操作:")
        print("  Space: フレームをキャプチャ")
        print("  Enter: キャリブレーション実行")
        print("  Esc: 終了")
        
        import time
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("エラー: フレームを読み込めませんでした。")
                break
            
            # チェッカーボードを検出
            checker_detected, corners = self.detect_checkerboard(frame)
            
            # 自動キャプチャ（チェッカーボードが検出され、一定時間経過した場合）
            current_time = time.time()
            if (checker_detected and 
                current_time - self.last_capture_time > self.capture_interval):
                if self.capture_frame(frame):
                    print(f"自動キャプチャしました。総数: {len(self.objpoints)}")
                    self.last_capture_time = current_time
            
            # 情報を描画
            self.draw_info(frame, corners if checker_detected else None)
            
            # フレームを表示
            cv2.imshow(self.window_name, frame)
            
            # キー入力を処理
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # Esc
                break
            elif key == 32:  # Space
                if checker_detected:
                    if self.capture_frame(frame):
                        print(f"手動キャプチャしました。総数: {len(self.objpoints)}")
                        self.last_capture_time = current_time
                    else:
                        print("フレームのキャプチャに失敗しました。")
                else:
                    print("チェッカーボードが検出されていません。")
            elif key == 13:  # Enter
                if self.calibrate_camera():
                    print("キャリブレーションが完了しました。")
                    
                    # 結果を保存
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_path = f"camera_info_{timestamp}.yaml"
                    self.save_camera_info(output_path)
                else:
                    print("キャリブレーションに失敗しました。")
        
        # クリーンアップ
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='カメラキャリブレーション')
    parser.add_argument('--camera', type=int, default=0, 
                       help='カメラデバイスID (デフォルト: 0)')
    parser.add_argument('--board-size', type=int, nargs=2, default=[9, 6],
                       help='チェッカーボードの内部コーナー数 (デフォルト: 9 6)')
    parser.add_argument('--square-size', type=float, default=0.02495,
                       help='チェッカーボードの正方形サイズ（メートル） (デフォルト: 0.025)')
    
    args = parser.parse_args()
    
    # キャリブレーターを作成して実行
    calibrator = CameraCalibrator(
        camera_id=args.camera,
        board_size=tuple(args.board_size),
        square_size=args.square_size
    )
    
    try:
        calibrator.run()
    except KeyboardInterrupt:
        print("\nキャリブレーションを中断しました。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")


if __name__ == "__main__":
    main() 
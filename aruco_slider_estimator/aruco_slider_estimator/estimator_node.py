#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import yaml
from typing import Optional, Tuple, List, Dict, Any


class ArucoSliderEstimator(Node):
    """
    ARマーカー式スライダ位置推定ノード
    
    静止したベース部分と移動するスライダ部分の両方にマーカーを設置し、
    カメラからの単一画像内で両者を同時に認識することで、
    ロバストで高精度なスライダのワールド座標系における位置姿勢を推定する。
    """
    
    def __init__(self):
        super().__init__('estimator_node')
        
        # パラメータの読み込み
        self._load_parameters()
        
        # OpenCV ArUco検出器の初期化
        self._init_aruco_detector()
        
        # マーカーボードの初期化
        self._init_marker_boards()
        
        # カメラの初期化
        self._init_camera()
        
        # サブスクライバとパブリッシャの初期化
        self._init_ros_interfaces()
        
        # CV Bridge（NumPy 2.x対応のため一時的に無効化）
        # self.bridge = CvBridge()
        
        # タイマーの作成（カメラ画像の取得用）
        self.timer = self.create_timer(0.033, self._camera_timer_callback)  # 30 FPS
        
        self.get_logger().info('ArucoSliderEstimator initialized successfully')
    
    def _load_parameters(self):
        """パラメータの読み込み"""
        self.marker_dictionary = self.declare_parameter(
            'marker_dictionary', 'DICT_4X4_100').value
        
        self.base_board_config = self.declare_parameter(
            'base_board_config', '').value
        
        self.slider_board_config = self.declare_parameter(
            'slider_board_config', '').value
        
        self.detector_params_file = self.declare_parameter(
            'detector_params_file', '').value
        
        self.world_frame_id = self.declare_parameter(
            'world_frame_id', 'world').value
        
        self.slider_frame_id = self.declare_parameter(
            'slider_frame_id', 'slider_base').value
        
        self.publish_tf = self.declare_parameter(
            'publish_tf', True).value
        
        self.show_debug_image = self.declare_parameter(
            'show_debug_image', True).value
        
        # カメラパラメータ
        self.camera_id = self.declare_parameter('camera_id', 0).value
        self.camera_info_file = self.declare_parameter('camera_info_file', '').value
        
        # パラメータの検証
        if not self.base_board_config or not self.slider_board_config:
            raise ValueError("base_board_config and slider_board_config must be specified")
    
    def _init_aruco_detector(self):
        """ArUco検出器の初期化"""
        # 辞書の取得
        dictionary_name = getattr(aruco, self.marker_dictionary)
        self.dictionary = aruco.getPredefinedDictionary(dictionary_name)
        
        # 検出器パラメータの設定
        self.detector_params = aruco.DetectorParameters()
        
        # マーカー検出の精度向上のためのパラメータ調整
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 10
        self.detector_params.adaptiveThreshConstant = 7
        self.detector_params.minMarkerPerimeterRate = 0.03
        self.detector_params.maxMarkerPerimeterRate = 4.0
        self.detector_params.polygonalApproxAccuracyRate = 0.03
        self.detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementWinSize = 5
        self.detector_params.cornerRefinementMaxIterations = 30
        self.detector_params.cornerRefinementMinAccuracy = 0.1
        self.detector_params.markerBorderBits = 1
        self.detector_params.minDistanceToBorder = 3
        self.detector_params.perspectiveRemovePixelPerCell = 4
        self.detector_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.detector_params.maxErroneousBitsInBorderRate = 0.35
        self.detector_params.minOtsuStdDev = 5.0
        self.detector_params.errorCorrectionRate = 0.6
        
        if self.detector_params_file:
            try:
                with open(self.detector_params_file, 'r') as f:
                    params_dict = yaml.safe_load(f)
                    for key, value in params_dict.items():
                        if hasattr(self.detector_params, key):
                            setattr(self.detector_params, key, value)
            except Exception as e:
                self.get_logger().warn(f"Failed to load detector parameters: {e}")
        
        # 検出器の作成
        self.detector = aruco.ArucoDetector(self.dictionary, self.detector_params)
        
        self.get_logger().info(f"ArUco detector initialized with dictionary: {self.marker_dictionary}")
    
    def _init_marker_boards(self):
        """マーカーボードの初期化"""
        # ベース側マーカーボードの読み込み
        self.base_board = self._load_marker_board(self.base_board_config)
        
        # スライダ側マーカーボードの読み込み
        self.slider_board = self._load_marker_board(self.slider_board_config)
        
        self.get_logger().info("Marker boards initialized")
    
    def _init_camera(self):
        """カメラの初期化"""
        # OpenCVカメラの初期化
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id}")
            raise RuntimeError(f"Camera {self.camera_id} not available")
        
        # カメラ設定
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # カメラ情報の読み込み
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 表示用ウィンドウの初期化
        self.show_window = self.declare_parameter('show_window', True).value
        if self.show_window:
            cv2.namedWindow('AR Marker Detection', cv2.WINDOW_AUTOSIZE)
        
        if self.camera_info_file:
            try:
                with open(self.camera_info_file, 'r') as f:
                    camera_info = yaml.safe_load(f)
                
                # カメラ行列の読み込み
                camera_matrix_data = camera_info['camera_matrix']['data']
                self.camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
                
                # 歪み係数の読み込み
                dist_coeffs_data = camera_info['distortion_coefficients']['data']
                self.dist_coeffs = np.array(dist_coeffs_data)
                
                self.get_logger().info(f"Camera info loaded from {self.camera_info_file}")
                
            except Exception as e:
                self.get_logger().warn(f"Failed to load camera info: {e}")
        
        self.get_logger().info(f"Camera initialized: ID={self.camera_id}")
    
    def _load_marker_board(self, config_file: str) -> Dict[str, Any]:
        """マーカーボード設定ファイルの読み込み"""
        try:
            with open(config_file, 'r') as f:
                if config_file.endswith('.json'):
                    config = json.load(f)
                else:
                    config = yaml.safe_load(f)
            
            # 設定の検証
            if 'marker_size_mm' not in config or 'markers' not in config:
                raise ValueError("Invalid marker board config format")
            
            return config
            
        except Exception as e:
            self.get_logger().error(f"Failed to load marker board config {config_file}: {e}")
            raise
    
    def _init_ros_interfaces(self):
        """ROSインターフェースの初期化"""
        # パブリッシャ
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '~/output/slider_pose',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '~/output/debug_image',
            10
        )
        
        # TFブロードキャスタ
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("ROS interfaces initialized")
    
    def _camera_timer_callback(self):
        """カメラタイマーコールバック - メイン処理"""
        if self.camera_matrix is None:
            self.get_logger().warn("Camera info not loaded yet")
            return
        
        try:
            # カメラから画像を取得
            ret, cv_image = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read from camera")
                return
            
            # マーカー検出
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            
            if ids is None:
                self.get_logger().debug("No markers detected")
                return
            
            # Pose推定処理
            slider_pose = self._estimate_slider_pose(corners, ids, cv_image)
            
            # 表示用画像の作成
            display_image = cv_image.copy()
            if slider_pose is not None:
                # 結果のパブリッシュ
                self._publish_slider_pose(slider_pose, self.get_clock().now().to_msg())
                
                # デバッグ画像のパブリッシュ
                if self.show_debug_image:
                    debug_image = self._create_debug_image(cv_image, corners, ids, slider_pose)
                    self._publish_debug_image(debug_image, self.get_clock().now().to_msg())
                
                # 表示用画像にPose情報を追加
                display_image = self._create_display_image(display_image, corners, ids, slider_pose)

                self.get_logger().info(f"slider_pose: {slider_pose}")
            else:
                # マーカーが検出されたがPose推定に失敗した場合
                if ids is not None:
                    aruco.drawDetectedMarkers(display_image, corners, ids)
                    cv2.putText(display_image, "Pose estimation failed", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # ウィンドウに表示
            if self.show_window:
                cv2.imshow('AR Marker Detection', display_image)
                cv2.waitKey(1)  # 1ms待機（ウィンドウ更新用）
        
        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")
    
    def _estimate_slider_pose(self, corners: List, ids: List, image: np.ndarray) -> Optional[np.ndarray]:
        """スライダのPose推定（ワールド座標系から見たスライダの位置姿勢）"""
        try:
            # ベース側マーカーの抽出
            base_corners, base_ids = self._extract_markers_by_board(
                corners, ids, self.base_board)
            
            if base_corners is None or len(base_corners) < 1:
                self.get_logger().debug("Insufficient base markers detected")
                return None
            
            # ワールド座標系 → カメラ座標系のPose推定
            world_to_camera = self._estimate_camera_pose(base_corners, base_ids)
            
            if world_to_camera is None:
                self.get_logger().debug("Failed to estimate camera pose")
                return None
            
            # スライダ側マーカーの抽出
            slider_corners, slider_ids = self._extract_markers_by_board(
                corners, ids, self.slider_board)
            
            if slider_corners is None or len(slider_corners) < 1:
                self.get_logger().debug("Insufficient slider markers detected")
                return None
            
            # スライダのローカル座標系でのPose推定
            slider_local_pose = self._estimate_slider_local_pose(slider_corners, slider_ids)
            
            if slider_local_pose is None:
                self.get_logger().debug("Failed to estimate slider local pose")
                return None
            
            # スライダのローカル座標系からワールド座標系への変換
            world_to_slider = self._convert_slider_local_to_world(
                slider_local_pose, world_to_camera)
            
            self.get_logger().debug(f"Camera position: {world_to_camera[:3, 3]}")
            self.get_logger().debug(f"Slider position (world): {world_to_slider[:3, 3]}")
            
            return world_to_slider
            
        except Exception as e:
            self.get_logger().error(f"Error in pose estimation: {e}")
            return None
    
    def _estimate_slider_local_pose(self, corners: List, ids: List) -> Optional[np.ndarray]:
        """スライダのローカル座標系でのPose推定"""
        try:
            # スライダの中心位置を計算（スライダ側マーカーの平均位置）
            slider_center = np.mean([marker['translation'] for marker in self.slider_board['markers']], axis=0)
            self.get_logger().debug(f"Slider center in local coordinates: {slider_center}")
            
            # スライダのローカル座標系でのマーカー位置を計算
            local_marker_positions = []
            for marker in self.slider_board['markers']:
                # スライダ中心を原点とした相対位置
                relative_pos = np.array(marker['translation']) - slider_center
                local_marker_positions.append(relative_pos)
            
            # 検出されたマーカーとローカル座標系の対応を取得
            obj_points, img_points = self._get_slider_local_points(
                corners, ids, local_marker_positions)
            
            if len(obj_points) < 1:
                self.get_logger().debug("Insufficient object points for slider local pose estimation")
                return None
            
            # solvePnPRansacでPose推定
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                np.array(obj_points, dtype=np.float32),
                np.array(img_points, dtype=np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not success:
                self.get_logger().debug("Failed to estimate slider local pose")
                return None
            
            # 回転ベクトルを回転行列に変換
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            
            # 4x4の同次変換行列を作成（カメラ座標系から見たスライダローカル座標系）
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()
            
            return transform_matrix
            
        except Exception as e:
            self.get_logger().error(f"Error in slider local pose estimation: {e}")
            return None
    
    def _get_slider_local_points(self, corners: List, ids: List, local_positions: List) -> Tuple[List, List]:
        """スライダのローカル座標系での3Dオブジェクトポイントと2Dイメージポイントの対応を取得"""
        obj_points = []
        img_points = []
        
        marker_size_mm = self.slider_board['marker_size_mm']
        markers = self.slider_board['markers']
        
        # マーカーIDから設定を取得する辞書を作成
        marker_configs = {marker['id']: marker for marker in markers}
        
        for i, marker_id in enumerate(ids):
            if marker_id in marker_configs:
                config = marker_configs[marker_id]
                # スライダ中心を原点とした相対位置を使用
                local_pos = np.array(config['translation']) - np.mean([marker['translation'] for marker in markers], axis=0)
                
                # マーカーの3Dコーナー座標を計算（ローカル座標系）
                half_size = marker_size_mm / 2.0
                marker_corners_3d = [
                    [local_pos[0] - half_size, local_pos[1] - half_size, local_pos[2]],
                    [local_pos[0] + half_size, local_pos[1] - half_size, local_pos[2]],
                    [local_pos[0] + half_size, local_pos[1] + half_size, local_pos[2]],
                    [local_pos[0] - half_size, local_pos[1] + half_size, local_pos[2]]
                ]
                
                # 2Dコーナー座標を取得
                marker_corners_2d = corners[i][0]
                
                # 対応点を追加
                obj_points.extend(marker_corners_3d)
                img_points.extend(marker_corners_2d)
        
        return obj_points, img_points
    
    def _convert_slider_local_to_world(self, slider_local_pose: np.ndarray, world_to_camera: np.ndarray) -> np.ndarray:
        """スライダのローカル座標系からワールド座標系への変換"""
        # カメラ座標系から見たスライダローカル座標系のPose
        camera_to_slider_local = slider_local_pose
        
        # ワールド座標系から見たカメラのPose
        camera_to_world = np.linalg.inv(world_to_camera)
        
        # ワールド座標系から見たスライダローカル座標系のPose
        world_to_slider_local = camera_to_world @ camera_to_slider_local
        
        # スライダの中心位置をワールド座標系に変換
        slider_center_local = np.mean([marker['translation'] for marker in self.slider_board['markers']], axis=0)
        slider_center_world = world_to_slider_local[:3, :3] @ slider_center_local + world_to_slider_local[:3, 3]
        
        # ワールド座標系から見たスライダのPose行列を作成
        world_to_slider = np.eye(4)
        world_to_slider[:3, :3] = world_to_slider_local[:3, :3]
        world_to_slider[:3, 3] = slider_center_world
        
        return world_to_slider
    
    def _extract_markers_by_board(self, corners: List, ids: List, board_config: Dict) -> Tuple[Optional[List], Optional[List]]:
        """指定されたボード設定に基づいてマーカーを抽出"""
        board_ids = [marker['id'] for marker in board_config['markers']]
        
        extracted_corners = []
        extracted_ids = []
        
        for i, marker_id in enumerate(ids):
            if marker_id[0] in board_ids:
                extracted_corners.append(corners[i])
                extracted_ids.append(marker_id[0])
        
        if len(extracted_corners) < 1:
            return None, None
        
        return extracted_corners, extracted_ids
    
    def _estimate_camera_pose(self, corners: List, ids: List) -> Optional[np.ndarray]:
        """カメラのPose推定（ワールド座標系から見たカメラの位置姿勢）"""
        try:
            # 3Dオブジェクトポイントと2Dイメージポイントの対応を取得
            obj_points, img_points = self._get_board_points(
                corners, ids, self.base_board)
            
            if len(obj_points) < 4:
                self.get_logger().debug("Insufficient object points for camera pose estimation")
                return None
            
            # solvePnPRansacでPose推定
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                np.array(obj_points, dtype=np.float32),
                np.array(img_points, dtype=np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not success:
                self.get_logger().debug("Failed to estimate camera pose")
                return None
            
            # 回転ベクトルを回転行列に変換
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            
            # 4x4の同次変換行列を作成
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()
            
            return transform_matrix
            
        except Exception as e:
            self.get_logger().error(f"Error in camera pose estimation: {e}")
            return None
    
    def _get_board_points(self, corners: List, ids: List, board_config: Dict) -> Tuple[List, List]:
        """ボード設定に基づいて3Dオブジェクトポイントと2Dイメージポイントの対応を取得"""
        obj_points = []
        img_points = []
        
        marker_size_mm = board_config['marker_size_mm']
        markers = board_config['markers']
        
        # マーカーIDから設定を取得する辞書を作成
        marker_configs = {marker['id']: marker for marker in markers}
        
        for i, marker_id in enumerate(ids):
            if marker_id in marker_configs:
                config = marker_configs[marker_id]
                translation = config['translation']
                
                # マーカーの3Dコーナー座標を計算
                half_size = marker_size_mm / 2.0
                marker_corners_3d = [
                    [translation[0] - half_size, translation[1] - half_size, translation[2]],
                    [translation[0] + half_size, translation[1] - half_size, translation[2]],
                    [translation[0] + half_size, translation[1] + half_size, translation[2]],
                    [translation[0] - half_size, translation[1] + half_size, translation[2]]
                ]
                
                # 2Dコーナー座標を取得
                marker_corners_2d = corners[i][0]
                
                # 対応点を追加
                obj_points.extend(marker_corners_3d)
                img_points.extend(marker_corners_2d)
        
        return obj_points, img_points
    
    def _publish_slider_pose(self, transform_matrix: np.ndarray, timestamp):
        """スライダのPoseをパブリッシュ"""
        # 変換行列から位置と姿勢を抽出
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        
        # 回転行列をクォータニオンに変換
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)
        
        # PoseStampedメッセージを作成
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.world_frame_id
        
        pose_msg.pose.position.x = float(position[0]) / 1000.0
        pose_msg.pose.position.y = float(position[1]) / 1000.0
        pose_msg.pose.position.z = float(position[2]) / 1000.0
        
        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])
        
        # パブリッシュ
        self.pose_pub.publish(pose_msg)
        
        # TFブロードキャスト
        if self.publish_tf:
            self._broadcast_tf(transform_matrix, timestamp)
    
    def _broadcast_tf(self, transform_matrix: np.ndarray, timestamp):
        """TFブロードキャスト"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.slider_frame_id
        
        # 位置（mmからメートルに変換）
        t.transform.translation.x = float(transform_matrix[0, 3]) / 1000.0
        t.transform.translation.y = float(transform_matrix[1, 3]) / 1000.0
        t.transform.translation.z = float(transform_matrix[2, 3]) / 1000.0
        
        # 回転（クォータニオン）
        rotation_matrix = transform_matrix[:3, :3]
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """回転行列をクォータニオンに変換"""
        # OpenCVのRodrigues関数を使用して回転ベクトルに変換
        rvec, _ = cv2.Rodrigues(rotation_matrix)
        
        # 回転ベクトルからクォータニオンを計算
        angle = np.linalg.norm(rvec)
        if angle < 1e-6:
            # 回転が非常に小さい場合
            quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            axis = rvec.flatten() / angle
            quaternion = np.array([
                axis[0] * np.sin(angle / 2.0),
                axis[1] * np.sin(angle / 2.0),
                axis[2] * np.sin(angle / 2.0),
                np.cos(angle / 2.0)
            ])
        
        return quaternion
    
    def _create_debug_image(self, image: np.ndarray, corners: List, ids: List, 
                           slider_pose: np.ndarray) -> np.ndarray:
        """デバッグ画像の作成"""
        debug_image = image.copy()
        
        # 検出されたマーカーを描画
        if ids is not None:
            aruco.drawDetectedMarkers(debug_image, corners, ids)
        
        # スライダの座標軸を描画
        if slider_pose is not None:
            # スライダの位置
            slider_position = slider_pose[:3, 3]
            
            # 座標軸の長さ（ピクセル）
            axis_length = 50
            
            # 座標軸の方向ベクトル
            x_axis = slider_pose[:3, 0] * axis_length
            y_axis = slider_pose[:3, 1] * axis_length
            z_axis = slider_pose[:3, 2] * axis_length
            
            # 3D点を2Dに投影
            points_3d = np.array([
                [0, 0, 0],
                [x_axis[0], x_axis[1], x_axis[2]],
                [y_axis[0], y_axis[1], y_axis[2]],
                [z_axis[0], z_axis[1], z_axis[2]]
            ], dtype=np.float32)
            
            points_2d, _ = cv2.projectPoints(
                points_3d,
                np.zeros(3),  # カメラ座標系での回転は0
                np.zeros(3),  # カメラ座標系での並進は0
                self.camera_matrix,
                self.dist_coeffs
            )
            
            # 座標軸を描画
            try:
                origin = (int(points_2d[0][0][0]), int(points_2d[0][0][1]))
                x_end = (int(points_2d[1][0][0]), int(points_2d[1][0][1]))
                y_end = (int(points_2d[2][0][0]), int(points_2d[2][0][1]))
                z_end = (int(points_2d[3][0][0]), int(points_2d[3][0][1]))
                
                # 画像の範囲内かチェック
                height, width = debug_image.shape[:2]
                def is_valid_point(point):
                    return 0 <= point[0] < width and 0 <= point[1] < height
                
                if (is_valid_point(origin) and is_valid_point(x_end) and 
                    is_valid_point(y_end) and is_valid_point(z_end)):
                    cv2.line(debug_image, origin, x_end, (0, 0, 255), 2)  # X軸（赤）
                    cv2.line(debug_image, origin, y_end, (0, 255, 0), 2)  # Y軸（緑）
                    cv2.line(debug_image, origin, z_end, (255, 0, 0), 2)  # Z軸（青）
                else:
                    self.get_logger().debug("Coordinate axes points outside image bounds")
            except Exception as e:
                self.get_logger().debug(f"Failed to draw coordinate axes in debug image: {e}")
        
        return debug_image
    
    def _create_display_image(self, image: np.ndarray, corners: List, ids: List, 
                             slider_pose: np.ndarray) -> np.ndarray:
        """表示用画像の作成"""
        display_image = image.copy()
        
        # 検出されたマーカーを描画
        if ids is not None:
            aruco.drawDetectedMarkers(display_image, corners, ids)
        
        # スライダのPose情報を表示
        if slider_pose is not None:
            # スライダの位置
            position = slider_pose[:3, 3]
            
            # 位置情報をテキストで表示
            pos_text = f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
            cv2.putText(display_image, pos_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 座標軸を描画
            axis_length = 50
            x_axis = slider_pose[:3, 0] * axis_length
            y_axis = slider_pose[:3, 1] * axis_length
            z_axis = slider_pose[:3, 2] * axis_length
            
            # 3D点を2Dに投影
            points_3d = np.array([
                [0, 0, 0],
                [x_axis[0], x_axis[1], x_axis[2]],
                [y_axis[0], y_axis[1], y_axis[2]],
                [z_axis[0], z_axis[1], z_axis[2]]
            ], dtype=np.float32)
            
            points_2d, _ = cv2.projectPoints(
                points_3d,
                np.zeros(3),
                np.zeros(3),
                self.camera_matrix,
                self.dist_coeffs
            )
            
            # 座標軸を描画
            try:
                origin = (int(points_2d[0][0][0]), int(points_2d[0][0][1]))
                x_end = (int(points_2d[1][0][0]), int(points_2d[1][0][1]))
                y_end = (int(points_2d[2][0][0]), int(points_2d[2][0][1]))
                z_end = (int(points_2d[3][0][0]), int(points_2d[3][0][1]))
                
                # 画像の範囲内かチェック
                height, width = display_image.shape[:2]
                def is_valid_point(point):
                    return 0 <= point[0] < width and 0 <= point[1] < height
                
                if (is_valid_point(origin) and is_valid_point(x_end) and 
                    is_valid_point(y_end) and is_valid_point(z_end)):
                    cv2.line(display_image, origin, x_end, (0, 0, 255), 2)  # X軸（赤）
                    cv2.line(display_image, origin, y_end, (0, 255, 0), 2)  # Y軸（緑）
                    cv2.line(display_image, origin, z_end, (255, 0, 0), 2)  # Z軸（青）
                else:
                    self.get_logger().debug("Coordinate axes points outside image bounds")
            except Exception as e:
                self.get_logger().debug(f"Failed to draw coordinate axes: {e}")
        
        return display_image
    
    def _publish_debug_image(self, debug_image: np.ndarray, timestamp):
        """デバッグ画像をパブリッシュ"""
        try:
            # NumPy 2.x対応のため一時的に無効化
            # debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            # debug_msg.header.stamp = timestamp
            # self.debug_image_pub.publish(debug_msg)
            self.get_logger().debug("Debug image publishing temporarily disabled for NumPy 2.x compatibility")
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")
    
    def destroy_node(self):
        """ノードの破棄"""
        if hasattr(self, 'cap'):
            self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoSliderEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
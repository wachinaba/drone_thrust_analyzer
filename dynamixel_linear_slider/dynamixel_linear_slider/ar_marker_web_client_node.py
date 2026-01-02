#!/usr/bin/env python3

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import yaml


class ArMarkerWebClientNode(Node):
    """
    Windows 側の AR マーカー検出 Web サーバから 2D corners 情報を取得し、
    既存の ArUco ベース推定ロジックと同様に PnP を用いて
    「ワールド座標系から見たスライダ中心位置」を推定し、
    その 1 次元成分を ar_slider_position [m] として publish するノード。

    Web API のレスポンス形式 (JSON) は次を想定:
    {
        "timestamp": 1234567890.123,
        "image_width": 1280,
        "image_height": 720,
        "markers": [
            {
                "id": 10,
                "corners": [[u1, v1], [u2, v2], [u3, v3], [u4, v4]]
            },
            ...
        ]
    }
    """

    def __init__(self) -> None:
        super().__init__("ar_marker_web_client_node")

        # --- パラメータ ---
        self.declare_parameter("server_url", "http://localhost:8000/detect")
        self.declare_parameter("poll_frequency", 20.0)  # Hz（カメラ側と合わせやすい値）

        # ArUco / ボード / カメラ設定
        self.declare_parameter("base_board_config", "")
        self.declare_parameter("slider_board_config", "")
        self.declare_parameter("camera_info_file", "")

        # スライダ位置の 1D 抽出
        self.declare_parameter("position_axis", "x")  # x / y / z
        self.declare_parameter("scale_factor", 1.0)   # 向き反転などが必要なら -1.0

        self.server_url: str = self.get_parameter("server_url").value
        self.poll_frequency: float = float(self.get_parameter("poll_frequency").value)

        self.base_board_config: str = self.get_parameter("base_board_config").value
        self.slider_board_config: str = self.get_parameter("slider_board_config").value
        self.camera_info_file: str = self.get_parameter("camera_info_file").value

        self.position_axis: str = self.get_parameter("position_axis").value
        self.scale_factor: float = float(self.get_parameter("scale_factor").value)

        if not self.base_board_config or not self.slider_board_config:
            raise ValueError(
                "base_board_config and slider_board_config must be specified"
            )

        # --- カメラパラメータ読み込み ---
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._load_camera_info()

        # --- マーカーボード設定読み込み ---
        self.base_board = self._load_marker_board(self.base_board_config)
        self.slider_board = self._load_marker_board(self.slider_board_config)

        # Publisher
        self.ar_position_pub = self.create_publisher(
            Float64,
            "ar_slider_position",
            10,
        )

        # Timer
        period = 1.0 / self.poll_frequency if self.poll_frequency > 0.0 else 0.2
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("ArMarkerWebClientNode initialized (PnP mode)")
        self.get_logger().info(f"server_url={self.server_url}")
        self.get_logger().info(f"base_board_config={self.base_board_config}")
        self.get_logger().info(f"slider_board_config={self.slider_board_config}")
        self.get_logger().info(f"camera_info_file={self.camera_info_file}")
        self.get_logger().info(
            f"position_axis={self.position_axis}, scale_factor={self.scale_factor}"
        )

    # --- 初期化系 ---

    def _load_camera_info(self) -> None:
        """カメラ行列と歪み係数を YAML から読み込む"""
        if not self.camera_info_file:
            self.get_logger().warn("camera_info_file is not specified")
            return

        try:
            with open(self.camera_info_file, "r") as f:
                camera_info = yaml.safe_load(f)

            camera_matrix_data = camera_info["camera_matrix"]["data"]
            self.camera_matrix = np.array(camera_matrix_data).reshape(3, 3)

            dist_coeffs_data = camera_info["distortion_coefficients"]["data"]
            self.dist_coeffs = np.array(dist_coeffs_data)

            self.get_logger().info(
                f"Camera info loaded from {self.camera_info_file}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load camera info: {e}")
            self.camera_matrix = None
            self.dist_coeffs = None

    def _load_marker_board(self, config_file: str) -> Dict[str, Any]:
        """マーカーボード設定ファイルの読み込み（JSON/YAML対応）"""
        try:
            with open(config_file, "r") as f:
                if config_file.endswith(".json"):
                    config = json.load(f)
                else:
                    config = yaml.safe_load(f)

            if "marker_size_mm" not in config or "markers" not in config:
                raise ValueError("Invalid marker board config format")

            return config
        except Exception as e:
            self.get_logger().error(
                f"Failed to load marker board config {config_file}: {e}"
            )
            raise

    # --- メイン処理 ---

    def timer_callback(self) -> None:
        """Web API から corners を取得し、PnP でスライダ位置を推定して publish"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().debug("Camera info not loaded yet")
            return

        data = self._fetch_marker_data()
        if data is None:
            return

        corners_cv, ids_cv = self._convert_json_to_cv_format(data)
        if corners_cv is None or ids_cv is None:
            return

        # ベースボードのマーカーから world_to_camera を推定
        base_corners, base_ids = self._extract_markers_by_board(
            corners_cv, ids_cv, self.base_board
        )
        if base_corners is None or base_ids is None:
            self.get_logger().debug("No base board markers found in frame")
            return

        world_to_camera = self._estimate_camera_pose(base_corners, base_ids)
        if world_to_camera is None:
            self.get_logger().debug("Failed to estimate camera pose")
            return

        # スライダボードのマーカーから slider_local_pose を推定
        slider_corners, slider_ids = self._extract_markers_by_board(
            corners_cv, ids_cv, self.slider_board
        )
        if slider_corners is None or slider_ids is None:
            self.get_logger().debug("No slider board markers found in frame")
            return

        slider_local_pose = self._estimate_slider_local_pose(
            slider_corners, slider_ids
        )
        if slider_local_pose is None:
            self.get_logger().debug("Failed to estimate slider local pose")
            return

        world_to_slider = self._convert_slider_local_to_world(
            slider_local_pose, world_to_camera
        )

        slider_pos_m = self._extract_1d_position(world_to_slider)
        if slider_pos_m is None:
            return

        msg = Float64()
        msg.data = slider_pos_m
        self.ar_position_pub.publish(msg)

        self.get_logger().debug(
            f"Slider position ({self.position_axis}) = {slider_pos_m:.4f} m"
        )

    def _fetch_marker_data(self) -> Optional[Dict[str, Any]]:
        """HTTP で Web サーバから JSON を取得"""
        try:
            with urllib.request.urlopen(self.server_url, timeout=0.5) as response:
                if response.status != 200:
                    self.get_logger().warn(
                        f"HTTP request failed with status {response.status}"
                    )
                    return None
                data_bytes = response.read()
        except (urllib.error.URLError, TimeoutError) as e:
            self.get_logger().debug(f"Failed to contact AR server: {e}")
            return None
        except Exception as e:
            self.get_logger().warn(f"Unexpected error while contacting AR server: {e}")
            return None

        try:
            data = json.loads(data_bytes.decode("utf-8"))
            return data
        except Exception as e:
            self.get_logger().warn(f"Failed to parse JSON from AR server: {e}")
            return None

    def _convert_json_to_cv_format(
        self, data: Dict[str, Any]
    ) -> Tuple[Optional[List[np.ndarray]], Optional[np.ndarray]]:
        """
        JSON 形式の markers 配列を、既存の OpenCV ArUco 検出結果に似た形式
        (corners: List[np.ndarray(shape=(1,4,2))], ids: np.ndarray(shape=(N,1))) に変換。
        """
        markers = data.get("markers")
        if not isinstance(markers, list) or len(markers) == 0:
            return None, None

        corners_cv: List[np.ndarray] = []
        ids_list: List[int] = []

        for m in markers:
            try:
                mid = int(m.get("id"))
                cs = m.get("corners", [])
                if not isinstance(cs, list) or len(cs) != 4:
                    continue
                pts = np.array(cs, dtype=np.float32).reshape(1, 4, 2)
            except Exception:
                continue

            ids_list.append(mid)
            corners_cv.append(pts)

        if not ids_list:
            return None, None

        ids_cv = np.array(ids_list, dtype=np.int32).reshape(-1, 1)
        return corners_cv, ids_cv

    # --- PnP / 幾何計算関連 ---

    def _extract_markers_by_board(
        self,
        corners: List[np.ndarray],
        ids: np.ndarray,
        board_config: Dict[str, Any],
    ) -> Tuple[Optional[List[np.ndarray]], Optional[List[int]]]:
        """指定されたボード設定に基づいてマーカーを抽出"""
        board_ids = [marker["id"] for marker in board_config["markers"]]

        extracted_corners: List[np.ndarray] = []
        extracted_ids: List[int] = []

        for i, marker_id in enumerate(ids):
            # ids は shape (N,1) の想定
            mid = int(marker_id[0])
            if mid in board_ids:
                extracted_corners.append(corners[i])
                extracted_ids.append(mid)

        if len(extracted_corners) < 1:
            return None, None

        return extracted_corners, extracted_ids

    def _get_board_points(
        self,
        corners: List[np.ndarray],
        ids: List[int],
        board_config: Dict[str, Any],
    ) -> Tuple[List[List[float]], List[List[float]]]:
        """ボード設定に基づいて3Dオブジェクトポイントと2Dイメージポイントの対応を取得"""
        obj_points: List[List[float]] = []
        img_points: List[List[float]] = []

        marker_size_mm = board_config["marker_size_mm"]
        markers = board_config["markers"]

        marker_configs = {marker["id"]: marker for marker in markers}

        for i, marker_id in enumerate(ids):
            if marker_id in marker_configs:
                config = marker_configs[marker_id]
                translation = config["translation"]

                half_size = marker_size_mm / 2.0
                marker_corners_3d = [
                    [
                        translation[0] - half_size,
                        translation[1] - half_size,
                        translation[2],
                    ],
                    [
                        translation[0] + half_size,
                        translation[1] - half_size,
                        translation[2],
                    ],
                    [
                        translation[0] + half_size,
                        translation[1] + half_size,
                        translation[2],
                    ],
                    [
                        translation[0] - half_size,
                        translation[1] + half_size,
                        translation[2],
                    ],
                ]

                marker_corners_2d = corners[i][0]

                obj_points.extend(marker_corners_3d)
                img_points.extend(marker_corners_2d)

        return obj_points, img_points

    def _estimate_camera_pose(
        self,
        corners: List[np.ndarray],
        ids: List[int],
    ) -> Optional[np.ndarray]:
        """カメラの Pose 推定（ワールド座標系から見たカメラの位置姿勢）"""
        try:
            obj_points, img_points = self._get_board_points(
                corners, ids, self.base_board
            )

            if len(obj_points) < 4:
                self.get_logger().debug(
                    "Insufficient object points for camera pose estimation"
                )
                return None

            success, rvec, tvec, _ = cv2.solvePnPRansac(
                np.array(obj_points, dtype=np.float32),
                np.array(img_points, dtype=np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not success:
                self.get_logger().debug("Failed to estimate camera pose")
                return None

            rotation_matrix, _ = cv2.Rodrigues(rvec)

            transform_matrix = np.eye(4, dtype=np.float64)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()

            return transform_matrix
        except Exception as e:
            self.get_logger().error(f"Error in camera pose estimation: {e}")
            return None

    def _get_slider_local_points(
        self,
        corners: List[np.ndarray],
        ids: List[int],
    ) -> Tuple[List[List[float]], List[List[float]]]:
        """スライダローカル座標系での 3D–2D 対応点を取得"""
        obj_points: List[List[float]] = []
        img_points: List[List[float]] = []

        marker_size_mm = self.slider_board["marker_size_mm"]
        markers = self.slider_board["markers"]

        marker_configs = {marker["id"]: marker for marker in markers}
        slider_center = np.mean(
            [marker["translation"] for marker in markers],
            axis=0,
        )

        for i, marker_id in enumerate(ids):
            if marker_id in marker_configs:
                config = marker_configs[marker_id]
                translation = np.array(config["translation"])
                local_pos = translation - slider_center

                half_size = marker_size_mm / 2.0
                marker_corners_3d = [
                    [
                        local_pos[0] - half_size,
                        local_pos[1] - half_size,
                        local_pos[2],
                    ],
                    [
                        local_pos[0] + half_size,
                        local_pos[1] - half_size,
                        local_pos[2],
                    ],
                    [
                        local_pos[0] + half_size,
                        local_pos[1] + half_size,
                        local_pos[2],
                    ],
                    [
                        local_pos[0] - half_size,
                        local_pos[1] + half_size,
                        local_pos[2],
                    ],
                ]

                marker_corners_2d = corners[i][0]

                obj_points.extend(marker_corners_3d)
                img_points.extend(marker_corners_2d)

        return obj_points, img_points

    def _estimate_slider_local_pose(
        self,
        corners: List[np.ndarray],
        ids: List[int],
    ) -> Optional[np.ndarray]:
        """カメラ座標系から見たスライダローカル座標系の Pose を推定"""
        try:
            obj_points, img_points = self._get_slider_local_points(corners, ids)

            if len(obj_points) < 4:
                self.get_logger().debug(
                    "Insufficient object points for slider local pose estimation"
                )
                return None

            success, rvec, tvec, _ = cv2.solvePnPRansac(
                np.array(obj_points, dtype=np.float32),
                np.array(img_points, dtype=np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not success:
                self.get_logger().debug("Failed to estimate slider local pose")
                return None

            rotation_matrix, _ = cv2.Rodrigues(rvec)

            transform_matrix = np.eye(4, dtype=np.float64)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()

            return transform_matrix
        except Exception as e:
            self.get_logger().error(f"Error in slider local pose estimation: {e}")
            return None

    def _convert_slider_local_to_world(
        self,
        slider_local_pose: np.ndarray,
        world_to_camera: np.ndarray,
    ) -> np.ndarray:
        """スライダのローカル座標系からワールド座標系への Pose を計算"""
        camera_to_slider_local = slider_local_pose
        camera_to_world = np.linalg.inv(world_to_camera)
        world_to_slider_local = camera_to_world @ camera_to_slider_local

        slider_center_local = np.mean(
            [marker["translation"] for marker in self.slider_board["markers"]],
            axis=0,
        )
        slider_center_world = (
            world_to_slider_local[:3, :3] @ slider_center_local
            + world_to_slider_local[:3, 3]
        )

        world_to_slider = np.eye(4, dtype=np.float64)
        world_to_slider[:3, :3] = world_to_slider_local[:3, :3]
        world_to_slider[:3, 3] = slider_center_world

        return world_to_slider

    def _extract_1d_position(self, world_to_slider: np.ndarray) -> Optional[float]:
        """4x4 Pose 行列から指定軸の位置成分を [m] で取り出す"""
        axis = self.position_axis.lower()
        idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if idx is None:
            self.get_logger().error(f"Invalid position_axis: {self.position_axis}")
            return None

        # board config は mm 単位なので、m に変換
        pos_mm = float(world_to_slider[idx, 3])
        pos_m = pos_mm / 1000.0
        return pos_m * self.scale_factor


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArMarkerWebClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




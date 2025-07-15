# ARマーカー式スライダ位置推定システム

## 概要

本パッケージは、ARマーカー（ArUcoマーカー）を用いて直線運動するスライダの精密な位置を推定するROS 2パッケージです。

静止したベース部分と移動するスライダ部分の両方にマーカーを設置し、カメラからの単一画像内で両者を同時に認識することで、カメラ自体の微小な振動（ブレ）成分を計算過程で相殺し、ロバストで高精度なスライダのワールド座標系における位置姿勢（Pose）をリアルタイムで提供します。

## システム構成

### ハードウェア構成

- **PC**: ROS 2および本パッケージを実行するコンピュータ
- **カメラ**: スライダに固定され、スライダと共に移動する。ベースとスライダのマーカーを同時に撮影可能な画角を持つこと
- **ベース側マーカー群**: ベースに、スライダの移動軸と平行な一直線上に、等間隔で複数設置される。ワールド座標系の基準となる
- **スライダ側マーカー群**: スライダ本体に、複数個が剛体として固定される。スライダ座標系の基準となる

### ソフトウェア要件

- **OS**: Ubuntu 22.04 (推奨)
- **ROS**: ROS 2 Humble Hawksbill (推奨)
- **主要ライブラリ**: OpenCV 4.x, tf2

## インストール

### 依存関係のインストール

```bash
sudo apt update
sudo apt install python3-opencv python3-yaml
```

### パッケージのビルド

```bash
cd ~/colcon_ws
colcon build --packages-select aruco_slider_estimator
source install/setup.bash
```

## 使用方法

### 1. ARマーカーの生成

本パッケージには、ArUcoマーカーの生成を自動化するスクリプトが含まれています。

#### 単一マーカーの生成

```bash
python3 scripts/generate_markers.py --id 0 --size 200 --dictionary DICT_4X4_100
```

#### マーカーボードの生成

```bash
# ベース側マーカーボード
python3 scripts/generate_board.py --type base --start-id 0 --end-id 4 --spacing 200

# スライダ側マーカーボード
python3 scripts/generate_board.py --type slider --start-id 50 --end-id 53 --spacing 100
```

#### 一括生成

```bash
python3 scripts/generate_all.py --config scripts/config/marker_generation.yaml
```

### 2. マーカー設定ファイルの準備

#### ベース側マーカー設定 (`config/base_board_config.json`)

```json
{
  "marker_size_mm": 50.0,
  "markers": [
    {
      "id": 0,
      "translation": [0.0, 0.0, 0.0]
    },
    {
      "id": 1,
      "translation": [200.0, 0.0, 0.0]
    }
  ]
}
```

#### スライダ側マーカー設定 (`config/slider_board_config.json`)

```json
{
  "marker_size_mm": 50.0,
  "markers": [
    {
      "id": 50,
      "translation": [0.0, 0.0, 0.0]
    },
    {
      "id": 51,
      "translation": [100.0, 0.0, 0.0]
    }
  ]
}
```

**重要**: ベース側マーカー群とスライダ側マーカー群で使用するIDの範囲は、重複しないように割り当ててください。

### 3. カメラキャリブレーション

本システムの精度はカメラキャリブレーションの正確さに大きく依存します。事前に以下のコマンドでカメラキャリブレーションを実行してください：

#### GUI付きキャリブレーション（推奨）

本パッケージに含まれるGUI付きキャリブレーションスクリプトを使用：

```bash
# デフォルト設定で実行
python3 scripts/camera_calibration.py

# カスタム設定で実行
python3 scripts/camera_calibration.py --camera 0 --board-size 8 6 --square-size 0.025
```

**操作方法:**
- **Space**: チェッカーボードが検出された状態でフレームをキャプチャ
- **Enter**: キャリブレーション実行（最低10枚の画像が必要）
- **Esc**: 終了

**注意事項:**
- チェッカーボードが検出されると緑色のコーナーが表示されます
- 様々な角度からチェッカーボードを撮影してください
- キャリブレーション結果は`camera_info_YYYYMMDD_HHMMSS.yaml`として保存されます

#### ROS 2標準キャリブレーション

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
```

### 4. カメラノードの起動

キャリブレーションで生成されたYAMLファイルを使用してカメラノードを起動します：

```bash
# camera_info_urlを使用する方法（推奨）
ros2 launch aruco_slider_estimator camera_with_info.launch.py \
  camera_info_file:=/path/to/camera_info_YYYYMMDD_HHMMSS.yaml \
  camera_id:=0

# または、直接パラメータとして指定する方法
ros2 launch aruco_slider_estimator camera.launch.py \
  camera_info_file:=/path/to/camera_info_YYYYMMDD_HHMMSS.yaml \
  camera_id:=0
```

### 5. 推定ノードの起動

```bash
ros2 launch aruco_slider_estimator estimator.launch.py
```

### 6. カスタムパラメータでの起動

```bash
ros2 launch aruco_slider_estimator estimator.launch.py \
  marker_dictionary:=DICT_4X4_100 \
  base_board_config:=/path/to/base_config.json \
  slider_board_config:=/path/to/slider_config.json \
  world_frame_id:=world \
  slider_frame_id:=slider_base
```

## 出力ファイル

### マーカー生成スクリプトの出力

一括生成スクリプトを実行すると、以下のファイルが生成されます：

```
output/
├── markers/
│   ├── base/
│   │   ├── marker_0.png
│   │   ├── marker_1.png
│   │   └── ...
│   └── slider/
│       ├── marker_50.png
│       ├── marker_51.png
│       └── ...
├── boards/
│   ├── base_board.png
│   └── slider_board.png
└── configs/
    ├── base_board_config.json
    └── slider_board_config.json
```

## トピック

### 入力トピック

| トピック名 | メッセージ型 | 説明 |
| :---- | :---- | :---- |
| `/camera/image_raw` | sensor_msgs/msg/Image | カメラからの生画像 |
| `/camera/camera_info` | sensor_msgs/msg/CameraInfo | カメラの内部パラメータと歪み係数 |

### 出力トピック

| トピック名 | メッセージ型 | 説明 |
| :---- | :---- | :---- |
| `~/output/slider_pose` | geometry_msgs/msg/PoseStamped | 推定されたスライダのPose |
| `~/output/debug_image` | sensor_msgs/msg/Image | 検出したマーカーや座標軸が描画されたデバッグ用画像 |
| `/tf` | tf2_msgs/msg/TFMessage | ワールド座標系からスライダ座標系への座標変換情報 |

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :---- | :---- | :---- | :---- |
| `marker_dictionary` | string | `DICT_4X4_100` | 使用するArUcoマーカーの辞書名 |
| `base_board_config` | string | `config/base_board_config.json` | ベース側マーカー群の定義ファイルへのパス |
| `slider_board_config` | string | `config/slider_board_config.json` | スライダ側マーカー群の定義ファイルへのパス |
| `detector_params_file` | string | `config/detector_params.yaml` | ArUco検出器のパラメータファイルへのパス |
| `world_frame_id` | string | `world` | ワールド座標系のフレーム名 |
| `slider_frame_id` | string | `slider_base` | スライダ座標系のフレーム名 |
| `publish_tf` | bool | `true` | `/tfトピックをパブリッシュするかどうかのフラグ |
| `show_debug_image` | bool | `true` | デバッグ用画像をパブリッシュするかどうかのフラグ |

## 可視化

### RVizでの可視化

1. RVizを起動：
```bash
ros2 run rviz2 rviz2
```

2. 以下の設定を追加：
   - **TF**: ワールド座標系とスライダ座標系の変換を表示
   - **Image**: `/aruco_slider_estimator/output/debug_image`トピックを表示

### デバッグ画像の確認

```bash
ros2 run rqt_image_view rqt_image_view
```

## トラブルシューティング

### マーカーが検出されない

1. カメラキャリブレーションが正しく行われているか確認
2. マーカーのサイズと設定ファイルの`marker_size_mm`が一致しているか確認
3. マーカーのIDが設定ファイルと一致しているか確認
4. 照明条件を改善

### Pose推定が不安定

1. より多くのマーカーを配置
2. マーカーの配置を最適化（直線的でない配置を試す）
3. 検出器パラメータを調整

### 座標系が正しくない

1. マーカーの物理配置と設定ファイルの座標が一致しているか確認
2. ワールド座標系とスライダ座標系の定義を確認

## ライセンス

MIT License

## 貢献

バグ報告や機能要望は、GitHubのIssueでお知らせください。 
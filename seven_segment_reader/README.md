# 7セグメントディスプレイ読み取りROS2パッケージ

このパッケージは、IPカメラから取得した画像を使用して7セグメントディスプレイを読み取り、数値化してROS2トピックでパブリッシュするパッケージです。

## 機能

- IPカメラサーバーからの画像取得（タイムスタンプ付き）
- Roboflowを使用した7セグメントディスプレイの検出・認識
- 複数領域の同時監視
- ROS2トピックでの結果パブリッシュ
- 遅延補正機能

## 前提条件

- ROS2 Humble
- Python 3.8以上

## セットアップ

### 1. 仮想環境の設定

```bash
cd seven_segment_reader
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 2. パッケージのビルド

```bash
cd ../../../
colcon build --packages-select seven_segment_reader
source install/setup.bash
```

## 使用方法

### 1. IPカメラサーバーの起動

Windows側でIPカメラサーバーを起動：

```bash
python webcam_ip.py --host 127.0.0.1 --port 5000
```

### 2. ROS2ノードの起動

#### 方法1: 起動スクリプトを使用

```bash
cd seven_segment_reader
./run_node.sh
```

#### 方法2: 手動でlaunchファイルを実行

```bash
# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をソース
source /opt/ros/humble/setup.bash
source ../../../install/setup.bash

# ノードを起動
ros2 launch seven_segment_reader seven_segment_reader.launch.py \
  server_url:=http://127.0.0.1:5000 \
  processing_interval:=0.1
```

### 3. パラメータの設定

launchファイルで以下のパラメータを設定できます：

- `server_url`: IPカメラサーバーのURL（デフォルト: http://127.0.0.1:5000）
- `model_name`: Roboflowモデル名（デフォルト: 7-segment-display-gxhnj）
- `model_version`: Roboflowモデルバージョン（デフォルト: 2）
- `confidence_threshold`: 信頼度閾値（デフォルト: 0.5）
- `iou_threshold`: IoU閾値（デフォルト: 0.5）
- `overlap_threshold`: 重複検出の閾値（デフォルト: 0.7）
- `processing_interval`: 処理間隔（秒）（デフォルト: 0.1）

### 4. DOI領域の設定

#### 方法1: DOI領域選択ツールを使用（推奨）

```bash
cd seven_segment_reader
./select_regions.sh
```

または

```bash
# 仮想環境をアクティベート
source venv/bin/activate

# DOI領域を選択
python scripts/select_doi_regions.py

# 既存の設定を読み込んで編集
python scripts/select_doi_regions.py --load-existing

# 別のサーバーを使用
python scripts/select_doi_regions.py --server-url http://192.168.1.100:5000
```

#### 方法2: 手動で設定ファイルを編集

`config/detector_params.yaml`でDOI領域を設定：

```yaml
doi_regions:
  - [100, 100, 300, 200]  # 領域1: (100,100)から(300,200)
  - [400, 100, 600, 200]  # 領域2: (400,100)から(600,200)
  # 必要に応じて追加の領域を設定
```

## パブリッシュされるトピック

- `/seven_segment/image` (sensor_msgs/Image): 検出結果が描画された画像
- `/seven_segment/detection` (std_msgs/String): 検出結果のJSON文字列
- `/seven_segment/values` (std_msgs/Float64MultiArray): 数値化された検出結果
- `/seven_segment/timestamp` (std_msgs/String): サーバータイムスタンプ

## 検出結果の形式

検出結果は以下の形式で出力されます：

```json
{
  "detections": ["1.47", "14.83"],
  "frame_count": 1234,
  "server_timestamp": "2024-01-01 12:00:00:123456",
  "client_timestamp": "2024-01-01 12:00:00:123",
  "delay_ms": 5.2
}
```

## トラブルシューティング

### 1. モデルが読み込まれない

- インターネット接続を確認
- モデル名とバージョンが正しいか確認

### 2. 画像が取得できない

- IPカメラサーバーが起動しているか確認
- サーバーURLが正しいか確認
- ファイアウォール設定を確認

### 3. 検出精度が低い

- 信頼度閾値を調整
- DOI領域の設定を確認
- カメラの解像度とフレームレートを調整

## ファイル構成

```
seven_segment_reader/
├── package.xml
├── setup.py
├── requirements.txt
├── run_node.sh
├── select_regions.sh
├── seven_segment_reader/
│   ├── __init__.py
│   └── seven_segment_reader_node.py
├── launch/
│   └── seven_segment_reader.launch.py
├── config/
│   └── detector_params.yaml
├── scripts/
│   ├── webcam_ip.py
│   ├── webcam_client.py
│   ├── video_infer.py
│   └── select_doi_regions.py
└── venv/
    └── (仮想環境)
```

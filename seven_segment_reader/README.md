# 7セグメントディスプレイ読み取りROS2パッケージ

このパッケージは、IPカメラから取得した画像を使用して7セグメントディスプレイを読み取り、数値化してROS2トピックでパブリッシュするパッケージです。

## 機能

- IPカメラサーバーからの画像取得（タイムスタンプ付き）
- Roboflowを使用した7セグメントディスプレイの検出・認識
- 複数領域の同時監視
- ROS2トピックでの結果パブリッシュ
- 遅延補正機能

## アーキテクチャ

```
カメラ → ip_cam_cropping.py → DOIクロッピング → 回転 → 後加工 → 複数画像送信 → seven_segment_reader_node.py → AI判定
```

**アーキテクチャの利点:**
- ネットワーク帯域の大幅削減（全画像 → 処理済みDOI画像のみ）
- 処理効率の向上（事前クロッピング・回転・後加工）
- AI判定精度の向上（正方形画像、適切なサイズ）
- 複数DOIの同時処理

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

### 1. DOI設定の作成

初回起動時またはDOI領域を変更する場合：

```bash
cd seven_segment_reader
source venv/bin/activate
python scripts/ip_cam_cropping.py --create-config
```

### 2. IPカメラクロッピングサーバーの起動

**重要**: WSL2ではIPカメラサーバーを手動で起動する必要があります。

```bash
cd seven_segment_reader
source venv/bin/activate
python scripts/ip_cam_cropping.py
```

### 3. ROS2ノードの起動

別のターミナルでROS2ノードを起動：

```bash
cd seven_segment_reader
./run_node.sh
```

または手動でlaunchファイルを実行：

```bash
# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をソース
source /opt/ros/humble/setup.bash
source ../../../install/setup.bash

# システムを起動
ros2 launch seven_segment_reader seven_segment_reader.launch.py
```

### 4. パラメータの設定

launchファイルで以下のパラメータを設定できます：

- `server_url`: IPカメラクロッピングサーバーのURL（デフォルト: http://127.0.0.1:5000）
- `model_name`: Roboflowモデル名（デフォルト: 7-segment-display-gxhnj）
- `model_version`: Roboflowモデルバージョン（デフォルト: 2）
- `api_key`: Roboflow APIキー
- `confidence_threshold`: 信頼度閾値（デフォルト: 0.5）
- `iou_threshold`: IoU閾値（デフォルト: 0.5）
- `overlap_threshold`: 重複検出の閾値（デフォルト: 0.7）
- `processing_interval`: 処理間隔（秒）（デフォルト: 0.1）
- `doi_count`: DOI領域数（デフォルト: 1）

## DOI設定ファイル

`config/doi_config.json`でDOI領域を設定します：

```json
{
  "doi_regions": [
    {
      "name": "region_1",
      "top_left": [237, 25],
      "bottom_right": [544, 342],
      "rotation_mode": "auto",
      "rotation_angle": 0,
      "output_size": 224
    }
  ],
  "server_settings": {
    "host": "127.0.0.1",
    "port": 5000
  }
}
```

### 回転ルール

- `top_left`が左上、`bottom_right`が右下: 正常、回転なし（0度）
- `top_left`が右下、`bottom_right`が左上: 180度回転
- `top_left`が左下、`bottom_right`が右上: 90度回転
- `top_left`が右上、`bottom_right`が左下: 270度回転

## パブリッシュされるトピック

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
  "delay_ms": 5.2,
  "doi_count": 2
}
```

## トラブルシューティング

### 1. モデルが読み込まれない

- インターネット接続を確認
- モデル名とバージョンが正しいか確認
- APIキーが正しく設定されているか確認

### 2. 画像が取得できない

- IPカメラクロッピングサーバーが起動しているか確認
- サーバーURLが正しいか確認
- ファイアウォール設定を確認

### 3. DOI設定エラー

- `config/doi_config.json`が正しく作成されているか確認
- DOI領域数と`doi_count`パラメータが一致しているか確認

### 4. 検出精度が低い

- 信頼度閾値を調整
- DOI領域の設定を確認
- カメラの解像度とフレームレートを調整

## ファイル構成

```
seven_segment_reader/
├── package.xml
├── setup.py
├── requirements.txt
├── run_node.sh                    # 起動スクリプト
├── seven_segment_reader/
│   ├── __init__.py
│   └── seven_segment_reader_node.py
├── launch/
│   └── seven_segment_reader.launch.py  # launchファイル
├── config/
│   ├── detector_params.yaml
│   └── doi_config.json           # DOI設定
├── scripts/
│   ├── ip_cam_cropping.py        # IPカメラクロッピングサーバー
│   ├── webcam_client.py
│   └── video_infer.py
└── venv/
    └── (仮想環境)
```

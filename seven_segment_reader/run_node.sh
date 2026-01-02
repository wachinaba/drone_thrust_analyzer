#!/bin/bash

# 7セグメントディスプレイ読み取りROS2ノード起動スクリプト

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 仮想環境をアクティベート
echo "仮想環境をアクティベート中..."
source venv/bin/activate

# ROS2環境をソース
echo "ROS2環境をソース中..."
source /opt/ros/humble/setup.bash

# ワークスペースをビルド
echo "ワークスペースをビルド中..."
cd ../../../
colcon build --packages-select seven_segment_reader

# ビルドしたパッケージをソース
echo "ビルドしたパッケージをソース中..."
source install/setup.bash

# パッケージディレクトリに戻る
cd src/drone_thrust_analyzer/seven_segment_reader

echo "7セグメントディスプレイ読み取りシステムを起動中..."
echo ""
echo "パラメータ例:"
echo "  ros2 launch seven_segment_reader seven_segment_reader.launch.py \\"
echo "    server_url:=http://127.0.0.1:5000 \\"
echo "    api_key:=YOUR_ROBOFLOW_API_KEY \\"
echo "    doi_count:=2 \\"
echo "    processing_interval:=0.1"
echo ""

# launchファイルを実行
ros2 launch seven_segment_reader seven_segment_reader.launch.py

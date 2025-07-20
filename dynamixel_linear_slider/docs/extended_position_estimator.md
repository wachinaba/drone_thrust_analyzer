# ExtendedPositionEstimatorNode

DynamixelHandlerのExtendedPositionをベースにした推定位置ノードです。ARマーカーによる原点リセット機能を備えています。

## 機能概要

### 主要機能
- **DynamixelHandler状態受信**: `/dynamixel/states`からExtendedPosition（`present.position_deg`）を取得
- **ARマーカー原点リセット**: `/ar_marker_position`を使用した原点リセット
- **速度監視**: モータ速度が0でない場合は原点リセットを実行しない
- **平均化処理**: 1秒間のARマーカーポジション平均値で原点リセット
- **推定位置発行**: 原点リセット後に`/estimated_position`を発行

### 原点リセット条件
1. **モータ速度**: 速度が閾値以下（デフォルト: 0.01 deg/s）
2. **ARマーカーデータ**: 最低5個のデータポイントが必要
3. **データ安定性**: 標準偏差が閾値以下（デフォルト: 0.01 m）
4. **時間範囲**: 指定時間（デフォルト: 1.0秒）のデータを使用

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `motor_id` | 1 | 対象モータのID |
| `rack_pitch` | 0.106214 | ラックピッチ (m/rev) |
| `gear_ratio` | 1.0 | ギア比 |
| `control_frequency` | 100.0 | 制御周波数 (Hz) |
| `velocity_zero_threshold` | 0.01 | 速度ゼロ判定閾値 (deg/s) |
| `ar_marker_average_duration` | 1.0 | ARマーカー平均化時間 (秒) |
| `ar_marker_std_threshold` | 0.01 | ARマーカー標準偏差閾値 (m) |

## トピック

### サブスクライブ
- `/dynamixel/states` (dynamixel_handler_msgs/DxlStates): DynamixelHandlerの状態
- `/ar_marker_position` (std_msgs/Float64): ARマーカー位置

### パブリッシュ
- `/estimated_position` (std_msgs/Float64): 推定位置（原点リセット後のみ）
- `/origin_offset_deg` (std_msgs/Float64): 原点オフセット（デバッグ用）

## 動作フロー

### 1. 初期化
- DynamixelHandlerの状態を受信開始
- ARマーカーポジションを受信開始
- 原点未設定状態で開始

### 2. 原点リセット判定
```python
# 条件チェック
if (velocity < threshold and 
    ar_marker_data_sufficient and 
    std_deviation < threshold):
    perform_origin_reset()
```

### 3. 原点リセット実行
```python
# 現在位置をメートルに変換
current_position_m = position_deg * rack_pitch / 360.0 * gear_ratio

# 原点オフセット計算
origin_offset_deg = (current_position_m - ar_marker_mean) * 360.0 / rack_pitch / gear_ratio
```

### 4. 推定位置計算
```python
# 原点オフセットを適用
adjusted_position_deg = current_position_deg - origin_offset_deg

# メートルに変換
estimated_position_m = adjusted_position_deg * rack_pitch / 360.0 * gear_ratio
```

## 使用例

### 基本的な使用
```bash
# テスト用launchファイルで実行
ros2 launch dynamixel_linear_slider extended_position_estimator_test.launch.py
```

### カスタムパラメータ
```bash
# パラメータをカスタマイズして実行
ros2 launch dynamixel_linear_slider extended_position_estimator_test.launch.py \
    velocity_zero_threshold:=0.005 \
    ar_marker_average_duration:=2.0 \
    ar_marker_std_threshold:=0.005
```

### トピック監視
```bash
# 推定位置を監視
ros2 topic echo /estimated_position

# 原点オフセットを監視
ros2 topic echo /origin_offset_deg

# ARマーカー位置を監視
ros2 topic echo /ar_marker_position
```

## ログメッセージ

### 初期化時
```
[INFO] [extended_position_estimator_node]: ExtendedPositionEstimatorNode initialized
[INFO] [extended_position_estimator_node]: Motor ID: 1
[INFO] [extended_position_estimator_node]: Rack pitch: 0.106214 m/rev
[INFO] [extended_position_estimator_node]: Gear ratio: 1.0
[INFO] [extended_position_estimator_node]: Velocity zero threshold: 0.01 deg/s
[INFO] [extended_position_estimator_node]: AR marker average duration: 1.0 s
[INFO] [extended_position_estimator_node]: AR marker std threshold: 0.01 m
```

### 原点リセット実行時
```
[INFO] [extended_position_estimator_node]: Origin reset performed: AR marker mean=0.1234 m, current position=0.1234 m, origin offset=0.00 deg
```

### デバッグ情報
```
[DEBUG] [extended_position_estimator_node]: Motor 1: position_deg=180.00, velocity_deg_s=0.00
[DEBUG] [extended_position_estimator_node]: AR marker position: 0.1234 m
[DEBUG] [extended_position_estimator_node]: Estimated position: 0.1234 m, Origin set: True
[DEBUG] [extended_position_estimator_node]: Waiting for origin reset...
```

## 注意事項

### 原点リセットのタイミング
- モータが完全に停止している必要があります
- ARマーカーデータが安定している必要があります
- 十分なデータポイント（最低5個）が必要です

### 推定精度
- ラックピッチとギア比の設定が重要です
- ARマーカーの検出精度に依存します
- 原点リセットの精度が推定精度を決定します

### トラブルシューティング
- **原点リセットが実行されない**: モータ速度、ARマーカーデータ、標準偏差を確認
- **推定位置が不正確**: ラックピッチ、ギア比、原点オフセットを確認
- **ARマーカーデータが不安定**: カメラ設定、照明条件、マーカー配置を確認 
# スライダ位置真値機能

XC330-T288-Tシミュレーターでは、エンコーダー値からスライダの物理的な位置（mm単位）を計算してパブリッシュする機能を提供します。

## 機能概要

### スライダ位置計算
- エンコーダー値（0-4095）からスライダ位置（mm）への変換
- 初期位置からの相対位置計算
- スライダ全長範囲内での制限

### パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `slider_total_length` | 100.0 | スライダ全長 (mm) |
| `slider_initial_position` | 50.0 | スライダ初期位置 (mm) |
| `encoder_to_slider_ratio` | 0.01 | エンコーダー値とスライダ位置の変換比 (mm/encoder_unit) |
| `slider_position_topic` | 'slider_position' | スライダ位置トピック名 |

## 計算方法

### エンコーダー値からスライダ位置への変換

1. **エンコーダー値の正規化**
   ```
   encoder_change = current_position - (encoder_resolution / 2)
   ```

2. **スライダ位置の計算**
   ```
   slider_change = encoder_change * encoder_to_slider_ratio
   slider_position = slider_initial_position + slider_change
   ```

3. **範囲制限**
   ```
   slider_position = max(0.0, min(slider_total_length, slider_position))
   ```

## 使用例

### 基本的な使用
```bash
# デフォルト設定でスライダ位置をパブリッシュ
ros2 launch dynamixel_linear_slider dynamixel_simulator.launch.py
```

### カスタムスライダ設定
```bash
# 200mmスライダ、初期位置100mmで使用
ros2 launch dynamixel_linear_slider dynamixel_simulator.launch.py \
    slider_total_length:=200.0 \
    slider_initial_position:=100.0 \
    encoder_to_slider_ratio:=0.02
```

### テスト用launchファイル
```bash
# スライダ位置を含むテストシミュレーション
ros2 launch dynamixel_linear_slider simulator_test.launch.py
```

## トピック

### 発行トピック
- `/slider_position` (Float64): スライダ位置真値 (mm)
- `/motor_status` (String): スライダ位置情報を含むステータス

### 購読トピック
- `/velocity_command` (Float64): 目標速度指令

## 設定例

### 実際のスライダシステム用設定
```yaml
# 300mmスライダ、中心初期位置
slider_total_length: 300.0
slider_initial_position: 150.0
encoder_to_slider_ratio: 0.05  # エンコーダー1単位 = 0.05mm

# 100mmスライダ、端部初期位置
slider_total_length: 100.0
slider_initial_position: 0.0
encoder_to_slider_ratio: 0.01  # エンコーダー1単位 = 0.01mm
```

## 監視方法

### スライダ位置の監視
```bash
# スライダ位置トピックの監視
ros2 topic echo /slider_position

# モーターステータスの監視（スライダ位置含む）
ros2 topic echo /motor_status
```

### 可視化
```bash
# rqt_plotでスライダ位置を可視化
ros2 run rqt_plot rqt_plot /slider_position/data
```

## 注意事項

- エンコーダー値は0-4095の範囲で循環します
- スライダ位置は全長範囲内に制限されます
- 初期位置はスライダ全長の範囲内に設定してください
- 変換比は実際のハードウェアに合わせて調整してください 
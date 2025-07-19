# 位置コマンド送信と待機ノード

指定された位置に移動し、移動完了を待って終了するROS2ノードです。

## 機能概要

### 主要機能
- パラメータで指定された位置に移動
- `target_position`トピックの監視
- 同じ位置へのリクエストの無視
- 移動状態の遷移確認
- 移動完了（停止）を待って自動終了
- タイムアウト機能

### 移動完了の判定条件
1. **移動開始確認**: 移動状態が`true`に遷移したことを確認
2. **移動状態**: `movement_status`が`false`（停止中）

**注意**: 
- `movement_status`は収束判定も含んでいるため、位置偏差の重複チェックは行いません
- 移動開始を確認してから終了判定を行うため、即座のノード終了を防ぎます
- 同じ位置へのリクエストは即座に終了します

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `target_position` | 0.05 | 目標位置 (m) |
| `wait_timeout` | 30.0 | 待機タイムアウト (秒) |

## 使用方法

### 基本的な使用
```bash
# デフォルト設定で0.05mに移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py
```

### カスタム位置での使用
```bash
# 0.1mに移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py \
    target_position:=0.1
```

### 短いタイムアウトでの使用
```bash
# 10秒でタイムアウト
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py \
    target_position:=0.05 \
    wait_timeout:=10.0
```

## 動作フロー

1. **初期化**: パラメータの読み込みとノードの初期化
2. **位置コマンド送信**: 目標位置を`target_position_command`トピックに送信
3. **移動監視**: `movement_status`と`target_position`を継続監視
4. **移動開始確認**: 移動状態が`true`に遷移したことを確認
5. **完了判定**: 移動状態が`false`（停止中）になった時点で完了
6. **終了**: 移動完了またはタイムアウトでノード終了

## トピック

### パブリッシュ
- `target_position_command` (std_msgs/Float64): 目標位置コマンド

### サブスクライブ
- `target_position` (std_msgs/Float64): 目標位置（監視用）
- `movement_status` (std_msgs/Bool): 移動状態（収束判定含む）
- `estimated_position` (std_msgs/Float64): 推定位置（デバッグ用）

## ログメッセージ

### 初期化時
```
[INFO] [position_command_and_wait_node]: PositionCommandAndWaitNode initialized
[INFO] [position_command_and_wait_node]: Target position: 0.050000 m
[INFO] [position_command_and_wait_node]: Wait timeout: 30.000000 s
```

### 位置コマンド送信時
```
[INFO] [position_command_and_wait_node]: Position command sent: 0.050000 m
```

### 同じ位置リクエスト時
```
[INFO] [position_command_and_wait_node]: Same position requested: 0.050000 m, exiting immediately
```

### 移動監視中（1秒ごと）
```
[INFO] [position_command_and_wait_node]: Time: 1.0s, Target: 0.0500m, Estimated: 0.0450m, Error: 0.005000m, Status: Moving, Movement: Started
[INFO] [position_command_and_wait_node]: Time: 2.0s, Target: 0.0500m, Estimated: 0.0495m, Error: 0.000500m, Status: Stopped, Movement: Started
```

### 移動開始時
```
[INFO] [position_command_and_wait_node]: Movement started
```

### 移動停止時
```
[INFO] [position_command_and_wait_node]: Movement stopped
```

### 移動完了時
```
[INFO] [position_command_and_wait_node]: Movement completed successfully
```

### タイムアウト時
```
[WARN] [position_command_and_wait_node]: Timeout reached: 30.0s
[ERROR] [position_command_and_wait_node]: Node terminating due to timeout
```

## 移動完了判定ロジック

```python
# 移動開始を確認していない場合は完了判定しない
if not self.has_movement_started:
    return False

# 移動状態がFalse（停止中）の場合、完了と判定
if not self.is_moving:
    return True  # 移動完了
```

## 同じ位置リクエストの判定ロジック

```python
# 同じ位置へのリクエストの場合、即座に終了
if abs(received_position - self.target_position) < 0.001:  # 1mm以下の差は同じとみなす
    return True  # 即座に終了
```

## 注意事項

- **タイムアウト**: 指定時間内に移動が完了しない場合はタイムアウトで終了
- **移動開始確認**: 移動状態が`true`に遷移したことを確認してから終了判定を行う
- **同じ位置リクエスト**: 同じ位置へのリクエストは即座に終了
- **収束判定の活用**: `movement_status`は収束判定も含むため、位置偏差の重複チェックは不要
- **依存関係**: `movement_status`と`target_position`トピックが必要
- **シミュレーター対応**: シミュレーター環境でも動作

## トラブルシューティング

### 移動が完了しない場合
1. `movement_status`トピックが正しくパブリッシュされているか確認
2. 移動開始が正しく検出されているか確認
3. 制御システムの動作を確認

### タイムアウトが発生する場合
1. 目標位置が到達可能な範囲内か確認
2. `wait_timeout`を長くする
3. 制御システムの動作を確認

### 即座に終了する場合
1. 同じ位置へのリクエストが発生していないか確認
2. `target_position`トピックの値を確認

### 移動完了が早すぎる場合
1. 位置制御のパラメータを調整
2. 収束判定のパラメータを調整

## 使用例

### 複数位置への順次移動
```bash
# 1つ目の位置に移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py target_position:=0.02

# 2つ目の位置に移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py target_position:=0.05

# 3つ目の位置に移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py target_position:=0.08
```

### 高速移動
```bash
# 移動開始確認付きの高速移動
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py \
    target_position:=0.05 \
    wait_timeout:=10.0
```

### 同じ位置でのテスト
```bash
# 現在位置と同じ位置へのリクエスト（即座に終了）
ros2 launch dynamixel_linear_slider position_command_and_wait_test.launch.py \
    target_position:=0.05
``` 
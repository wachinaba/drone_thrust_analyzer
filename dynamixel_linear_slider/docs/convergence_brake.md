# 位置収束ブレーキ機能

位置制御において、目標位置に収束した際に自動的にブレーキをかける機能を提供します。

## 機能概要

### 収束判定条件
1. **位置偏差**: `calculate_position_error()`の結果が閾値以下
2. **速度指令**: velocityコマンドがゼロ（停止意図がある）

### ブレーキ処理
- 収束判定時にvelocity_controlで速度0を送信
- モーターの回転を停止し、位置を保持

### 移動状態パブリッシュ機能
- 移動中かどうかの状態を`movement_status`トピックでパブリッシュ
- 位置偏差と速度指令の両方を考慮して移動状態を判定
- bool型で常にパブリッシュ（true: 移動中, false: 停止中）

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| `convergence_threshold` | 0.002 | 位置偏差の閾値 (m) |
| `convergence_duration` | 1.0 | 収束判定に必要な持続時間 (秒) |
| `velocity_zero_threshold` | 0.01 | velocityコマンドのゼロ判定閾値 (m/s) |
| `enable_convergence_brake` | true | 収束ブレーキの有効/無効 |
| `movement_velocity_threshold` | 0.005 | 移動判定の速度閾値 (m/s) |
| `movement_position_threshold` | 0.001 | 移動判定の位置偏差閾値 (m) |

## 使用方法

### 基本的な使用
```bash
# デフォルト設定で収束ブレーキ機能を有効化
ros2 launch dynamixel_linear_slider convergence_brake_test.launch.py
```

### カスタムパラメータでの使用
```bash
# より厳密な収束判定（偏差0.001m、持続時間2秒）
ros2 launch dynamixel_linear_slider convergence_brake_test.launch.py \
    convergence_threshold:=0.001 \
    convergence_duration:=2.0 \
    velocity_zero_threshold:=0.005 \
    movement_velocity_threshold:=0.003 \
    movement_position_threshold:=0.0005
```

### 収束ブレーキ機能の無効化
```bash
# 収束ブレーキ機能を無効化
ros2 launch dynamixel_linear_slider convergence_brake_test.launch.py \
    enable_convergence_brake:=false
```

## 動作フロー

1. **通常制御**: 位置制御とvelocity制御を並行実行
2. **移動状態監視**: 位置偏差とvelocityコマンドを監視して移動状態を判定
3. **移動状態パブリッシュ**: 常に`movement_status`トピックでパブリッシュ
4. **収束監視**: 位置偏差とvelocityコマンドを継続監視
5. **収束検出**: 両条件が満たされた時点で収束開始時刻を記録
6. **持続確認**: 指定時間（デフォルト1秒）条件が継続することを確認
7. **ブレーキ実行**: velocity_controlで速度0を送信
8. **ログ出力**: 収束とブレーキ実行をログに記録

## トピック

### パブリッシュ
- `movement_status` (std_msgs/Bool): 移動状態
  - `true`: 移動中
  - `false`: 停止中

### サブスクライブ
- `velocity_command` (std_msgs/Float64): 速度指令値
- `estimated_position` (std_msgs/Float64): 推定位置
- `target_position` (std_msgs/Float64): 目標位置
- `/dynamixel/states` (dynamixel_handler_msgs/DxlStates): Dynamixel状態

## ログメッセージ

### 移動状態（デバッグレベル）
```
[DEBUG] [dynamixel_handler_position_controller_node]: Movement status: MOVING
[DEBUG] [dynamixel_handler_position_controller_node]: Movement status: STOPPED
```

### 収束検出時
```
[INFO] [dynamixel_handler_position_controller_node]: Convergence detected: position_error=0.001500m, velocity=0.005000m/s
```

### 収束完了時
```
[INFO] [dynamixel_handler_position_controller_node]: Position converged for 1.0s: position_error=0.001500m, velocity=0.005000m/s
```

### ブレーキ実行時
```
[INFO] [dynamixel_handler_position_controller_node]: Brake applied due to position convergence
[INFO] [dynamixel_handler_position_controller_node]: Brake command sent: velocity=0.0 deg/s
```

## 移動状態判定ロジック

移動状態は以下の条件で判定されます：

```python
is_moving = (position_error > movement_position_threshold or 
            velocity_abs > movement_velocity_threshold)
```

- **位置偏差**: 目標位置と推定位置の差が閾値以上
- **速度指令**: velocityコマンドの絶対値が閾値以上
- **どちらかの条件を満たす場合に移動中と判定**

## 注意事項

- **一度ブレーキがかかると、ノード再起動まで継続**: ブレーキ状態はリセットされません
- **velocityコマンドの監視**: `velocity_command`トピックからのデータが必要です
- **パラメータ調整**: システム特性に応じて閾値を調整してください
- **シミュレーター対応**: シミュレーター環境でも動作します
- **常時パブリッシュ**: 移動状態は常にパブリッシュされます（制御周波数に依存）

## トラブルシューティング

### ブレーキがかからない場合
1. `velocity_command`トピックが正しくパブリッシュされているか確認
2. 位置偏差が閾値を下回っているか確認
3. velocityコマンドがゼロに近い値になっているか確認
4. パラメータ値を調整

### 移動状態が正しく判定されない場合
1. `movement_velocity_threshold`と`movement_position_threshold`を調整
2. 位置偏差とvelocityコマンドの値を確認
3. `movement_status`トピックのメッセージを確認

### ブレーキが早すぎる場合
1. `convergence_threshold`を小さくする
2. `convergence_duration`を長くする
3. `velocity_zero_threshold`を小さくする

### ブレーキが遅すぎる場合
1. `convergence_threshold`を大きくする
2. `convergence_duration`を短くする
3. `velocity_zero_threshold`を大きくする 
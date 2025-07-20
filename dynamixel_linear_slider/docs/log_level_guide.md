# ROS 2 ログレベル指定ガイド

## 1. launchファイルでの指定方法

### 個別ノードのログレベル指定
```python
Node(
    package='dynamixel_linear_slider',
    executable='extended_position_estimator_node',
    name='extended_position_estimator_node',
    arguments=['--ros-args', '--log-level', 'extended_position_estimator_node:=DEBUG']
)
```

### 複数ノードのログレベル指定
```python
Node(
    package='dynamixel_linear_slider',
    executable='extended_position_estimator_node',
    name='extended_position_estimator_node',
    arguments=[
        '--ros-args', 
        '--log-level', 'extended_position_estimator_node:=DEBUG',
        '--log-level', 'rcl:=WARN'
    ]
)
```

## 2. コマンドラインでの指定方法

### launchファイル実行時の指定
```bash
# 特定のノードのログレベルを指定
ros2 launch dynamixel_linear_slider integrated_system.launch.py \
    --ros-args --log-level extended_position_estimator_node:=DEBUG

# 複数ノードのログレベルを指定
ros2 launch dynamixel_linear_slider integrated_system.launch.py \
    --ros-args \
    --log-level extended_position_estimator_node:=DEBUG \
    --log-level estimator_node:=INFO \
    --log-level rcl:=WARN
```

### 個別ノード実行時の指定
```bash
# 直接ノードを実行する場合
ros2 run dynamixel_linear_slider extended_position_estimator_node \
    --ros-args --log-level DEBUG

# 特定のロガー名を指定
ros2 run dynamixel_linear_slider extended_position_estimator_node \
    --ros-args --log-level extended_position_estimator_node:=DEBUG
```

## 3. 利用可能なログレベル

| レベル | 説明 | 使用例 |
|--------|------|--------|
| `DEBUG` | デバッグ情報 | 詳細な内部状態、変数値 |
| `INFO` | 一般的な情報 | 初期化完了、状態変化 |
| `WARN` | 警告 | 非致命的な問題 |
| `ERROR` | エラー | 致命的な問題 |
| `FATAL` | 致命的エラー | システム停止が必要 |

## 4. よく使用されるロガー名

### ノード固有のロガー
- `extended_position_estimator_node`
- `estimator_node`
- `dynamixel_handler_position_controller_node`
- `trajectory_generator_node`

### システムロガー
- `rcl`: ROS 2 クライアントライブラリ
- `rclcpp`: C++ クライアントライブラリ
- `rclpy`: Python クライアントライブラリ
- `rcutils`: ユーティリティライブラリ

## 5. 実用的な設定例

### 開発時の設定
```bash
# デバッグ情報を多く表示
ros2 launch dynamixel_linear_slider integrated_system.launch.py \
    --ros-args \
    --log-level extended_position_estimator_node:=DEBUG \
    --log-level estimator_node:=DEBUG \
    --log-level rcl:=WARN
```

### 本番環境での設定
```bash
# 重要な情報のみ表示
ros2 launch dynamixel_linear_slider integrated_system.launch.py \
    --ros-args \
    --log-level extended_position_estimator_node:=INFO \
    --log-level estimator_node:=INFO \
    --log-level rcl:=ERROR
```

### エラー調査時の設定
```bash
# エラーと警告のみ表示
ros2 launch dynamixel_linear_slider integrated_system.launch.py \
    --ros-args \
    --log-level extended_position_estimator_node:=WARN \
    --log-level estimator_node:=WARN \
    --log-level rcl:=ERROR
```

## 6. ログレベルの動的変更

実行中のノードのログレベルを動的に変更することも可能です：

```bash
# ログレベルを動的に変更
ros2 service call /extended_position_estimator_node/set_logger_level \
    rcl_interfaces/srv/SetLoggerLevel \
    "{logger_name: 'extended_position_estimator_node', level: 'DEBUG'}"

# 現在のログレベルを確認
ros2 service call /extended_position_estimator_node/get_logger_level \
    rcl_interfaces/srv/GetLoggerLevel \
    "{logger_name: 'extended_position_estimator_node'}"
```

## 7. 注意事項

- ログレベルを`DEBUG`に設定すると、大量のログが出力される可能性があります
- 本番環境では`INFO`または`WARN`レベルを推奨します
- パフォーマンスに影響を与える可能性があるため、必要に応じて調整してください 
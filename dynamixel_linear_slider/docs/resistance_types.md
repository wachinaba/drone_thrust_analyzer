# Dynamixelシミュレーターの抵抗力タイプ

XC330-T288-Tシミュレーターでは、様々なタイプの抵抗力を設定できます。

## 抵抗力タイプ

### 1. `none` - 抵抗力なし
- 抵抗力が発生しない理想的なシミュレーション
- 目標速度に完全に追従

### 2. `viscous` - 粘性抵抗
- 速度に比例する抵抗力
- パラメータ: `viscous_coefficient` (N·m·s/rad)
- 高速時に大きな抵抗力が発生

### 3. `friction` - 摩擦抵抗
- 速度の符号に依存する定常的な摩擦
- パラメータ: `friction_coefficient` (N·m)
- 静止摩擦の閾値（0.01 rad/s）以下では抵抗力なし

### 4. `time_varying` - 時間変化抵抗
- 正弦波の時間変化 + 粘性抵抗
- パラメータ: 
  - `time_varying_amplitude` (N·m)
  - `time_varying_frequency` (Hz)
  - `viscous_coefficient` (N·m·s/rad)

### 5. `random` - ランダム抵抗
- ガウシアンノイズ + 粘性抵抗
- パラメータ:
  - `random_resistance_std` (N·m)
  - `viscous_coefficient` (N·m·s/rad)

### 6. `combined` - 複合抵抗
- 摩擦 + 粘性 + 時間変化の組み合わせ
- 最も現実的なシミュレーション
- パラメータ: すべての抵抗力パラメータ

## 使用例

### 基本的な粘性抵抗
```bash
ros2 launch dynamixel_linear_slider dynamixel_simulator.launch.py resistance_type:=viscous viscous_coefficient:=0.2
```

### 時間変化抵抗
```bash
ros2 launch dynamixel_linear_slider dynamixel_simulator.launch.py resistance_type:=time_varying time_varying_amplitude:=0.3 time_varying_frequency:=2.0
```

### 複合抵抗（最も現実的）
```bash
ros2 launch dynamixel_linear_slider dynamixel_simulator.launch.py resistance_type:=combined
```

## パラメータ推奨値

### XC330-T288-T用推奨設定
- `viscous_coefficient`: 0.1 - 0.3 N·m·s/rad
- `friction_coefficient`: 0.05 - 0.15 N·m
- `time_varying_amplitude`: 0.1 - 0.3 N·m
- `time_varying_frequency`: 0.5 - 2.0 Hz
- `random_resistance_std`: 0.05 - 0.15 N·m

## トピック

### 発行トピック
- `/resistance_torque` (Float64): 現在の抵抗力トルク
- `/joint_states` (JointState): effortフィールドに抵抗力トルクを含む
- `/motor_status` (String): 抵抗力情報を含むステータス

### 購読トピック
- `/velocity_command` (Float64): 目標速度指令 
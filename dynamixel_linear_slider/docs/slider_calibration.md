## リニアスライダ キャリブレーション手順

このドキュメントでは、`calibrated_slider_controller_node` を用いて  
**Dynamixel モータ角度 [deg] ↔ スライダ位置 [m] の非線形マッピング（区間線形）** を構築する手順をまとめます。

- 実測位置は **ARマーカー（PnP）から得られる `/ar_slider_position` [m]**
- モータ角度は **`/dynamixel/states` の `present.position_deg`**

を利用します。

---

## 前提条件

- 起動する launch:
  - `dynamixel_linear_slider/launch/integrated_system_calibrated.launch.py`
    - `dynamixel_handler`
    - `dynamixel_leadscrew_slider`（上下スライダ）
    - `dynamixel_linear_slider/ar_marker_web_client_node`
    - `dynamixel_linear_slider/calibrated_slider_controller_node`
    - 力センサノード など
- Windows 側で AR Web サーバ（`/detect` が `id + corners` を返す）が起動済み。
- `aruco_slider_estimator` の
  - `base_board_config.json`
  - `slider_board_config.json`
  - `emeet_c960.yaml`
  が正しく設定されており、PnP でスライダ位置が復元できること。

---

## ステップ 1: システム起動と AR 位置の確認

### 1.1 システム起動

```bash
cd ~/colcon_ws
. install/setup.bash
ros2 launch dynamixel_linear_slider integrated_system_calibrated.launch.py
```

### 1.2 AR 位置トピックの確認

```bash
ros2 topic echo /ar_slider_position
```

- スライダをゆっくり動かし、
  - 値が滑らかに変化すること
  - 向き（符号）やスケールが大きくおかしくないこと  
を確認します。

---

## ステップ 2: 既存キャリブレーションの確認・リセット

### 2.1 起動時ログ

`calibrated_slider_controller_node` 起動時に次のようなログが出ます。

- 既に YAML がある場合（正常ロード）:
  - **WARN**: `Loaded calibration: N points (file motor_id=..., rack_pitch=..., gear_ratio=...)`
- ファイル未存在など:
  - **INFO**: `Calibration file not found: ...`  
    → 初回キャリブレーション時はこれで正常。

### 2.2 既存キャリブレーションのクリア（必要な場合）

```bash
ros2 service call /calibrated_slider_controller_node/clear_calibration std_srvs/srv/Trigger "{}"
```

- すべての登録点が削除され、ゼロからキャリブレーションし直せます。

---

## ステップ 3: キャリブレーション点の取得

ここでは、「**モータ角度 θᵢ [deg] と AR 実測位置 xᵢ [m]**」を 1 点ずつ登録していきます。

### 3.0 （任意）手動操作のためのトルク ON/OFF

キャリブレーション時に「スライダを手で動かしたい」場合は、  
**必ずトルクを OFF にしてから** 行ってください（無理な力をかけるとギヤやモータを傷めます）。

`calibrated_slider_controller_node` は、対象モータ（`motor_id` パラメータ）に対する  
トルク ON/OFF を行うサービスを提供しています。

- トルク OFF（手で動かしたいとき）:

```bash
ros2 service call /calibrated_slider_controller_node/disable_output std_srvs/srv/Trigger "{}"
```

- トルク ON（キャリブレーション後に制御に戻したいとき）:

```bash
ros2 service call /calibrated_slider_controller_node/enable_output std_srvs/srv/Trigger "{}"
```

> 他のノード（例: `dynamixel_leadscrew_slider`）が同じモータ ID に対して  
> トルクONを出している場合は、その影響も考慮してください。

### 3.1 スライダを既知の物理位置に移動

- 手動、または既存ノードを使って、スライダを既知の位置に移動します。
  - 例: `position_command_and_wait_node` を使用

    ```bash
    ros2 run dynamixel_linear_slider position_command_and_wait_node \
      --ros-args -p target_position:=0.050
    ```

  - または `dynamixel_leadscrew_slider` の `/slider_bottom/move_mm` などを利用。

- リニアスケールや治具を用いて、物理的な基準点（0mm, 50mm, 100mm, ...）を決めておきます。

### 3.2 AR 位置の安定を確認

- 所望の位置に止めた状態で:

```bash
ros2 topic echo /ar_slider_position
```

- 数秒程度観察し、値がほぼ一定であることを確認します。  
  （現状は単発値を使用するため、「十分安定しているか」はオペレータが判断します）

### 3.3 キャリブレーション点の登録

- その状態で、次のサービスを呼びます。

```bash
ros2 service call /calibrated_slider_controller_node/add_calibration_point std_srvs/srv/Trigger "{}"
```

- 内部で以下のペアが追加されます。
  - θᵢ = `/dynamixel/states` の `present.position_deg`（`motor_id` で指定されたモータ）
  - xᵢ = その瞬間の `/ar_slider_position` [m]

- 成功ログ例:

```text
[INFO] Added calibration point: theta=1234.567 deg, x=0.050000 m
```

### 3.4 ストローク全域で複数点を取得

- 上記 3.1〜3.3 を繰り返し、**ストローク全域で 5〜10 点以上** のキャリブレーション点を取ります。
  - 例: 0.00 m, 0.05 m, 0.10 m, 0.15 m, 0.20 m, ...
  - 遊びや撓みが出やすい端部や中間領域は細かくサンプリングすると、精度が向上します。

---

## ステップ 4: キャリブレーションの確定と保存

### 4.1 点列の確定（マッピング有効化）

```bash
ros2 service call /calibrated_slider_controller_node/finalize_calibration std_srvs/srv/Trigger "{}"
```

- これにより:
  - θ 昇順で点がソートされ、
  - **ピースワイズ線形マッピング** が有効になります。

- 成功ログ例:

```text
[INFO] Calibration finalized with 8 points
```

### 4.2 YAML への保存（永続化）

```bash
ros2 service call /calibrated_slider_controller_node/save_calibration std_srvs/srv/Trigger "{}"
```

- デフォルト保存先:
  - `dynamixel_linear_slider/config/calibration.yaml`
- 次回起動時、`load_calibration_from_file` により自動で読み込まれ、  
  **WARN ログ** で内容が表示されます。

---

## ステップ 5: キャリブレーション結果の確認

### 5.1 目標位置への移動

- 例として 0.10 m へ移動し、完了を待ちます。

```bash
ros2 run dynamixel_linear_slider position_command_and_wait_node \
  --ros-args -p target_position:=0.10
```

### 5.2 推定位置と AR 位置の比較

- 別ターミナルで:

```bash
ros2 topic echo /estimated_position
ros2 topic echo /ar_slider_position
```

- 到達後に、
  - `estimated_position` と `ar_slider_position` の差が十分小さいか（例: 数 mm 程度）
  - ストロークの複数点で同様に誤差が小さいか
  を確認します。

### 5.3 追加調整

- 誤差が大きい領域があれば、その周辺でキャリブレーション点を追加し、
  - 再度 `finalize_calibration` → `save_calibration`
  を実行してマップを更新します。

---

## ステップ 6: 通常運用

- 一度キャリブレーション＆保存しておけば、以降は:

  1. `integrated_system_calibrated.launch.py` を起動
  2. `calibrated_slider_controller_node` が `config/calibration.yaml` を自動ロード
  3. 上位ノードから `target_position_command` [m] を publish

- これだけで、**AR基準で補正された非線形マッピングを使った位置制御** が利用可能になります。

---

## 参考: よく使うコマンド例

- 0.15 m へ移動して完了待ち:

```bash
ros2 run dynamixel_linear_slider position_command_and_wait_node \
  --ros-args -p target_position:=0.15
```

- キャリブレーション点リセット:

```bash
ros2 service call /calibrated_slider_controller_node/clear_calibration std_srvs/srv/Trigger "{}"
```

- キャリブレーション点追加:

```bash
ros2 service call /calibrated_slider_controller_node/add_calibration_point std_srvs/srv/Trigger "{}"
```

- マッピング確定:

```bash
ros2 service call /calibrated_slider_controller_node/finalize_calibration std_srvs/srv/Trigger "{}"
```

- YAML に保存:

```bash
ros2 service call /calibrated_slider_controller_node/save_calibration std_srvs/srv/Trigger "{}"
```



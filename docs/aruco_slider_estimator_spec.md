# **ARマーカー式スライダ位置推定システム 仕様書・設計書 (OpenCV 4対応)**

## **1\. 概要**

本ドキュメントは、ARマーカー（ArUcoマーカー）を用いて、直線運動するスライダの精密な位置を推定するROS 2パッケージの仕様および設計を定義する。

本システムは、静止したベース部分と移動するスライダ部分の両方にマーカーを設置し、カメラからの単一画像内で両者を同時に認識する。これにより、カメラ自体の微小な振動（ブレ）成分を計算過程で相殺し、ロバストで高精度なスライダのワールド座標系における位置姿勢（Pose）をリアルタイムで提供することを目的とする。

## **2\. 用語定義**

| 用語 | 説明 |
| :---- | :---- |
| **ワールド座標系** | システム全体の基準となる静止した3次元座標系。ベース側マーカー群によって定義される。 |
| **カメラ座標系** | カメラの光学中心を原点とする3次元座標系。 |
| **スライダ座標系** | スライダの特定の基準点（例：中心）を原点とするローカルな3次元座標系。 |
| **ベース** | スライダが走行する静止した土台部分。 |
| **スライダ** | ベース上を直線移動する可動部分。カメラとスライダ側マーカーが搭載される。 |
| **Pose** | 位置（Position）と姿勢（Orientation）を合わせた情報。 |

## **3\. システム構成**

### **3.1. ハードウェア構成**

| コンポーネント | 役割・要件 |
| :---- | :---- |
| **PC** | ROS 2および本パッケージを実行するコンピュータ。 |
| **カメラ** | スライダに固定され、スライダと共に移動する。ベースとスライダのマーカーを同時に撮影可能な画角を持つこと。 |
| **ベース側マーカー群** | ベースに、スライダの移動軸と平行な一直線上に、等間隔で複数設置される。ワールド座標系の基準となる。 |
| **スライダ側マーカー群** | スライダ本体に、複数個が剛体として固定される。スライダ座標系の基準となる。 |

#### **構成概念図**

        \[ワールド座標系 (World Frame)\]  
           ^ Z  
           |  
           o-----\> X (スライダ移動軸)

\<-- スライダ移動方向 \--\>  
\+-----------------------------------------------------------------+  
|                                                                 |  
|   \[スライダ\]                                                    |  
|   \+---------------------------------+                           |  
|   | \[カメラ\]                        |                           |  
|   |   ||                            |  \[スライダ側マーカー\]       |  
|   |   \\/                            |   \+----+   \+----+         |  
|   |  (撮影)                         |   | ID |   | ID |         |  
|   \+---------------------------------+   \+----+   \+----+         |  
|                                                                 |  
\+-----------------------------------------------------------------+

\+-----------------------------------------------------------------+ \[ベース\]  
|  \[ベース側マーカー\]                                             |  
|  \+----+      \+----+      \+----+      \+----+      \+----+         |  
|  | ID |      | ID |      | ID |      | ID |      | ID |         |  
|  \+----+      \+----+      \+----+      \+----+      \+----+         |  
\+-----------------------------------------------------------------+

### **3.2. ソフトウェア構成**

| コンポーネント | バージョン等 |
| :---- | :---- |
| **OS** | Ubuntu 22.04 (推奨) |
| **ROS** | ROS 2 Humble Hawksbill (推奨) |
| **主要ライブラリ** | OpenCV 4.x, tf2 |

**注記:** 本システムはOpenCV 4で導入されたcv2.aruco.ArucoDetectorクラスを利用することを前提とする。

## **4\. ROS 2 パッケージ設計**

### **4.1. パッケージ名**

aruco\_slider\_estimator

### **4.2. ノード構成**

単一のノードで全ての処理を完結させる。

* **ノード名:** estimator\_node  
* **役割:** カメラ画像を入力とし、マーカー検出、Pose計算、座標変換を経て、最終的なスライダのPoseを出力する。

### **4.3. トピック、サービス、アクションインターフェース**

#### **サブスクリプション (Inputs)**

| トピック名 | メッセージ型 | 説明 |
| :---- | :---- | :---- |
| \~/input/image\_raw | sensor\_msgs/msg/Image | カメラからの生画像。 |
| \~/input/camera\_info | sensor\_msgs/msg/CameraInfo | カメラの内部パラメータと歪み係数。 |

#### **パブリケーション (Outputs)**

| トピック名 | メッセージ型 | 説明 |
| :---- | :---- | :---- |
| \~/output/slider\_pose | geometry\_msgs/msg/PoseStamped | 推定されたスライダのPose。ヘッダにはワールド座標系名とタイムスタンプが含まれる。 |
| \~/output/debug\_image | sensor\_msgs/msg/Image | 検出したマーカーや座標軸が描画されたデバッグ用画像。 |
| /tf | tf2\_msgs/msg/TFMessage | ワールド座標系からスライダ座標系への座標変換情報をブロードキャストする。RVizでの可視化に利用。 |

### **4.4. パラメータ**

ノード起動時にyamlファイルで設定可能なパラメータ。

| パラメータ名 | 型 | 説明 |
| :---- | :---- | :---- |
| marker\_dictionary | string | 使用するArUcoマーカーの辞書名 (例: DICT\_4X4\_100)。 |
| base\_board\_config | string | ベース側マーカー群の定義ファイルへのパス。 |
| slider\_board\_config | string | スライダ側マーカー群の定義ファイルへのパス。 |
| detector\_params\_file | string | ArUco検出器のパラメータファイル(\*.yaml)へのパス。省略した場合、デフォルト値が使用される。 |
| world\_frame\_id | string | ワールド座標系のフレーム名 (例: world)。 |
| slider\_frame\_id | string | スライダ座標系のフレーム名 (例: slider\_base)。 |
| publish\_tf | bool | /tfトピックをパブリッシュするかどうかのフラグ。 |
| show\_debug\_image | bool | デバッグ用画像をパブリッシュするかどうかのフラグ。 |

## **5\. 処理フロー詳細**

estimator\_nodeは、同期した画像とカメラ情報を受け取るたびに以下の処理を実行する。

1. **初期化処理 (ノード起動時)**  
   1. 全パラメータを読み込む。  
   2. marker\_dictionary名に基づき、ArUcoマーカーの辞書(Dictionary)オブジェクトを取得する。  
   3. ArUco検出器のパラメータ(DetectorParameters)オブジェクトを生成する。detector\_params\_fileが指定されていればファイルから読み込み、なければデフォルト値を使用する。  
   4. 上記で生成した辞書と検出器パラメータを用いて、ArucoDetectorオブジェクトを生成する。  
   5. base\_board\_configとslider\_board\_configファイルから、各マーカー群の3D座標とIDを読み込み、OpenCVのBoardオブジェクトを生成する。  
   6. サブスクライバ、パブリッシャを初期化する。  
2. **メインコールバック処理 (画像受信ごと)**  
   1. **マーカー検出:**  
      * 初期化時に生成したArucoDetectorオブジェクトのdetectMarkers()メソッドを用いて、入力画像(image\_raw)から全てのマーカーのIDとコーナー座標を検出する。  
   2. **Pose推定① (ワールド座標系 → カメラ座標系):**  
      1. ベース側BoardオブジェクトのIDリストを使い、検出されたマーカー群からベース側マーカーのコーナー座標(corners)とID(ids)を抽出する。  
      2. cv2.aruco.Board.matchImagePoints()メソッド（OpenCV 4.7以降）または同等の処理を用いて、抽出したマーカーに対応する3Dオブジェクトポイント(objPoints)と2Dイメージポイント(imgPoints)を取得する。  
      3. 十分な数の対応点が得られた場合、cv2.solvePnPRansac()を実行し、ワールド座標系から見たカメラのPose (rvec1, tvec1) を計算する。  
      4. Pose推定に失敗した、または対応点が不足している場合は後続の処理を中断する。  
   3. **Pose推定② (カメラ座標系 → スライダ座標系):**  
      1. スライダ側BoardオブジェクトのIDリストを使い、検出されたマーカー群からスライダ側マーカーのコーナー座標とIDを抽出する。  
      2. 上記と同様に、Boardオブジェクトから対応する3Dオブジェクトポイントと2Dイメージポイントを取得する。  
      3. 十分な数の対応点が得られた場合、cv2.solvePnPRansac()を実行し、カメラ座標系から見たスライダのPose (rvec2, tvec2) を計算する。  
      4. Pose推定に失敗した、または対応点が不足している場合は後続の処理を中断する。  
   4. **Pose合成と座標変換:**  
      * rvec1, tvec1 と rvec2, tvec2 をそれぞれ4x4の同次変換行列 M\_world\_to\_camera と M\_camera\_to\_slider に変換する。  
      * 行列の乗算 M\_world\_to\_slider \= M\_world\_to\_camera \* M\_camera\_to\_slider を行い、最終的なワールド座標系に対するスライダのPoseを算出する。  
   5. **結果の出力:**  
      * 合成された行列 M\_world\_to\_slider から並進成分と回転成分（クォータニオン）を抽出し、geometry\_msgs/msg/PoseStampedメッセージとして\~/output/slider\_poseにパブリッシュする。  
      * publish\_tfがtrueの場合、同じ変換情報をtf2を用いてブロードキャストする。  
      * show\_debug\_imageがtrueの場合、検出したマーカーや推定した座標軸を描画した画像を\~/output/debug\_imageにパブリッシュする。

## **6\. マーカーの物理配置と設定ファイル**

### **6.1. マーカー設定ファイル形式**

マーカー群の定義には、可読性の高いJSONまたはYAML形式を採用する。

**例: base\_board\_config.json**

{  
  "marker\_size\_mm": 50.0,  
  "markers": \[  
    { "id": 0, "translation": \[0.0, 0.0, 0.0\] },  
    { "id": 1, "translation": \[200.0, 0.0, 0.0\] },  
    { "id": 2, "translation": \[400.0, 0.0, 0.0\] }  
  \]  
}

* marker\_size\_mm: マーカーの一辺の長さ（mm）。  
* translation: 各マーカーの中心の3D座標（mm）。ベース側はワールド座標、スライダ側はスライダのローカル座標で記述する。

### **6.2. マーカーIDの割り当て**

**重要:** ベース側マーカー群とスライダ側マーカー群で使用するIDの範囲は、**重複しないように**割り当てる必要がある。

* 例: ベース側マーカー: ID 0-49 / スライダ側マーカー: ID 50-99

## **7\. ARマーカー作成用スクリプト**

### **7.1. 概要**

本パッケージには、ArUcoマーカーの生成を自動化するスクリプト群が含まれる。これにより、システムに必要なマーカーを簡単に作成し、印刷用の最適化された画像を生成できる。

### **7.2. スクリプト構成**

| スクリプト名 | 機能 |
| :---- | :---- |
| generate\_markers.py | 単一マーカーの生成 |
| generate\_board.py | マーカーボードの生成 |
| generate\_all.py | 設定ファイルからの一括生成 |

### **7.3. ディレクトリ構成**

```
aruco_slider_estimator/
├── scripts/
│   ├── generate_markers.py
│   ├── generate_board.py
│   ├── generate_all.py
│   └── config/
│       ├── marker_generation.yaml
│       ├── templates/
│       │   ├── base_board_template.yaml
│       │   └── slider_board_template.yaml
├── output/
│   ├── markers/
│   │   ├── base/
│   │   └── slider/
│   ├── boards/
│   └── configs/
```

### **7.4. 設定ファイル仕様**

#### **marker\_generation.yaml**

```yaml
# 基本設定
dictionary: DICT_4X4_100
marker_size_mm: 50.0
output_format: png
dpi: 300

# ベース側マーカー
base_markers:
  start_id: 0
  end_id: 4
  spacing_mm: 200.0
  layout: linear

# スライダ側マーカー
slider_markers:
  start_id: 50
  end_id: 53
  spacing_mm: 100.0
  layout: grid

# 出力設定
output:
  individual_markers: true
  board_layout: true
  config_files: true
```

#### **パラメータ詳細**

| パラメータ | 型 | 説明 |
| :---- | :---- | :---- |
| dictionary | string | 使用するArUco辞書名 |
| marker\_size\_mm | float | マーカーの物理サイズ（mm） |
| output\_format | string | 出力形式（png, pdf, svg） |
| dpi | int | 印刷解像度 |
| base\_markers.start\_id | int | ベース側マーカーの開始ID |
| base\_markers.end\_id | int | ベース側マーカーの終了ID |
| base\_markers.spacing\_mm | float | ベース側マーカー間隔（mm） |
| base\_markers.layout | string | レイアウト（linear, grid） |
| slider\_markers.start\_id | int | スライダ側マーカーの開始ID |
| slider\_markers.end\_id | int | スライダ側マーカーの終了ID |
| slider\_markers.spacing\_mm | float | スライダ側マーカー間隔（mm） |
| slider\_markers.layout | string | レイアウト（linear, grid） |

### **7.5. 使用方法**

#### **単一マーカー生成**

```bash
python3 scripts/generate_markers.py --id 0 --size 50 --dictionary DICT_4X4_100
```

#### **マーカーボード生成**

```bash
python3 scripts/generate_board.py --type base --start-id 0 --end-id 4 --spacing 200
```

#### **一括生成**

```bash
python3 scripts/generate_all.py --config config/marker_generation.yaml
```

### **7.6. 出力ファイル**

#### **個別マーカー**
- `output/markers/base/marker_0.png`
- `output/markers/base/marker_1.png`
- `output/markers/slider/marker_50.png`

#### **マーカーボード**
- `output/boards/base_board.png`
- `output/boards/slider_board.png`

#### **設定ファイル**
- `output/configs/base_board_config.json`
- `output/configs/slider_board_config.json`

### **7.7. 印刷仕様**

* **解像度**: 300 DPI（推奨）
* **用紙サイズ**: A4（210mm × 297mm）
* **マージン**: 10mm
* **色**: 黒白（モノクロ）
* **印刷設定**: スケール100%、回転なし

## **8\. 補足事項**

* **カメラキャリブレーション:** 本システムの精度はカメラキャリブレーションの正確さに大きく依存する。事前にros2 run camera\_calibration cameracalibrator等を用いて、camera\_infoを正確に求めておくことが必須である。  
* **堅牢性:** Pose推定にはsolvePnPRansacを使用する。この関数はRANSACアルゴリズムに基づき、マーカーの誤検出などの外れ値（outlier）データを除外して計算を行うため、ロバストなPose推定が可能である。  
* **可視化:** RViz上で/tfとデバッグ画像を可視化することで、システムの動作状況を直感的に確認できる。
* **マーカー作成:** 本パッケージに含まれるスクリプトを使用することで、システムに必要なマーカーを簡単に作成できる。印刷時は高解像度（300 DPI以上）での印刷を推奨する。
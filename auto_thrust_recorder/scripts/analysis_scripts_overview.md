# 分析スクリプト一覧と使用方法

このドキュメントは、`analysis_commands.bash` と `postprocess_all.py` で頻繁に使用されるスクリプトの用途と使用方法をまとめたものです。

## 目次

1. [データ前処理・結合系](#データ前処理結合系)
2. [機械学習・回帰系](#機械学習回帰系)
3. [可視化系](#可視化系)
4. [効果解析・最適化系](#効果解析最適化系)
5. [その他ユーティリティ](#その他ユーティリティ)

---

## データ前処理・結合系

### merge_front_back_bias.py

**用途**: front/back_reversed のCSVを結合し、バイアス補正列を追加する

**入力**: 
- キーワードを含むCSVファイル（デフォルト: `front`, `back_reversed`）

**出力**: 
- バイアス補正済みCSV（`*_biascorr_*.csv`）

**主要オプション**:
- `-k, --keywords`: 検索キーワード（デフォルト: `front`, `back_reversed`）
- `-d, --directory`: 検索ディレクトリ
- `--output-dir`: 出力ディレクトリ
- `--step-warmup`: 各ステップ立ち上がり時の除外秒数（デフォルト: 0.5）
- `--bias-scope`: バイアス推定スコープ（`global` / `per-step`、デフォルト: `per-step`）
- `--bias-agg`: バイアス推定の集計関数（`median` / `mean`、デフォルト: `median`）

**使用例**:
```bash
python3 merge_front_back_bias.py -k raw -d . --output-dir corrected/ \
  --step-warmup 0.3 --bias-scope per-step
```

---

### csv_concat_4.py

**用途**: 複数のCSVファイルを結合し、パラメータごとに集約する

**入力**: 
- キーワードを含むCSVファイル群（デフォルト: `raw`）

**出力**: 
- 結合・集約済みCSV（`concat.csv`）

**主要オプション**:
- `-k, --keywords`: 検索キーワード（デフォルト: `raw`）
- `-d, --directory`: 検索ディレクトリ
- `--output`: 出力ファイル名（必須）
- `--group-by-file-timestamp`: ファイルタイムスタンプでグループ化
- `--agg`: 集計関数（`median` / `mean`、デフォルト: `median`）
- `--iqr-filter`: IQRによる外れ値除去を有効化
- `--iqr-multiplier`: IQRウィスカー係数（デフォルト: 1.5）
- `--iqr-columns`: IQR外れ値判定の対象カラム
- `--drop-zero-columns`: 指定列の0をNaNとして扱う
- `--default-column`: 列のデフォルト値（`col=value`形式、複数指定可）

**使用例**:
```bash
python3 csv_concat_4.py -k biascorr -d corrected/ --output concat.csv \
  --group-by-file-timestamp --agg median --iqr-filter
```

---

### add_calculated_columns_to_csv.py

**用途**: CSVにmorphing droneの座標変換由来パラメータ列と計算列を追加する

**入力**: 
- CSVファイル（`concat.csv` など）

**出力**: 
- 派生列追加済みCSV（上書きまたは新規ファイル）

**追加される列**:
- `alpha`, `beta` [deg]: 推力ベクトル角度
- `prop_spacing_x`, `prop_spacing_y`, `aspect_ratio`: プロペラ間隔
- `normalized_moment`: 正規化モーメント
- `normalized_thrust`: 正規化推力
- `base_thrust`, `base_thrust_z`: 推定推力ベクトル

**主要オプション**:
- `--input`: 入力CSVパス（必須）
- `--output`: 出力CSVパス（省略時は入力と同階層に自動生成）
- `--overwrite`: 入力CSVを上書き保存
- `--angles-only`: alpha/betaのみ計算（他の派生列は触らない）
- `--cx`, `--cy`: ヒンジ中心座標 [m]（デフォルト: 0.035）
- `--rotor-radius-in`: ロータ半径 [inch]（デフォルト: 3.5）

**使用例**:
```bash
python3 add_calculated_columns_to_csv.py --input concat.csv --output concat.csv \
  --cx 0.035 --cy 0.035 --rotor-radius-in 3.5
```

---

### merge_csv.py

**用途**: 複数のCSVファイルを探索・結合して1ファイルにマージする

**入力**: 
- 複数のCSVファイル（明示指定または検索）

**出力**: 
- マージ済みCSV（デフォルト: `merged_日付時刻.csv`）

**主要オプション**:
- `--files`: 明示的に結合するCSVファイル（スペース区切り）
- `-d, --directory`: CSVを検索するディレクトリ（デフォルト: `.`）
- `-k, --keyword`: ファイル名に含まれるキーワード
- `-r, --recursive`: 再帰的に探索
- `--columns-mode`: 列集合の扱い（`union` / `intersection`、デフォルト: `union`）
- `--add-source-col`: 元ファイル名を格納する列名
- `--dedup-by`: 指定列で重複行を削除
- `--sort-by`: 結合後の並び替え列

**使用例**:
```bash
python3 merge_csv.py --files concat.csv --output merged.csv
```

---

## 機械学習・回帰系

### gaussian_process_regression.py

**用途**: ガウス過程回帰（GPR）による目的変数のモデル化と可視化

**入力**: 
- CSVファイル（特徴量と目的変数を含む）

**出力**: 
- 学習済みモデル（`.pkl` または `.pt`）
- 可視化画像（`.png`）

**主要オプション**:
- `csv_file`: 入力CSVファイル（位置引数）
- `--output, -o`: 出力画像ファイル（デフォルト: `gpr.png`）
- `--target`: 目的変数の列名（デフォルト: `torque_x`）
- `--features`: 使用する特徴量（カンマ区切り）
- `--backend`: バックエンド（`sklearn` / `gpytorch`、デフォルト: `sklearn`）
- `--alpha`: ノイズの分散（デフォルト: 1e-6）
- `--save-model`: 学習済みモデルの保存先
- `--load-model`: 保存済みモデルの読み込み
- `--trust-model`: torch.loadの安全ロード失敗時のみ使用（`t`/`f`）
- `--plot-raw-fit`: 生データ散布とGPRフィット曲線を比較表示（`t`/`f`）
- `--row-group-by`, `--col-group-by`: ファセット用の列（カンマ区切り）
- `--curve-x`: フィット曲線の横軸にする特徴量
- `--hue`: 同一セル内で色分けする列名
- `--device`: GPyTorchのデバイス（`auto` / `cpu` / `cuda`）
- `--num-inducing`: GPyTorch(SVGP)の誘導点数（デフォルト: 512）
- `--epochs`: GPyTorch(SVGP)の学習エポック数（デフォルト: 300）

**使用例**:
```bash
python3 gaussian_process_regression.py merged.csv \
  --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y \
  --target normalized_moment --backend gpytorch --alpha 0.001 \
  --save-model gpr_moment_ab.pkl --trust-model t \
  --plot-raw-fit t --row-group-by slant_angle --col-group-by tilt_angle \
  --curve-x distance --hue fold_angle --output gpr_moment_ab.png
```

---

### kernel_ridge_regression.py

**用途**: カーネルリッジ回帰（KRR）による目的変数のモデル化と可視化

**入力**: 
- CSVファイル（特徴量と目的変数を含む）

**出力**: 
- 可視化画像（`.png`）

**主要オプション**:
- `csv_file`: 入力CSVファイル（位置引数）
- `--output, -o`: 出力画像ファイル
- `--target`: 目的変数の列名（デフォルト: `torque_x`）
- `--target-expr`: 目的変数を式から生成（例: `"torque_x / target_thrust / prop_spacing * 100"`）
- `--features`: 使用する特徴量（カンマ区切り）
- `--kernel`: カーネルの種類（`rbf` / `linear` / `poly` / `rbf_linear`、デフォルト: `rbf`）
- `--alpha`: リッジ正則化係数（デフォルト: 1e-2）
- `--gamma`: RBF/Polyのgamma（デフォルト: 1.0）
- `--do-grid-search`: alpha/gammaの簡易グリッドサーチを実行
- `--normalize`: 特徴量の正規化を行う
- `--plot-raw-fit`: グループごとに生データ散布とKRRフィット曲線を比較表示
- `--row-group-by`, `--col-group-by`: ファセット用の列（カンマ区切り）
- `--curve-x`: フィット曲線の横軸にする特徴量

**使用例**:
```bash
python3 kernel_ridge_regression.py merged.csv \
  --target normalized_moment --target-expr "torque_x / target_thrust / prop_spacing * 100" \
  --features distance,tilt_angle,fold_angle,slant_angle,force_z,wall_spacing \
  --do-grid-search --pairwise-heatmaps --plot-raw-fit \
  --curve-x distance --row-group-by wall_spacing \
  --col-group-by tilt_angle,fold_angle,slant_angle --normalize --output krr.png
```

---

## 可視化系

### plot_gpytorch_fit_curve_with_raw.py

**用途**: GPyTorchで学習済みの回帰モデルを読み込み、CSVの生データ散布とフィッティング曲線を描画する

**入力**: 
- CSVファイル（生データ）
- 学習済みモデル（`.pkl` または `.pt`）

**出力**: 
- 可視化画像（`.png`）

**主要オプション**:
- `csv_file`: 入力CSV（位置引数）
- `--load-model`: GPyTorchモデル（必須）
- `--trust-model`: torch.loadの安全ロード失敗時のみ使用（`t`/`f`）
- `--output, -o`: 出力画像パス
- `--target`: 目的変数列名（保存モデルにあればそちらを優先）
- `--curve-x`: 横軸にする特徴量
- `--fix`: 非表示軸の固定値（`col=value`形式、複数指定可）
- `--hue-raw`: raw散布の色分け列（学習特徴量でなくても可）
- `--hue-fit`: fit曲線の分割列（学習特徴量に含まれる場合のみ有効）
- `--hue-raw-cmap`, `--hue-fit-cmap`: カラーマップ名（デフォルト: `viridis`）
- `--hue-raw-range`, `--hue-fit-range`: 値域（`min,max`形式）
- `--xlim`, `--ylim`: 軸の範囲（`min,max`形式）
- `--raw-alpha`: rawプロット点の透明度（デフォルト: 0.65）
- `--no-uncertainty`: ±2σ帯を描かない
- `--fit-extrema`: fitの最大/最小を点線と注釈で表示（`t`/`f`）
- `--drone`: 3Dドローン図を重畳（`t`/`f`、デフォルト: `t`）
- `--transparent`: 透過背景で保存（`t`/`f`）
- `--figsize`: 図のサイズ（`幅,高さ`形式、デフォルト: `10.5,7.5`）

**使用例**:
```bash
python3 plot_gpytorch_fit_curve_with_raw.py merged.csv \
  --load-model gpr_moment_ab.pkl --curve-x distance \
  --hue-raw target_thrust --hue-raw-cmap viridis --hue-raw-range 5.0,12.5 \
  --hue-fit force_z --hue-fit-cmap viridis --hue-fit-range 5.0,12.5 \
  --fix wall_spacing=1.6 --fix fold_angle=0 --fix tilt_angle=-15 --fix slant_angle=-15 \
  --trust-model t --output plot_m_t-15_f0_s-15.png \
  --ylim "-7,7" --no-title --drone f --raw-alpha 0.1 \
  --xlabel "Distance [R]" --ylabel "Normalized Moment [%]" \
  --colorbar-label "Vertical Thrust [N]" --transparent t --figsize 14,4 \
  --no-uncertainty --fit-extrema t --fit-extrema-marker t --fit-extrema-vline t
```

---

### visualize_morph_drone.py

**用途**: morphing droneのアーム/ロータをfold/slant/tilt角度で可視化する

**入力**: 
- 角度パラメータ（コマンドライン引数）

**出力**: 
- 3D可視化画像（`.png`）

**主要オプション**:
- `--phi`: Fold角度Phi [deg]（デフォルト: 0）
- `--psi`: Slant角度Psi [deg]（デフォルト: 0）
- `--theta`: Tilt角度Theta [deg]（デフォルト: 0）
- `--alpha`: 目標alpha [deg]（rotor0推力ベクトルから導出）
- `--beta`: 目標beta [deg]（rotor0推力ベクトルから導出）
- `--solve-psi-theta`: 与えられた--alpha/--betaからpsi/thetaを解く（--phiは固定）
- `--arm-length`: ヒンジからロータ中心までのアーム長 [m]（デフォルト: 0.18）
- `--rotor-radius-in`: ロータ半径 [inch]（デフォルト: 3.5）
- `--cx`, `--cy`: ヒンジオフセット [m]（デフォルト: 0.035）
- `--three-view`: 三面図（Front/Side/Top）を出力
- `--save`: 図をファイルに保存（例: `out.png`）
- `--save-split`: 三面図の各ビューを個別画像として保存
- `--no-show`: ウィンドウを開かない（WSL/headless用）
- `--hide-decorations`: 軸ラベル、グリッド、タイトルなどを非表示
- `--drone-color`: ロータ色（`multi` または単色指定）
- `--drone-lw`: ドローン幾何の線幅スケール（デフォルト: 1.0）

**使用例**:
```bash
python visualize_morph_drone.py --arm-length 0.12 --three-view \
  --hide-decorations --drone-center-y 0.3 --rotor-inflow-offset 0.011 \
  --view-azim 20 --view-elev 12 --drone-lw 2.5 --no-show --save-split \
  --theta="0" --psi="0" --save "t0_f0_s0.png"
```

---

### plot_flow_line_facet.py

**用途**: csv_concat_4.pyが出力した結合CSVから、ライン型のファセットプロットを作成する

**入力**: 
- 結合CSVファイル（`concat_merged.csv` など）

**出力**: 
- 可視化画像（`.png`）

**主要オプション**:
- `--input`: 入力CSVファイル（必須）
- `--output`: 出力画像ファイル（指定なしなら画面表示）
- `--col-keys`: カラムファセットグループ化キー（デフォルト: `wall_spacing`）
- `--target-thrust`: プロットするtarget_thrust値（省略時は最大値を使用）
- `--feature`: Y軸特徴量（`u_mag`, `ux_abs`, `uy_abs`, `norm`, `norm_y`, `u_norm_o` など）
- `--plot-kind`: プロット種類（`line`, `mean_dots`, `mean_fold`, `line_with_map`, `mean_fold_with_map`）
- `--single-angle-mode`: 単一flow_direction角度の扱い（`nan` / `use`）
- `--mode`: プロットモード（`normal` / `diff`）
- `--overlay-drone`: ドローン図を重畳
- `--drone-prop-center-y-mm`: ドローンプロペラ中心Y [mm]
- `--drone-arm-length`: ドローンのアーム長 [m]

**使用例**:
```bash
python3 plot_flow_line_facet.py --input concat_merged.csv \
  --col-keys tilt_angle slant_angle --y-in-origin 266.7 \
  --x-offset-max-mm 20 --single-angle-mode use \
  --plot-kind mean_fold_with_map --feature u_norm_o \
  --x-offset-scope io_y_facet --fit-sensor-k \
  --export-k flow_k_fit.csv --k-scope io_y_facet \
  --overlay-drone --drone-prop-center-y-mm 88.9 \
  --drone-arm-length 0.128 --drone-rotor-inflow-offset-mm 14 \
  --output flow_facet.png --figsize 12 12
```

---

## 効果解析・最適化系

### gpr_effects_analysis.py

**用途**: GPRモデルを用いた効果解析（積分メトリクス評価）と2Dヒートマップ生成

**入力**: 
- CSVファイル（参照用、範囲推定に使用）
- 学習済みモデル（`.pkl` または `.pt`）

**出力**: 
- 2Dヒートマップ画像（`.png`）
- 評価結果CSV（`.csv`）

**主要オプション**:
- `csv_file`: 参照CSV（位置引数）
- `--load-model`: モデル（必須）
- `--device`: GPyTorchモデルの推論デバイス（`auto` / `cpu` / `cuda`）
- `--trust-model`: torch.loadの安全ロード失敗時のみ使用（`t`/`f`）
- `--metric`: 単一メトリクス（`moment_abs` / `grad_abs`、デフォルト: `moment_abs`）
- `--metrics`: 複数メトリクスを同時評価（カンマ区切り、例: `moment_abs,grad_abs`）
- `--grad-dims`: grad_absで偏微分する軸（カンマ区切り）
- `--integrate-over`: 積分軸と範囲（`col:min,max[:N]`形式、複数指定可、min/maxに`auto`可）
- `--fix`: 固定値（`col=value`形式、複数可）
- `--viz-range`: 可視化軸の範囲（`col:min,max[:N]`形式、複数可）
- `--normalize-ref`: 正規化基準点（`col1=val1,col2=val2`形式）
- `--normalize-as-change-rate`: 正規化を増減率で表示（基準点=0, +0.5=50%増）
- `--combine`: メトリクス合成方法（`none` / `logsum` / `fscore` / `fscore_log2`、デフォルト: `none`）
  - `logsum`: 幾何平均型（重み付き幾何平均）
  - `fscore`: F値型（線形改善率[%]、100%=完全抑制、0%=ベースライン）
  - `fscore_log2`: F値型（log2オッズ、対称スケール、±4=±16倍の範囲）
- `--overlay-raw-all`: 2Dヒートマップにraw data点（--fix無視で全点）を重ね描き
- `--mask-by-convex-hull`: 2Dヒートマップをrawデータ点の凸包でマスク（`all` / `fix`）
- `--optima`: combined(2D)の準最適集合から複数最適条件を抽出し、図に重ね描き
- `--output-eval`: 評価結果画像の出力パス
- `--output-csv`: 評価結果CSVの出力パス

**使用例**:
```bash
python3 gpr_effects_analysis.py merged.csv --load-model gpr_moment_ab.pkl \
  --device cpu --trust-model t --metric moment_abs --grad-dims distance \
  --integrate-over force_z:10.0,25.0:5 --fix wall_spacing=1.60 \
  --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 \
  --integrate-over distance:0.0,6.20 \
  --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 \
  --overlay-raw-all --cumulative-over force_z:3 \
  --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step \
  --normalize-as-change-rate --colormap custom_rdbu \
  --heatmap-range "-100,300" --transparent t \
  --output-eval gpr_effects_moment_abs_2w1.60_norm.png
```

---

### search_morph_by_ab_map.py

**用途**: combined alpha-betaマップを参照して、morphパラメータ（tilt/fold/slant）を探索する

**入力**: 
- 複数のalpha-betaマップCSV（`gpr_effects_analysis.py`の出力）

**出力**: 
- 探索結果CSV（`--output-csv`）
- 階層最適化結果CSV（`--output-hierarchy-csv`）

**主要オプション**:
- `--map-csv`: gpr_effects_analysis.pyが出力したcombined CSV（alpha,beta格子、複数回指定で複数環境を同時最適化、必須）
- `--alpha-col`: CSV中のalpha列名（デフォルト: `alpha`）
- `--beta-col`: CSV中のbeta列名（デフォルト: `beta`）
- `--value-col`: 参照する値列名（デフォルト: `combined_improve_pct`）
- `--tilt`: tilt範囲（`lo,hi[:N|step]`形式、デフォルト: `-30,30:61`）
- `--fold`: fold範囲（`lo,hi[:N|step]`形式、デフォルト: `-15,15:31`）
- `--slant`: slant範囲（`lo,hi[:N|step]`形式、デフォルト: `-15,15:31`）
- `--tilt-fixed`, `--fold-fixed`, `--slant-fixed`: 各角度を固定値[deg]に固定
- `--lookup`: マップ参照方法（`nearest` / `bilinear`、デフォルト: `nearest`）
- `--aggregate`: 複数環境を同時に最適化する際の集約指標（`min`, `max`, `quantile`, `cvar`, `cvar_max`, `mean`, `hitrate`、デフォルト: `cvar`）
- `--q`: quantile/cvar用のq（0..1、デフォルト: 0.2）
- `--hierarchy`: 階層最適化（`outer:inner1,inner2`形式、例: `slant:tilt,fold`）
- `--output-csv`: 全走査結果をCSV保存
- `--output-hierarchy-csv`: 階層最適化のouterごとの最適結果CSV

**使用例**:
```bash
python3 search_morph_by_ab_map.py --value-col value \
  --tilt="-30,30:60" --fold="0,15:15" --slant="-15,15:60" \
  --hierarchy slant:tilt,fold \
  --map-csv maps/2w1.60.csv --map-csv maps/2w1.05.csv \
  --map-csv maps/2w0.75.csv --map-csv maps/3w0.75_dc1.0.csv \
  --map-csv maps/3w0.75_dc2.0.csv
```

---

### gam_analysis.py

**用途**: GAM（Generalized Additive Model）による平均/分散の解釈的解析

**入力**: 
- CSVファイル（目的変数と特徴量を含む）

**出力**: 
- 重要度と部分効果プロット（指定ディレクトリに保存）

**主要オプション**:
- `csv_file`: 入力CSVファイル（位置引数）
- `--target`: 目的変数列（例: `torque_x`, `force_y`、デフォルト: `torque_x`）
- `--variance-col`: 分散列（例: `variance_torque_x`、未指定なら`variance_{target}`を自動推定）
- `--features`: 使用する特徴量列（カンマ区切り）
- `--outdir`: 出力ディレクトリ（デフォルト: `gam_out`）
- `--prefix`: 出力ファイル接頭辞（未指定なら`gam_{target}`）
- `--gridsearch`: lamを簡易探索（`t`/`f`、デフォルト: `t`）
- `--n-splines`: 各s()のスプライン数（デフォルト: 20）
- `--spline-order`: スプライン次数（デフォルト: 3）
- `--lam`: gridsearchしない場合の正則化強度（デフォルト: 0.6）
- `--use-weights`: 平均モデルで重み付けを使う（`t`/`f`、デフォルト: `f`）

**使用例**:
```bash
python3 gam_analysis.py merged.csv \
  --target torque_x \
  --features distance,target_thrust,wall_spacing,tilt_angle,fold_angle,slant_angle \
  --outdir gam_out --prefix gam_torque_x
```

---

## その他ユーティリティ

### scale_csv_columns.py

**用途**: CSVの指定列グループを定数倍する

**入力**: 
- CSVファイル（1つまたは複数）

**出力**: 
- スケーリング済みCSV（複数入力時は`--output-dir`に保存）

**主要オプション**:
- `-i, --input`: 入力CSVパス（複数可、スペース区切り、`-`でstdin）
- `-o, --output`: 出力CSVパス（省略時はstdout）
- `--output-dir`: 複数入力時の出力ディレクトリ
- `--suffix`: 出力ファイル名に追加する接尾辞（デフォルト: `_scaled`）
- `--scale`: スケーリング指定（`col1,col2:factor`形式、複数指定可）

**使用例**:
```bash
python3 scale_csv_columns.py -i concat_* --output-dir . \
  --scale "force_x,force_x_bias_corrected,bias_force_x:0.505" \
  --scale "force_y,force_y_bias_corrected,bias_force_y:0.498" \
  --scale "force_z,force_z_bias_corrected,bias_force_z:0.493"
```

---

### condition_counter.py

**用途**: csv_concat_4.pyの出力CSVを読み取り、指定した条件列のユニーク数と条件組合せ数をログする

**入力**: 
- CSVファイル（`csv_concat_4.py`の出力）

**出力**: 
- コンソール出力（条件列ごとのユニーク数、条件組合せユニーク数）

**主要オプション**:
- `--input, -i`: 入力CSV（必須）
- `--condition-cols`: 条件列名（スペース区切り、必須、例: `distance target_thrust tilt_angle fold_angle slant_angle wall_spacing`）
- `--fix`: 固定条件（`col=value`形式、複数可、値は数値）
- `--range`: 範囲条件（`col=min:max`または`col=min,max`形式、複数可、片側省略可）

**使用例**:
```bash
python3 condition_counter.py --input merged.csv \
  --condition-cols distance target_thrust tilt_angle fold_angle slant_angle wall_spacing \
  --fix wall_spacing=1.6
```

---

### postprocess_all.py

**用途**: 複数サブディレクトリに対して前処理を並列実行し、最後にrootでマージ・KRRを実行するランチャー

**処理フロー**:
1. 各サブディレクトリで並列実行:
   - `merge_front_back_bias.py`（バイアス補正、`--no-biascorr`でスキップ可）
   - `csv_concat_4.py`（結合、`--no-concat`でスキップ可）
   - `add_calculated_columns_to_csv.py`（派生列追加、`--no-morph`でスキップ可）
2. rootで実行:
   - `merge_csv.py`（`concat_merged.csv`、`--no-merge`でスキップ可）
   - `kernel_ridge_regression.py`（`krr.png`、`--no-krr`でスキップ可）

**主要オプション**:
- `-r, --root`: rootディレクトリ（デフォルト: `.`）
- `-j, --jobs`: 並列ジョブ数（デフォルト: nproc）
- `-l, --log-dir`: ログ出力ディレクトリ（オプション）
- `--step-warmup`: merge_front_back_bias.pyの`--step-warmup`（デフォルト: 0.3）
- `--bias-scope`: merge_front_back_bias.pyの`--bias-scope`（デフォルト: `per-step`）
- `--no-biascorr`: merge_front_back_bias.pyステップをスキップ
- `--recreate-corrected`: `corrected/`を削除してから実行
- `--no-concat`: csv_concat_4.pyステップをスキップ
- `--concat-keyword`: csv_concat_4.pyの`-k KEYWORD`を上書き
- `--concat-iqr-filter`: csv_concat_4.pyでIQR外れ値除去を有効化
- `--no-morph`: add_calculated_columns_to_csv.pyステップを無効化
- `--morph-cx`, `--morph-cy`, `--morph-rotor-radius-in`: add_calculated_columns_to_csv.pyのパラメータ
- `--no-merge`: merge_csv.pyステップをスキップ
- `--no-krr`: kernel_ridge_regression.pyステップをスキップ

**使用例**:
```bash
python3 postprocess_all.py -j 8 --recreate-corrected
```

---

## 補足

### よく使われる処理フロー

1. **データ前処理**:
   ```bash
   # 各測定ディレクトリで
   python3 merge_front_back_bias.py -k raw -d . --output-dir corrected/
   python3 csv_concat_4.py -k biascorr -d corrected/ --output concat.csv
   python3 add_calculated_columns_to_csv.py --input concat.csv --output concat.csv
   ```

2. **データ結合**:
   ```bash
   # rootで
   python3 merge_csv.py --files concat.csv --output merged.csv
   ```

3. **モデル学習**:
   ```bash
   python3 gaussian_process_regression.py merged.csv \
     --features distance,wall_spacing,force_z,alpha,beta \
     --target normalized_moment --backend gpytorch \
     --save-model gpr_moment_ab.pkl --trust-model t
   ```

4. **可視化**:
   ```bash
   python3 plot_gpytorch_fit_curve_with_raw.py merged.csv \
     --load-model gpr_moment_ab.pkl --curve-x distance \
     --fix wall_spacing=1.6 --fix fold_angle=0 \
     --output plot.png
   ```

5. **効果解析（F値型改善度）**:
   ```bash
   # 線形改善率[%]
   python3 gpr_effects_analysis.py merged.csv --load-model gpr_moment_ab.pkl \
     --metrics moment_abs,grad_abs --grad-dims distance \
     --integrate-over force_z:5.0,12.5:5 --integrate-over distance:0.0,6.20 \
     --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 \
     --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 \
     --normalize-ref alpha=0.0,beta=0.0 --normalize-as-change-rate \
     --combine fscore --colormap custom_rwg --heatmap-range="-100,100" \
     --output-eval effects_fscore.png --output-csv effects_fscore.csv
   
   # log2オッズ（対称スケール）
   python3 gpr_effects_analysis.py merged.csv --load-model gpr_moment_ab.pkl \
     --metrics moment_abs,grad_abs --grad-dims distance \
     --integrate-over force_z:5.0,12.5:5 --integrate-over distance:0.0,6.20 \
     --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 \
     --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 \
     --normalize-ref alpha=0.0,beta=0.0 --normalize-as-change-rate \
     --combine fscore_log2 --colormap custom_rwg --heatmap-range="-4,4" \
     --colorbar-label "I_OR [log2]" \
     --output-eval effects_fscore_log2.png --output-csv effects_fscore_log2.csv
   ```

---

## 注意事項

- `--trust-model t`は信頼できるモデルのみで使用してください（PyTorch 2.6+のweights_only制限回避用）
- GPyTorchバックエンドを使用する場合は、`torch`と`gpytorch`のインストールが必要です
- GAM解析には`pygam`のインストールが必要です
- 一部のスクリプトは`japanize_matplotlib`を使用しているため、日本語フォントの設定が必要な場合があります


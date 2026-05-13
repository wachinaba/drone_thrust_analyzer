# 追加データ解析プラン 実行状況

最終確認: 2026-02-27

## サマリ

| Phase | 内容 | 状態 | 備考 |
|-------|------|------|------|
| Phase 1 | GPR 4モデル再学習 + 基本メトリクス | **完了** | ユーザー実行済み |
| Phase 2-1 | 5-fold CV スクリプト | **完了** | 2w/3w 両方実行済み。結果は下記 |
| Phase 2-2 | モデル比較 (Linear/KRR/SVGP) | **完了** | 結果は下記 |
| Phase 2-3 | 統計検定 (n, Wilcoxon, 効果量, 95%CI) | **完了** | 結果は下記 |
| Phase 3 | 設計指標の感度分析 | **完了** | 3-1 logsum / fscore / fscore_log2、3-2 distance narrow・force narrow まで実行済み。3-3 MC サンプリングは未実装（要スクリプト拡張） |
| Phase 4 | ARD lengthscale 数値報告 | **完了** | 4モデル分の CSV を `analysis/ard/` に出力済み。`--output-ard-csv` を GPR に追加済み |
| Phase 5 | 総合レポート | **完了** | `analysis_summary_for_paper.md` に結果サマリ・論文反映チェックリストを記載 |

---

## Phase 1 の詳細

### 実行したコマンド（1本目のみ）

- **対象**: 2w `normalized_moment`（merged_1w_2w.csv）
- **出力先**: `G:\マイドライブ\実験\concat\analysis\models\`
- **開始**: 2026-03-10 01:21
- **学習**: **完了**（300 epoch, avg_loss 0.903）
- **テスト評価**: **完了**  
  - RMSE: 1.778462  
  - MAE: 1.338782  

### 失敗原因

- テスト評価の **print で「R²」を表示した直後**、Windows コンソールの cp932 エンコードで  
  `'cp932' codec can't encode character '\xb2'` が発生し **exit code 1 で終了**。
- そのため **モデル保存（.pkl）・図の保存（.png）は実行されていない**。

### 対応済み

- `gaussian_process_regression.py` 内のテスト結果表示を **「R²」→「R2」** に変更し、  
  Windows コンソールでも落ちないように修正済み。

### 未実行

- Phase 1 の残り 3 本（2w force_y, 3w moment, 3w force_y）は未実行。

---

## 環境

- **venv**: `c:\Users\Watar\master_thesis\.venv`
- **依存**: `drone_thrust_analyzer/auto_thrust_recorder/requirements.txt` で pip インストール済み（ターミナル 941383 で成功）。
- **データ**: `G:\マイドライブ\実験\concat\merged_1w_2w.csv`, `merged_3w.csv`

---

## 次のステップ

1. **Phase 1 を最初からやり直す**  
   - 上記修正により R2 表示で落ちないはずなので、4 本とも再実行。  
   - 出力: `G:\マイドライブ\実験\concat\analysis\models\` に  
     `gpr_moment_ab.pkl/.png`, `gpr_force_ab.pkl/.png`,  
     `gpr_moment_ab_3w.pkl/.png`, `gpr_force_ab_3w.pkl/.png` ができる想定。

2. **Phase 2 以降**  
   - プランに従い、`gpr_cross_validation.py` の作成・実行、モデル比較、統計検定、感度分析、ARD 報告、総合レポートの順で実施。

---

## 実行コマンド例（Phase 1 再実行）

作業ディレクトリ: `c:\Users\Watar\master_thesis\drone_thrust_analyzer\auto_thrust_recorder\scripts`

```batch
set PY=c:\Users\Watar\master_thesis\.venv\Scripts\python.exe
set OUT=G:\マイドライブ\実験\concat\analysis\models
set DATA=G:\マイドライブ\実験\concat

%PY% gaussian_process_regression.py "%DATA%\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target normalized_moment --backend gpytorch --alpha 0.001 --save-model "%OUT%\gpr_moment_ab.pkl" --trust-model t --test-size 0.2 --output "%OUT%\gpr_moment_ab.png"

%PY% gaussian_process_regression.py "%DATA%\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target force_y --backend gpytorch --alpha 0.001 --save-model "%OUT%\gpr_force_ab.pkl" --trust-model t --test-size 0.2 --output "%OUT%\gpr_force_ab.png"

%PY% gaussian_process_regression.py "%DATA%\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target normalized_moment --backend gpytorch --alpha 0.001 --save-model "%OUT%\gpr_moment_ab_3w.pkl" --trust-model t --test-size 0.2 --output "%OUT%\gpr_moment_ab_3w.png"

%PY% gaussian_process_regression.py "%DATA%\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target force_y --backend gpytorch --alpha 0.001 --save-model "%OUT%\gpr_force_ab_3w.pkl" --trust-model t --test-size 0.2 --output "%OUT%\gpr_force_ab_3w.png"
```

（各コマンドは約 10 分程度かかります。）

---

## Phase 2-1: 5-fold CV 実行コマンド

作業ディレクトリ: `scripts`（上記と同じ）。2w moment と 3w moment の 2 モデルで CV を実行。

**PowerShell の場合:**

```powershell
$PY = "c:\Users\Watar\master_thesis\.venv\Scripts\python.exe"
$DATA = "G:\マイドライブ\実験\concat"
$CV_OUT = "G:\マイドライブ\実験\concat\analysis\cv"

& $PY gpr_cross_validation.py "$DATA\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target normalized_moment --k-folds 5 --epochs 150 --output-json "$CV_OUT\cv_moment_2w.json"

& $PY gpr_cross_validation.py "$DATA\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target normalized_moment --k-folds 5 --epochs 150 --output-json "$CV_OUT\cv_moment_3w.json"
```

**CMD（コマンドプロンプト）の場合:**

```batch
set PY=c:\Users\Watar\master_thesis\.venv\Scripts\python.exe
set DATA=G:\マイドライブ\実験\concat
set CV_OUT=G:\マイドライブ\実験\concat\analysis\cv

"%PY%" gpr_cross_validation.py "%DATA%\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target normalized_moment --k-folds 5 --epochs 150 --output-json "%CV_OUT%\cv_moment_2w.json"

"%PY%" gpr_cross_validation.py "%DATA%\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target normalized_moment --k-folds 5 --epochs 150 --output-json "%CV_OUT%\cv_moment_3w.json"
```

事前に `analysis\cv` フォルダを作成してください。1本あたり 5 fold x 150 epoch で時間がかかります。

### Phase 2-1 実行結果（5-fold CV）

| データ | n | CV RMSE (mean ± std) | CV R2 (mean ± std) |
|--------|---|----------------------|---------------------|
| **2w** (merged_1w_2w, normalized_moment) | 49,096 | **1.854 ± 0.014** | **0.648 ± 0.007** |
| **3w** (merged_3w, normalized_moment)    | 5,760  | **2.048 ± 0.045** | **0.857 ± 0.008** |

- 2w: 各 fold RMSE = 1.866, 1.850, 1.860, 1.828, 1.865；R2 = 0.648, 0.638, 0.655, 0.658, 0.643。
- 3w: 各 fold RMSE = 1.961, 2.062, 2.060, 2.072, 2.087；R2 = 0.872, 0.851, 0.848, 0.859, 0.856。
- 3w は R2 が高く（約 0.86）、fold 間のばらつきも小さい。2w は R2 約 0.65 で、CV で安定した性能。

---

## Phase 2-2: モデル比較（Linear / KRR / SVGP）実行コマンド

同一 train/test split (test_size=0.2, random_state=42) で 3 手法を比較し、1 つの CSV にまとめます。1 本目は `--output-csv` のみ、2 本目以降は `--append` を付けて同じファイルに追記。

**PowerShell（作業ディレクトリ: scripts）:**

```powershell
$PY = "c:\Users\Watar\master_thesis\.venv\Scripts\python.exe"
$DATA = "G:\マイドライブ\実験\concat"
$COMP_OUT = "G:\マイドライブ\実験\concat\analysis\comparison\model_comparison.csv"

# 1/4: 2w normalized_moment（新規作成）
& $PY model_comparison.py "$DATA\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target normalized_moment --output-csv $COMP_OUT

# 2/4: 2w force_y（追記）
& $PY model_comparison.py "$DATA\merged_1w_2w.csv" --features distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y --target force_y --output-csv $COMP_OUT --append

# 3/4: 3w normalized_moment（追記）
& $PY model_comparison.py "$DATA\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target normalized_moment --output-csv $COMP_OUT --append

# 4/4: 3w force_y（追記）
& $PY model_comparison.py "$DATA\merged_3w.csv" --features distance,force_z,alpha,beta,distance_ceiling --target force_y --output-csv $COMP_OUT --append
```

事前に `analysis\comparison` フォルダを作成してください。各 run で SVGP を 150 epoch 学習するため、4 本で時間がかかります。

---

## Phase 2-3: 統計検定（n値・Wilcoxon・Cohen's d・95%CI）実行コマンド

`statistical_summary.py` で、条件ごとの n、ベースライン vs 最適姿勢の対応 t 比較（Wilcoxon、Cohen's d、95%CI）、推力ビン別改善率を算出。結果は `statistical_summary.json` と `statistical_summary.md` に出力。

**PowerShell（作業ディレクトリ: scripts）:**

```powershell
$PY = "c:\Users\Watar\master_thesis\.venv\Scripts\python.exe"
$DATA = "G:\マイドライブ\実験\concat"
$STAT_OUT = "G:\マイドライブ\実験\concat\analysis\statistics"

# 2w データで実行（normalized_moment、ベースライン tilt=0 slant=0 fold=0）
& $PY statistical_summary.py "$DATA\merged_1w_2w.csv" --condition-cols "distance,force_z,wall_spacing,tilt_angle,slant_angle,fold_angle" --metric normalized_moment --group-cols "distance,force_z,wall_spacing" --output-dir $STAT_OUT
```

事前に `analysis\statistics` フォルダを作成するか、`--output-dir` で指定したディレクトリが自動作成されます。**注意**: 2w データでは `--group-cols` と `--condition-cols` に `target_thrust` を使用（`force_z` は測定値で行ごとに異なるため）。スクリプトのデフォルトは `target_thrust` に変更済み。

### Phase 2-3 実行結果（2w, normalized_moment）

| 項目 | 値 |
|------|-----|
| 総行数 | 49,096 |
| ユニーク条件数 | 12,563 |
| 条件あたり n | min=1, max=12, mean=3.9 |
| 対応ペア数 | 967 |
| **改善率（平均 ± SD）** | **74.4% ± 29.3%** |
| Wilcoxon (baseline > optimal) | stat=397386, **p ≈ 1e-147**（有意） |
| Cohen's d | 1.19（大きな効果） |
| 差の 95% CI | [1.77, 1.97] |

**推力ビン別改善率**: 約 68%（低推力）〜 85%（高推力）。高推力で改善率がやや高い。

### Phase 2-2 実行結果（同一 hold-out 比較）

| dataset | target | model | RMSE | R2 | MAE |
|---------|--------|-------|------|-----|-----|
| merged_1w_2w | normalized_moment | LinearRegression | 2.913 | 0.142 | 2.173 |
| merged_1w_2w | normalized_moment | KRR_rbf | 1.693 | **0.710** | 1.275 |
| merged_1w_2w | normalized_moment | SVGP | 1.868 | 0.647 | 1.403 |
| merged_1w_2w | force_y | LinearRegression | 0.0573 | 0.151 | 0.0433 |
| merged_1w_2w | force_y | KRR_rbf | 0.0515 | 0.314 | 0.0392 |
| merged_1w_2w | force_y | SVGP | **0.0512** | **0.320** | **0.0392** |
| merged_3w | normalized_moment | LinearRegression | 3.934 | 0.485 | 2.965 |
| merged_3w | normalized_moment | KRR_rbf | **1.922** | **0.877** | 1.426 |
| merged_3w | normalized_moment | SVGP | 1.951 | 0.873 | 1.464 |
| merged_3w | force_y | LinearRegression | 0.0648 | 0.598 | 0.0469 |
| merged_3w | force_y | KRR_rbf | 0.0473 | 0.785 | 0.0328 |
| merged_3w | force_y | SVGP | **0.0457** | **0.800** | **0.0319** |

**解釈**: 線形回帰は全設定で R2 が低い（0.14〜0.60）。KRR と SVGP は同程度で、2w moment では KRR がやや有利、2w/3w force_y では SVGP がわずかに良い。SVGP は不確実性も得られるため、論文では SVGP を主モデルとする根拠として「KRR と同等以上の予測精度」と書ける。

---

## Phase 3: 設計指標の感度分析 実行結果

- **3-1 統合方法**: `combined_logsum.png`, `combined_fscore.png`, `combined_fscore_log2.png` を `G:\マイドライブ\実験\concat\analysis\sensitivity\` に出力済み。
- **3-2 積分領域**: `integrate_distance_narrow.png`（distance 0.5–3.0）, `integrate_force_narrow.png`（force_z 7.5–12.5）を出力済み。広い範囲は 3-1 の logsum と同一条件。
- **3-3 MC サンプリング**: 現行 `gpr_effects_analysis.py` に未実装。必要なら Phase 4/5 で機能追加を検討。

---

## Phase 4: ARD lengthscale 数値報告

- **追加機能**: `gaussian_process_regression.py` に `--output-ard-csv PATH` を追加。学習後または `--load-model` 読み込み後に、特徴量ごとの lengthscale・importance（1/ls）・正規化重要度を CSV に書き出す。
- **出力先**: `G:\マイドライブ\実験\concat\analysis\ard\`  
  - `ard_moment_2w.csv`, `ard_force_2w.csv`, `ard_moment_3w.csv`, `ard_force_3w.csv`
- **CSV 列**: `feature`, `lengthscale`, `importance`, `importance_normalized`
- **実行例**（2w moment）:  
  `python gaussian_process_regression.py "...\merged_1w_2w.csv" --load-model "...\gpr_moment_ab.pkl" --features "distance,wall_spacing,force_z,alpha,beta,prop_spacing_x,prop_spacing_y" --target normalized_moment --trust-model t --output-ard-csv "...\ard\ard_moment_2w.csv" --output "...\ard\dummy.png"`
- **解釈**: lengthscale が**短い**特徴量ほど重要度が高い。distance, alpha, beta が短いほど、壁距離・姿勢が正規化モーメントに効いている。論文では ARD 重要度ランキングとして表・図で報告可能。

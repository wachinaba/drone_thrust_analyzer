# Phase 3: 設計指標の感度分析 — 実行手順

**実行済み（2026-03-11）**: 3-1（logsum / fscore / fscore_log2）および 3-2（distance narrow, force narrow）はすべて実行済み。出力は `G:\マイドライブ\実験\concat\analysis\sensitivity\`。

`gpr_effects_analysis.py` を用い、統合方法・積分領域の感度を評価する。

## 前提

- **Python**: プロジェクト venv の `python.exe`
- **CSV**: `G:\マイドライブ\実験\concat\merged_1w_2w.csv`
- **モデル**: `G:\マイドライブ\実験\concat\analysis\models\gpr_moment_ab.pkl`
- **出力先**: `G:\マイドライブ\実験\concat\analysis\sensitivity\`（事前に作成するか自動作成）

作業ディレクトリ: `drone_thrust_analyzer/auto_thrust_recorder/scripts`

**重要**: `gpr_effects_analysis.py` は `auto_thrust_recorder` パッケージを import するため、**bash で実行する場合は先に次を実行してください。**

```bash
export PYTHONPATH="c:/Users/Watar/master_thesis/drone_thrust_analyzer/auto_thrust_recorder"
```

PowerShell の場合は:

```powershell
$env:PYTHONPATH = "c:\Users\Watar\master_thesis\drone_thrust_analyzer\auto_thrust_recorder"
```

---

## 3-1. 統合方法の感度（logsum / fscore / fscore_log2）

同一条件で `--combine` のみ変更し、3 本実行。出力図を比較して最適姿勢の違いを確認する。

```powershell
$PY = "c:\Users\Watar\master_thesis\.venv\Scripts\python.exe"
$DATA = "G:\マイドライブ\実験\concat"
$MODEL = "$DATA\analysis\models\gpr_moment_ab.pkl"
$OUT = "$DATA\analysis\sensitivity"

# logsum（幾何平均型）
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:5.0,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,6.2 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine logsum --output-eval "$OUT\combined_logsum.png"

# fscore（線形改善率）
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:5.0,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,6.2 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine fscore --output-eval "$OUT\combined_fscore.png"

# fscore_log2（log2 オッズ）
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:5.0,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,6.2 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine fscore_log2 --output-eval "$OUT\combined_fscore_log2.png"
```

---

## 3-2. 積分領域 Ω の感度

### distance 範囲

- 狭い範囲（近距離のみ）: `distance:0.5,3.0`
- 広い範囲（全範囲）: `distance:0.5,6.2`

```powershell
# 狭い範囲
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:5.0,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,3.0 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine logsum --output-eval "$OUT\integrate_distance_narrow.png"

# 広い範囲
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:5.0,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,6.2 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine logsum --output-eval "$OUT\integrate_distance_wide.png"
```

### force_z 範囲

- 広い範囲: `force_z:5.0,12.5`
- 狭い範囲: `force_z:7.5,12.5`

```powershell
# 狭い推力範囲
& $PY gpr_effects_analysis.py "$DATA\merged_1w_2w.csv" --load-model $MODEL --device cpu --trust-model t --metrics moment_abs,grad_abs --grad-dims distance --integrate-over force_z:7.5,12.5:5 --fix wall_spacing=1.60 --fix prop_spacing_x=0.25 --fix prop_spacing_y=0.25 --integrate-over distance:0.5,6.2 --viz-range alpha:-40.0,40.0:30 --viz-range beta:-40.0,40.0:30 --normalize-ref alpha=0.0,beta=0.0 --normalize-ref-per-step --normalize-as-change-rate --combine logsum --output-eval "$OUT\integrate_force_narrow.png"

# 広い推力範囲（上記 3-1 の combined_logsum.png と同一）
# 既に combined_logsum.png で force_z:5.0,12.5 を使用済み
```

---

## 3-3. 改善率 I(ω) の不確実性（MC サンプリング）

プランでは GP 予測から N 回サンプリングし、各サンプルで I(ω) を計算して 95% CI を報告する想定。  
現行の `gpr_effects_analysis.py` には MC サンプリングオプションはないため、実施する場合はスクリプトへの機能追加が必要。Phase 4/5 で必要に応じて実装する。

---

## 実行前の準備

```powershell
New-Item -ItemType Directory -Force -Path "G:\マイドライブ\実験\concat\analysis\sensitivity"
cd c:\Users\Watar\master_thesis\drone_thrust_analyzer\auto_thrust_recorder\scripts
```

各コマンドは数分かかることがあります。

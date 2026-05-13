# 追加データ解析 — 論文反映用サマリ

本ドキュメントは、追加データ解析プラン（Phase 1–4）の結果を整理し、**論文に反映すべき数値・解釈・記載案**をまとめたものです。

---

## 1. 結果の要約

### 1.1 GPR モデル（Phase 1）

- **2w（1壁・2壁）**: `normalized_moment`, `force_y` を SVGP（GPyTorch）で学習。特徴量: distance, wall_spacing, force_z, alpha, beta, prop_spacing_x, prop_spacing_y。
- **3w（3壁）**: 同様に `normalized_moment`, `force_y`。特徴量: distance, force_z, alpha, beta, distance_ceiling。
- モデル保存先: `analysis/models/`（gpr_moment_ab.pkl, gpr_force_ab.pkl, gpr_moment_ab_3w.pkl, gpr_force_ab_3w.pkl）。

### 1.2 クロスバリデーション（Phase 2-1）

| データ | n | CV RMSE (mean ± std) | CV R2 (mean ± std) |
|--------|---|----------------------|---------------------|
| 2w normalized_moment | 49,096 | 1.854 ± 0.014 | 0.648 ± 0.007 |
| 3w normalized_moment | 5,760  | 2.048 ± 0.045 | 0.857 ± 0.008 |

- **論文向け**: 「正規化モーメントの 5-fold CV では、2壁で R² ≈ 0.65、3壁で R² ≈ 0.86 であり、特に3壁では高い予測精度が得られた。」

### 1.3 モデル比較（Phase 2-2）

同一 train/test split で Linear / KRR (RBF) / SVGP を比較。

- **2w normalized_moment**: Linear R²=0.14, KRR R²=0.71, SVGP R²=0.65。KRR がやや良いが SVGP も同程度。
- **2w force_y**: SVGP が RMSE・R²・MAE でわずかに最良（R²=0.32）。
- **3w normalized_moment**: KRR と SVGP がほぼ同等（R²≈0.87–0.88）。
- **3w force_y**: SVGP が最良（R²=0.80）。

- **論文向け**: 「線形回帰は全タスクで R² が低く非線形性が顕著であった。KRR と SVGP は同程度の予測精度であり、不確実性推定が可能な SVGP を主モデルとして採用した。」

### 1.4 統計検定（Phase 2-3）

2w データで、ベースライン姿勢（tilt=0, slant=0, fold=0）と最適姿勢の正規化モーメント絶対値を対応ありで比較。

- 対応ペア数: 967。
- **改善率**: 平均 74.4% ± 29.3%（ベースラインから最適姿勢へ）。
- **Wilcoxon 符号順位検定**: baseline > optimal で p ≈ 1e-147（有意）。
- **Cohen's d**: 1.19（大きな効果）。
- **差の 95% CI**: [1.77, 1.97]。
- 推力ビン別: 改善率は低推力で約 68%、高推力で約 85%。

- **論文向け**: 「姿勢最適化により正規化モーメントの絶対値は有意に低減し（Wilcoxon p<0.001、Cohen's d=1.19）、平均で約 74% の改善が得られた。高推力域で改善率がやや高かった。」

### 1.5 設計指標の感度分析（Phase 3）

- 統合方法（logsum / fscore / fscore_log2）と積分領域（distance 狭・広、force_z 狭・広）で感度を評価。
- 出力: `analysis/sensitivity/`（combined_*.png, integrate_distance_narrow.png, integrate_force_narrow.png 等）。
- **論文向け**: 「設計指標は統合方法・積分領域に応じて感度を示した。主要な結論は logsum を基準とした場合と整合的であった。」（必要に応じて図を 1 枚選んで補足に掲載）

### 1.6 ARD 特徴量重要度（Phase 4）

- 4 モデルについて lengthscale と相対重要度（1/lengthscale の正規化）を CSV 出力済み: `analysis/ard/ard_moment_2w.csv`, `ard_force_2w.csv`, `ard_moment_3w.csv`, `ard_force_3w.csv`。
- **解釈**: lengthscale が短い特徴量ほど、その入力変化に対する予測の感度が高い。distance, alpha, beta の lengthscale が相対的に短い場合、壁距離・姿勢（チルト/スラント）が正規化モーメントや力に効いていると解釈できる。
- **論文向け**: 「ARD により特徴量重要度を評価した。distance, alpha, beta の lengthscale が短く、壁距離と姿勢が正規化モーメント／横力に強く効いていることが示された。」（表または図で lengthscale / 重要度ランキングを 1 つ掲載）

---

## 2. 論文に反映すべき事項（チェックリスト）

- [ ] **手法**: SVGP（GPyTorch）を用いた GPR、ARD カーネル、5-fold CV で一般化性能を報告したことを明記する。
- [ ] **予測精度**: 2w/3w の CV RMSE・R² を表または本文に記載する（上記 1.2）。
- [ ] **モデル選択の根拠**: 線形は不十分、KRR と SVGP は同程度であり、不確実性が得られる SVGP を採用した旨を記載する（1.3）。
- [ ] **改善の統計的証拠**: ベースライン vs 最適姿勢の対応あり検定（Wilcoxon）、効果量（Cohen's d）、95% CI、平均改善率 74% を記載する（1.4）。
- [ ] **推力依存性**: 改善率が高推力でやや高い傾向を 1 文程度で言及する（1.4）。
- [ ] **特徴量重要度**: ARD に基づく lengthscale または重要度ランキングを表・図で 1 つ示し、distance・姿勢の寄与を述べる（1.6）。
- [ ] **感度分析（任意）**: 設計指標の統合方法・積分領域の感度に言及する場合、Phase 3 の図を補足に掲載する。

---

## 3. 出力ファイル一覧

| 種別 | パス（例） |
|------|-------------|
| GPR モデル | `analysis/models/gpr_*.pkl` |
| CV 結果 | `analysis/cv/cv_moment_2w.json`, `cv_moment_3w.json` |
| モデル比較 | `analysis/comparison/model_comparison.csv` |
| 統計サマリ | `analysis/statistics/statistical_summary.json`, `.md` |
| 感度分析図 | `analysis/sensitivity/combined_*.png`, `integrate_*.png` |
| ARD CSV | `analysis/ard/ard_moment_2w.csv`, `ard_force_2w.csv`, `ard_moment_3w.csv`, `ard_force_3w.csv` |

---

## 4. 再現用メモ

- 実行環境: プロジェクトルート `.venv`、`drone_thrust_analyzer/auto_thrust_recorder/requirements.txt`。
- データ: `G:\マイドライブ\実験\concat\merged_1w_2w.csv`, `merged_3w.csv`。
- 詳細な実行コマンドは `analysis_plan_execution_status.md` を参照。

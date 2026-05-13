---
marp: true
theme: default
paginate: true
math: mathjax
style: |
  section {
    font-family: 'Noto Sans JP', 'Meiryo', 'Yu Gothic', 'Helvetica Neue', Arial, sans-serif;
    font-size: 24px;
  }
  h1 { font-size: 34px; color: #1a365d; }
  h2 { font-size: 28px; color: #2c5282; }
  table { font-size: 19px; margin: 0 auto; }
  th { background: #2c5282; color: white; padding: 4px 10px; }
  td { padding: 4px 10px; }
  .columns { display: flex; gap: 24px; }
  .col { flex: 1; }
  strong { color: #c53030; }
  .note { font-size: 18px; color: #718096; font-style: italic; }
---

# フライトログ壁効果解析: DiD 分析サマリー

閉ループ姿勢制御下での壁効果検出と機体構成間比較

---

## 実験セットアップ

- ベンチ固定 + **PX4 姿勢制御ループ動作** (STABILIZED モード)
- MAVLink スクリプトで **4 フェーズを自動実行**
- ULog から姿勢・角速度・トルク指令・モータ出力を抽出

![w:950](analysis_output/slides/phase_concept.png)

---

## Group B 代表ラン: タイムライン

![w:1000](analysis_output/plot1_timeline.png)

---

## Group A 代表ラン: タイムライン

![w:1000](analysis_output/group_a/plot1_timeline.png)W

---

## Difference-in-Differences (DiD) の考え方

2 グループはトリム設定が異なる → 絶対値の直接比較はバイアスを含む

$$\Delta_i = \text{metric}(\text{Hover2}_{\text{壁近接}}) - \text{metric}(\text{Hover1}_{\text{壁なし}})$$

各ランで壁効果の前後差 Δ をとることでトリムバイアスが**キャンセル**され、
**壁効果そのものの大きさ**をグループ間で公平に比較できる

<br>

| | Group A (baseline?, n=4) | Group B (morphed?, n=11) |
|---|---|---|
| Roll trim | −0.10 〜 −0.16 | −0.20 |
| Δ を比較 | 壁効果の大きさ A | 壁効果の大きさ B |

---

## DiD 全 6 指標

![w:920](analysis_output/slides/did_all_metrics.png)

---

## DiD 分析の解釈

<div class="columns">
<div class="col">

### 有意な指標 (p < 0.05)

- **Motor Asym M1−M3** (d=4.21, p=0.001)
  壁効果の**方向が反転**
  Group A: Δ = −0.022 / Group B: Δ = +0.008

- **Roll Integ Mean** (d=2.37, p=0.001)
  定常外乱補償の**蓄積方向が反転**
  Group A: Δ = −0.004 / Group B: Δ = +0.002

</div>
<div class="col">

### 有意でないが傾向あり

- **Roll Rate Std** (d=−0.78, p=0.280)
  Group A の壁効果が **2.3 倍**
  Δ = +5.48 vs +2.40 deg/s

- **Roll Torque Std** (d=−0.56, p=0.343)
  Group A の壁効果が **1.8 倍**
  Δ = +0.034 vs +0.019

- いずれも **n=4 で検出力不足**

</div>
</div>

---

## 核心: モータ非対称度の壁効果方向が反転

![w:950](analysis_output/plot8_did_analysis.png)

壁効果の**大きさ**だけでなく**方向**が機体構成で反転 (d = 4.21, p = 0.001)
→ ベンチ計測の壁効果モーメント符号変化と整合

---

## 何を意味するか

| 観点 | Group A (baseline?) | Group B (morphed?) |
|---|---|---|
| Motor Asym の壁効果 Δ | **−0.022** (壁側モータ減) | **+0.008** (壁側モータ増) |
| Roll Integ の壁効果 Δ | **−0.004** (負方向蓄積) | **+0.002** (正方向微増) |
| Roll Rate Std の壁効果 Δ | +5.48 deg/s | +2.40 deg/s |
| Roll Torque RMS (Hover2) | 0.086 | 0.031 (**1/3**) |

<br>

- 機体変形は壁効果を単に「小さく」するだけでなく、**空力応答の質を変えている**
- 制御系が壁効果を補償する方向自体が反転 → **6 軸力センサなしで検出可能**
- ベンチ計測 (モーメント最大 90% 低減) の知見が制御系レベルで裏付けられた

# SVGP（Sparse Variational Gaussian Process）ハイパーパラメータ詳細

## 概要

`gaussian_process_regression.py` では、GPyTorchライブラリを使用したSVGP（Sparse Variational Gaussian Process）が実装されています。SVGPは誘導点（inducing points）を用いた変分近似により、大規模データセットでも効率的にガウス過程回帰を実行できます。

---

## ハイパーパラメータ一覧

### 1. 誘導点数（`--num-inducing`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 512 |
| 型 | int |
| 有効範囲 | 4 ～ データサンプル数 |

**根拠:**
- 誘導点数は計算量とモデル表現力のトレードオフを決定する重要なパラメータ
- 512は中規模データセット（数千〜数万点）で一般的に良好なバランスを提供
- 計算量は O(M²N) （M: 誘導点数、N: データ数）のため、512程度であれば実用的な学習時間を維持
- コード内で `m = max(4, min(m, n))` により、データ数より大きくならないよう制限

```python
# inducing points
m = int(getattr(args, "num_inducing", 512) or 512)
m = max(4, min(m, n))
rs = np.random.RandomState(int(getattr(args, "random_state", 42) or 42))
idx = rs.choice(n, size=m, replace=False) if n > m else np.arange(n)
inducing = torch.from_numpy(X_np[idx]).to(device)
```

---

### 2. 学習エポック数（`--epochs`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 300 |
| 型 | int |

**根拠:**
- 変分推論の収束には十分な反復が必要
- 300エポックはELBO（Evidence Lower Bound）の収束に一般的に十分
- GPyTorchの公式チュートリアルでも類似の設定（100〜500）が使用される
- 早期収束時のリソース浪費を避けるため、`--log-every`でモニタリング可能

---

### 3. 学習率（`--lr`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 0.01 |
| 型 | float |

**根拠:**
- Adamオプティマイザとの組み合わせで標準的な値
- GPyTorchの推奨範囲（0.001〜0.1）の中間値
- 大きすぎると発散、小さすぎると収束が遅くなるバランス点
- 誘導点位置とカーネルパラメータを同時に学習する場合の安定性を考慮

```python
optimizer = torch.optim.Adam(list(model.parameters()) + list(likelihood.parameters()), lr=lr)
mll = gpytorch.mlls.VariationalELBO(likelihood, model, num_data=n)
```

---

### 4. バッチサイズ（`--batch-size`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 1024 |
| 型 | int |

**根拠:**
- ミニバッチ確率的変分推論（SVI）のためのパラメータ
- 1024はGPUメモリ効率とグラディエント推定の分散のバランス
- 大規模データでも各エポックの計算時間を抑制
- バッチサイズが小さいと勾配のノイズが増加、大きすぎるとメモリ不足のリスク

---

### 5. 予測バッチサイズ（`--pred-batch-size`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 8192 |
| 型 | int |

**根拠:**
- 予測時は勾配計算が不要なため、学習時より大きなバッチが可能
- 8192は多くのGPU（8GB以上）で効率的に処理可能なサイズ
- 大規模予測グリッド（ヒートマップ生成など）での効率化

```python
def predict(self, X, return_std=True):
    # ...
    bs = max(1, int(self.pred_batch_size))
    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        for i in range(0, n, bs):
            xb = torch.from_numpy(X_np[i:i+bs]).to(self.device)
            pred = self.likelihood(self.model(xb))
```

---

### 6. カーネルタイプ（`--kernel`）

| 選択肢 | 説明 |
|--------|------|
| `rbf`（デフォルト） | RBF（Radial Basis Function）カーネル |
| `matern` | Matérnカーネル |
| `rbf_white` | RBF + WhiteNoiseカーネル |
| `rbf_linear` | RBF + 線形カーネル |

**根拠:**
- **RBF**: 無限微分可能、滑らかな関数に適する。最も汎用的
- **Matérn**: 微分可能性を制御可能（nuパラメータ）、物理現象のモデリングに適する
- **RBF + Linear**: 線形トレンドと非線形成分の両方を捕捉

```python
if kernel_type in ["rbf", "rbf_white", "rbf_linear"]:
    base_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=ard_num_dims)
elif kernel_type == "matern":
    base_kernel = gpytorch.kernels.MaternKernel(nu=float(matern_nu), ard_num_dims=ard_num_dims)

covar = gpytorch.kernels.ScaleKernel(base_kernel)
if use_linear:
    covar = covar + gpytorch.kernels.LinearKernel()
```

---

### 7. 異方性 / ARD（`--anisotropic`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | True |
| 型 | bool (t/f) |

**根拠:**
- ARD（Automatic Relevance Determination）により各特徴量ごとに独立した長さスケールを学習
- 特徴量の重要度を自動推定可能（逆長さスケール ∝ 重要度）
- 異なるスケールの特徴量が混在する場合に有効
- Trueの場合、`ard_num_dims=n_features` が設定される

```python
ard = d if bool(getattr(args, "anisotropic", False)) else None
```

---

### 8. Matérn νパラメータ（`--matern-nu`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 1.5 |
| 型 | float |
| 典型的な値 | 0.5, 1.5, 2.5 |

**根拠:**
- νは関数の滑らかさ（微分可能性）を制御
  - ν = 0.5: 連続だが微分不可能（Ornstein-Uhlenbeck過程）
  - ν = 1.5: 1回微分可能
  - ν = 2.5: 2回微分可能
  - ν → ∞: RBFカーネルに収束
- 1.5は多くの物理・工学データで良好な汎化性能を示す

---

### 9. ノイズ分散 / α（`--alpha`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | 1e-6 |
| 型 | float |

**根拠:**
- 観測ノイズの初期値として使用
- GPyTorchでは `likelihood.noise` の初期値として設定
- 学習中に最適化されるため、初期値は保守的（小さめ）に設定
- 数値安定性のため `max(a, 1e-6)` で下限を設定

```python
likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
try:
    a = float(getattr(args, "alpha", 1e-6) or 1e-6)
    if np.isfinite(a) and a > 0:
        likelihood.noise = max(a, 1e-6)
except Exception:
    pass
```

---

### 10. 長さスケール境界（`--length-scale-bounds`）

| 項目 | 値 |
|------|-----|
| デフォルト値 | None（制約なし） |
| 形式 | "下限,上限"（例: "1e-2,1e3"） |

**根拠:**
- カーネルの長さスケールの探索範囲を制限
- GPyTorchでは `gpytorch.constraints.Interval` で実装
- 事前知識がある場合やオーバーフィッティング防止に有効
- 指定しない場合はGPyTorchのデフォルト制約が適用

```python
if length_scale_bounds is not None:
    try:
        lo, hi = float(length_scale_bounds[0]), float(length_scale_bounds[1])
        if (lo > 0) and (hi > lo):
            base_kernel.register_constraint("raw_lengthscale", gpytorch.constraints.Interval(lo, hi))
    except Exception:
        pass
```

---

### 11. デバイス（`--device`）

| 選択肢 | 説明 |
|--------|------|
| `auto`（デフォルト） | CUDA利用可能ならGPU、なければCPU |
| `cpu` | CPU強制 |
| `cuda` | GPU強制 |

**根拠:**
- GPUがあれば自動的に利用することで学習を高速化
- CUDA非対応環境でも動作保証
- 大規模データ・多数の誘導点ではGPUが必須級の高速化を提供

```python
def _select_torch_device(device_str: str):
    if torch is None:
        return None
    s = (device_str or "auto").strip().lower()
    if s == "cpu":
        return torch.device("cpu")
    if s == "cuda":
        return torch.device("cuda")
    # auto
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")
```

---

## 内部実装の詳細

### 変分分布
- **CholeskyVariationalDistribution**: コレスキー分解による変分分布の表現
- 誘導点数と同じ次元の変分パラメータを持つ

### 変分戦略
- **VariationalStrategy**: 標準的なSVGP戦略
- `learn_inducing_locations=True`: 誘導点位置も学習対象

### 損失関数
- **VariationalELBO**: 変分下界（ELBO）の最大化
- `num_data=n` でKLダイバージェンス項を適切にスケーリング

### 予測時の高速化
- `gpytorch.settings.fast_pred_var()`: 予測分散の高速計算を有効化

---

## 目的変数の正規化

| 項目 | 処理 |
|------|------|
| 正規化 | `y_norm = (y - y_mean) / y_std` |
| 逆変換（予測時） | `m * y_std + y_mean` |

**根拠:**
- sklearnのGPRの `normalize_y=True` に相当
- 数値安定性の向上
- カーネルパラメータの解釈性向上

```python
y_mean = float(np.mean(y_np)) if n > 0 else 0.0
y_std = float(np.std(y_np)) if n > 0 else 1.0
if not np.isfinite(y_std) or y_std <= 0.0:
    y_std = 1.0
y_norm = (y_np - y_mean) / y_std
```

---

## 推奨設定ガイドライン

| データ規模 | 誘導点数 | バッチサイズ | エポック数 |
|------------|----------|--------------|------------|
| 小（〜1,000） | 100-200 | 256 | 100-200 |
| 中（1,000〜10,000） | 256-512 | 512-1024 | 200-300 |
| 大（10,000〜100,000） | 512-1024 | 1024-2048 | 300-500 |
| 超大規模（100,000〜） | 1024-2048 | 2048-4096 | 500+ |

---

## コマンドライン引数一覧（SVGP関連）

```bash
# 基本的な使用例
python gaussian_process_regression.py data.csv \
    --backend gpytorch \
    --num-inducing 512 \
    --epochs 300 \
    --lr 0.01 \
    --batch-size 1024 \
    --pred-batch-size 8192 \
    --kernel rbf \
    --anisotropic t \
    --alpha 1e-6 \
    --device auto \
    --output results.png \
    --save-model model.pkl
```

---

## 参考文献・根拠

1. **GPyTorch公式ドキュメント**: https://gpytorch.ai/
2. **Hensman et al. (2015)**: "Scalable Variational Gaussian Process Classification" - SVGP の理論的基盤
3. **Wilson & Nickisch (2015)**: "Kernel Interpolation for Scalable Structured Gaussian Processes" - 誘導点の効率的な配置
4. **Rasmussen & Williams (2006)**: "Gaussian Processes for Machine Learning" - GPの基礎理論

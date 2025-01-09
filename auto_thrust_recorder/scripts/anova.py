import argparse
import pandas as pd
import numpy as np
import statsmodels.api as sm
from statsmodels.stats.anova import anova_lm
from scipy import stats
import seaborn as sns
import matplotlib.pyplot as plt
import sys

def perform_shapiro_wilk(data, group_cols, dependent_var):
    """
    各グループに対してShapiro-Wilk検定を実施し、結果を返します。
    """
    shapiro_results = []
    grouped = data.groupby(group_cols)
    for name, group in grouped:
        if len(group[dependent_var]) < 3:
            print(f"Warning: Group {name} has less than 3 observations. Shapiro-Wilk test may not be reliable.", file=sys.stderr)
            stat, p = np.nan, np.nan
            normal = np.nan
        else:
            stat, p = stats.shapiro(group[dependent_var])
            normal = p > 0.05
        shapiro_results.append({
            'Group': name,
            'Shapiro-Wilk Statistic': stat,
            'p-value': p,
            'Normal': normal
        })
    return pd.DataFrame(shapiro_results)

def perform_levene_test(data, group_cols, dependent_var):
    """
    Levene検定を実施し、結果を返します。
    """
    groups = [group[dependent_var].values for name, group in data.groupby(group_cols)]
    if len(groups) < 2:
        print("Error: Levene's test requires at least two groups.", file=sys.stderr)
        sys.exit(1)
    stat, p = stats.levene(*groups)
    return {'Levene Statistic': stat, 'p-value': p, 'Homogeneous': p > 0.05}

def perform_ancova(data, formula, weights):
    """
    共分散分析（ANCOVA）を実施し、結果を返します。
    """
    try:
        ancova_model = sm.WLS.from_formula(formula, data=data, weights=weights)
        ancova_res = ancova_model.fit(cov_type='HAC', cov_kwds={'maxlags':1})
        return ancova_res
    except Exception as e:
        print(f"Error during ANCOVA model fitting: {e}", file=sys.stderr)
        sys.exit(1)

def compute_effect_sizes(anova_table, total_ss):
    """
    効果量（η^2）を計算します。
    """
    effect_sizes = {}
    for factor, row in anova_table.iterrows():
        if not pd.isnull(row['sum_sq']):
            effect_sizes[factor] = row['sum_sq'] / total_ss
    return effect_sizes

def visualize_boxplot(data, group_cols, dependent_var, output_file):
    """
    箱ひげ図を作成して保存します。
    """
    plt.figure(figsize=(10, 6))
    sns.boxplot(x=group_cols[0], y=dependent_var, hue=group_cols[1], data=data)
    plt.title("Boxplot of " + dependent_var + " by " + " and ".join(group_cols))
    plt.show()
    print(f"Boxplot saved to {output_file}")

def visualize_effect_sizes(effect_sizes, output_file):
    """
    効果量の積み上げ横棒グラフを作成して保存します（全体を100%に正規化）。
    """
    plt.figure(figsize=(8, 6))
    factors = list(effect_sizes.keys())
    eta_squared = list(effect_sizes.values())
    total = sum(eta_squared)
    eta_squared_percent = [val / total * 100 for val in eta_squared]

    # 積み上げ
    plt.bar(factors, eta_squared_percent, label='η²')
    plt.xlabel("Effect Size (% of Total)")
    plt.ylabel("Factors")
    plt.title("Normalized Effect Sizes (η²) by Factor")
    plt.tight_layout()
    plt.show()
    print(f"Effect size bar chart saved to {output_file}")

def offset_by_distance(df, dependent_var, target_disatnce):
    ref_df = df[df["distance"] == target_disatnce].groupby(["tilt_angle", "prop_spacing"])[dependent_var].mean().reset_index().rename(columns={dependent_var: f"{dependent_var}_ref"})
    df = pd.merge(df, ref_df, on=["tilt_angle", "prop_spacing"], how="left")
    df[dependent_var] = df[dependent_var] - df[f"{dependent_var}_ref"]
    return df

def main():
    parser = argparse.ArgumentParser(description='Perform ANOVA and ANCOVA with WLS and check assumptions.')
    parser.add_argument('--input', '-i', required=True, help='Input CSV file containing the data.')
    parser.add_argument('--output', '-o', default='analysis_result.txt', help='Output file to save ANOVA and test results.')
    parser.add_argument('--boxplot', '-b', default='boxplot.png', help='Output file for boxplot.')
    parser.add_argument('--effect_sizes_chart', '-e', default='effect_sizes.png', help='Output file for effect size bar chart.')
    parser.add_argument('--dependent_var', '-d', required=True, help='Dependent variable for analysis.')
    args = parser.parse_args()

    # === データ読み込み ===
    try:
        df = pd.read_csv(args.input)
    except FileNotFoundError:
        print(f"Error: File '{args.input}' not found.", file=sys.stderr)
        sys.exit(1)
    except pd.errors.EmptyDataError:
        print(f"Error: File '{args.input}' is empty.", file=sys.stderr)
        sys.exit(1)
    except pd.errors.ParserError:
        print(f"Error: File '{args.input}' is malformed.", file=sys.stderr)
        sys.exit(1)

    dependent_var = args.dependent_var

    # 必要な列が存在するか確認
    required_columns = ['tilt_angle', 'prop_spacing', 'distance', dependent_var, f'variance_{dependent_var}']
    for col in required_columns:
        if col not in df.columns:
            print(f"Error: Column '{col}' not found in the input data.", file=sys.stderr)
            sys.exit(1)

    # distance = 5.0を基準として、それ以外のデータをオフセットする
    df = offset_by_distance(df, dependent_var, 5.0)

    # カテゴリカル変数に変換
    df['tilt_angle'] = df['tilt_angle'].astype('category')
    df['prop_spacing'] = df['prop_spacing'].astype('category')

    # 重みの計算
    if (df[f'variance_{dependent_var}'] == 0).any():
        print(f"Error: 'variance_{dependent_var}' contains zero(s), which would lead to infinite weights.", file=sys.stderr)
        sys.exit(1)
    df['weight'] = 1.0 / df[f'variance_{dependent_var}']

    # === ANCOVAの実行 ===
    formula_ancova = f'{dependent_var} ~ C(tilt_angle)*C(prop_spacing) + distance + target_thrust'
    ancova_res = perform_ancova(data=df, formula=formula_ancova, weights=df['weight'])

    shapiro_results = perform_shapiro_wilk(df, ['tilt_angle', 'prop_spacing'], dependent_var)

    # === ANCOVAのANOVA表 ===
    try:
        ancova_anova_res = anova_lm(ancova_res, typ=2)
    except Exception as e:
        print(f"Error during ANCOVA ANOVA computation: {e}", file=sys.stderr)
        sys.exit(1)

    # === 効果量の計算 ===
    total_ss = ancova_anova_res['sum_sq'].sum()
    effect_sizes = compute_effect_sizes(ancova_anova_res, total_ss)

    # 効果量の棒グラフを可視化
    visualize_effect_sizes(effect_sizes, args.effect_sizes_chart)

    # コンソール出力
    print("=== ANCOVA Results ===")
    print(ancova_res.summary())
    print("\n=== ANCOVA ANOVA Table ===")
    print(ancova_anova_res)
    print("\n=== Effect Sizes (η^2) ===")
    for factor, eta_sq in effect_sizes.items():
        print(f"{factor}: {eta_sq:.4f}")
    print("\n=== Shapiro-Wilk Test Results ===")
    print(shapiro_results)

if __name__ == '__main__':
    main()

import numpy as np
import pandas as pd
import argparse
import sys
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns

def calculate_k(data, distance_col, torque_x_col):
    """
    各行に対してKを算出します。distance=5.0の行は除外します。
    K = torque_x / (5.0 - distance)
    """
    # distanceが5.0の行を除外
    filtered_data = data[data[distance_col] != 5.0].copy()
    
    if filtered_data.empty:
        print("Error: No data left after filtering distance=5.0.", file=sys.stderr)
        sys.exit(1)
    
    # Kの計算
    filtered_data['K'] = filtered_data[torque_x_col] / (5.0 - filtered_data[distance_col])
    
    return filtered_data

def perform_two_way_anova(data, tilt_angle_col, prop_spacing_col, dependent_var_col):
    """
    statsmodelsを用いて二元配置ANOVAを実行します。
    """
    # ANOVAモデルの定義
    formula = f'{dependent_var_col} ~ C({tilt_angle_col}) + C({prop_spacing_col}) + C({tilt_angle_col}):C({prop_spacing_col})'
    model = ols(formula, data=data).fit()
    
    # ANOVA表の作成
    anova_table = sm.stats.anova_lm(model, typ=2)
    
    return anova_table

def perform_tukey_hsd(data, factor_col, dependent_var_col, alpha=0.05):
    """
    TukeyのHSD検定を実行します。
    """
    tukey = pairwise_tukeyhsd(endog=data[dependent_var_col],
                              groups=data[factor_col],
                              alpha=alpha)
    return tukey

def check_normality(data, tilt_angle_col, prop_spacing_col, dependent_var_col):
    """
    Shapiro-Wilk検定を用いて正規性を検定します。
    """
    print("\n--- 正規性の検定（Shapiro-Wilk検定） ---")
    for (tilt, spacing), group_data in data.groupby([tilt_angle_col, prop_spacing_col]):
        stat, p = stats.shapiro(group_data[dependent_var_col])
        print(f'Group (Tilt: {tilt}, Spacing: {spacing}) - Shapiro-Wilk p-value: {p:.4f}')
    print("--------------------------------------------\n")

def check_homogeneity(data, tilt_angle_col, prop_spacing_col, dependent_var_col):
    """
    Levene検定を用いて等分散性を検定します。
    """
    groups = [group[dependent_var_col].values for name, group in data.groupby([tilt_angle_col, prop_spacing_col])]
    stat, p = stats.levene(*groups)
    print("\n--- 等分散性の検定（Levene検定） ---")
    print(f"Levene's test statistic: {stat:.4f}, p-value: {p:.4f}")
    print("----------------------------------------\n")

def plot_data(data, tilt_angle_col, prop_spacing_col, dependent_var_col):
    """
    データの可視化（ヒストグラム、Q-Qプロット、ボックスプロット、交互作用プロット）を行います。
    """
    # ヒストグラム
    plt.figure(figsize=(8, 6))
    sns.histplot(data[dependent_var_col], kde=True, bins=30)
    plt.title(f'Histogram of {dependent_var_col}')
    plt.xlabel(dependent_var_col)
    plt.ylabel('Frequency')
    plt.tight_layout()
    plt.show()
    
    # Q-Qプロット
    sm.qqplot(data[dependent_var_col], line='45')
    plt.title(f'Q-Q Plot of {dependent_var_col}')
    plt.tight_layout()
    plt.show()
    
    # ボックスプロット
    plt.figure(figsize=(10, 6))
    sns.boxplot(x=tilt_angle_col, y=dependent_var_col, hue=prop_spacing_col, data=data)
    plt.title(f'Boxplot of {dependent_var_col} by {tilt_angle_col} and {prop_spacing_col}')
    plt.xlabel(tilt_angle_col)
    plt.ylabel(dependent_var_col)
    plt.legend(title=prop_spacing_col)
    plt.tight_layout()
    plt.show()
    
    # 交互作用プロット
    plt.figure(figsize=(10, 6))
    means = data.groupby([tilt_angle_col, prop_spacing_col])[dependent_var_col].mean().unstack()
    means.plot(kind='line', marker='o')
    plt.title('Interaction Plot')
    plt.xlabel(tilt_angle_col)
    plt.ylabel(f'Mean {dependent_var_col}')
    plt.legend(title=prop_spacing_col)
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Calculate K and perform Two-Way ANOVA with improved analysis')
    parser.add_argument('-i', '--input', required=True, help='Path to input CSV file')
    parser.add_argument('--distance', default='distance', help='Column name for distance')
    parser.add_argument('--torque_x', default='torque_x', help='Column name for torque_x')
    parser.add_argument('--tilt_angle', default='tilt_angle', help='Column name for tilt_angle')
    parser.add_argument('--prop_spacing', default='prop_spacing', help='Column name for prop_spacing')
    parser.add_argument('-o', '--output', help='Path to output file (optional)')
    parser.add_argument('--save_plots', action='store_true', help='Save plots instead of displaying them')
    
    args = parser.parse_args()
    
    # データの読み込み
    try:
        data = pd.read_csv(args.input)
    except FileNotFoundError:
        print(f"Error: File '{args.input}' not found.", file=sys.stderr)
        sys.exit(1)
    except pd.errors.EmptyDataError:
        print(f"Error: File '{args.input}' is empty.", file=sys.stderr)
        sys.exit(1)
    except pd.errors.ParserError:
        print(f"Error: File '{args.input}' is malformed.", file=sys.stderr)
        sys.exit(1)
    
    # 必要な列が存在するか確認
    required_columns = [args.distance, args.torque_x, args.tilt_angle, args.prop_spacing]
    for col in required_columns:
        if col not in data.columns:
            print(f"Error: Column '{col}' not found in the input data.", file=sys.stderr)
            sys.exit(1)
    
    # Kの計算
    processed_data = calculate_k(
        data,
        distance_col=args.distance,
        torque_x_col=args.torque_x
    )
    
    # ANOVAの実行
    anova_results = perform_two_way_anova(
        processed_data,
        tilt_angle_col=args.tilt_angle,
        prop_spacing_col=args.prop_spacing,
        dependent_var_col='K'
    )
    
    # ANOVA結果の整形
    anova_output = anova_results.to_string()
    
    # 前提条件の検証
    check_normality(processed_data, args.tilt_angle, args.prop_spacing, 'K')
    check_homogeneity(processed_data, args.tilt_angle, args.prop_spacing, 'K')
    
    # 事後検定（Tukey HSD）
    print("\n--- Tukey's HSD Post Hoc Test for tilt_angle ---")
    tukey_results = perform_tukey_hsd(processed_data, args.tilt_angle, 'K')
    print(tukey_results)
    print("--------------------------------------------\n")
    
    # データの可視化
    if not args.save_plots:
        plot_data(processed_data, args.tilt_angle, args.prop_spacing, 'K')
    else:
        # プロットを保存する場合
        sns.set(style="whitegrid")
        
        # ヒストグラム
        plt.figure(figsize=(8, 6))
        sns.histplot(processed_data['K'], kde=True, bins=30)
        plt.title('Histogram of K')
        plt.xlabel('K')
        plt.ylabel('Frequency')
        plt.tight_layout()
        plt.savefig('histogram_k.png')
        plt.close()
        
        # Q-Qプロット
        sm.qqplot(processed_data['K'], line='45')
        plt.title('Q-Q Plot of K')
        plt.tight_layout()
        plt.savefig('qqplot_k.png')
        plt.close()
        
        # ボックスプロット
        plt.figure(figsize=(10, 6))
        sns.boxplot(x=args.tilt_angle, y='K', hue=args.prop_spacing, data=processed_data)
        plt.title('Boxplot of K by Tilt Angle and Prop Spacing')
        plt.xlabel(args.tilt_angle)
        plt.ylabel('K')
        plt.legend(title=args.prop_spacing)
        plt.tight_layout()
        plt.savefig('boxplot_k.png')
        plt.close()
        
        # 交互作用プロット
        plt.figure(figsize=(10, 6))
        means = processed_data.groupby([args.tilt_angle, args.prop_spacing])['K'].mean().unstack()
        means.plot(kind='line', marker='o')
        plt.title('Interaction Plot')
        plt.xlabel(args.tilt_angle)
        plt.ylabel('Mean K')
        plt.legend(title=args.prop_spacing)
        plt.tight_layout()
        plt.savefig('interaction_plot_k.png')
        plt.close()
        print("プロットを保存しました。")
    
    # ANOVA結果の出力
    if args.output:
        try:
            with open(args.output, 'w') as f:
                f.write("=== Two-Way ANOVA Results ===\n")
                f.write(anova_output + '\n\n')
                f.write("=== Tukey's HSD Post Hoc Test for tilt_angle ===\n")
                f.write(str(tukey_results) + '\n')
            print(f"結果を '{args.output}' に保存しました。")
        except IOError as e:
            print(f"Error: Could not write to file '{args.output}'. {e}", file=sys.stderr)
            sys.exit(1)
    else:
        print("\n=== Two-Way ANOVA Results ===")
        print(anova_results)
        print("\n=== Tukey's HSD Post Hoc Test for tilt_angle ===")
        print(tukey_results)

if __name__ == "__main__":
    main()

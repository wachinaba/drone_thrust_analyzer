#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import os
from sklearn.linear_model import LinearRegression

def parse_arguments():
    """
    コマンドライン引数を解析します。
    """
    parser = argparse.ArgumentParser(description="CSVファイルを読み込み、各 (distance, tilt_angle, prop_spacing) グループごとに target_thrust と指定列の関係をプロットします。distance=5.0を基準にデータをオフセットし、分散をエラーバーとして表示します。また、回帰係数と相関係数をヒートマップで表示し、distance vs offset のプロットを2x2グリッドで表示します。")
    parser.add_argument('--input', '-i', type=str, required=True, help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', type=str, default=None, help='ファセットプロットの出力画像ファイルのパス (例: facet_plot.png)')
    parser.add_argument('--distance_ref', '-d', type=float, default=5.0, help='オフセットの基準となる distance の値 (デフォルト: 5.0)')
    parser.add_argument('--col', '-c', type=str, default='force_y', help='プロットする列名')
    parser.add_argument('--col_display', '-cd', type=str, default='force_y', help='プロットする列名(表示用)')
    parser.add_argument('--col_unit', '-u', type=str, default='N', help='プロットする列の単位')
    parser.add_argument('--invert_col', '-ic', action='store_true', help='プロットする列を反転するかどうか')
    return parser.parse_args()

def read_and_validate_csv(csv_file):
    """
    CSVファイルを読み込み、必要な列が存在するか確認します。
    
    Parameters:
        csv_file (str): CSVファイルのパス。
    
    Returns:
        pd.DataFrame: 読み込まれたデータフレーム。
    """
    required_columns = [
        'target_thrust', 'sample_count', 'control',
        'force_x', 'force_y', 'force_z',
        'torque_x', 'torque_y', 'torque_z',
        'variance_force_x', 'variance_force_y', 'variance_force_z',
        'variance_torque_x', 'variance_torque_y', 'variance_torque_z',
        'distance', 'tilt_angle', 'prop_spacing'
    ]
    
    if not os.path.isfile(csv_file):
        print(f"エラー: ファイル '{csv_file}' が存在しません。")
        sys.exit(1)
    
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"エラー: ファイル '{csv_file}' の読み込みに失敗しました。\n詳細: {e}")
        sys.exit(1)
    
    # 必要な列がすべて存在するか確認
    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        print(f"エラー: CSVファイルに必要な列が不足しています: {missing_columns}")
        sys.exit(1)
    
    # 必要な列のみを選択
    df = df[required_columns].copy()
    
    return df

def process_data(df, col, distance_ref, invert_col=False):
    """
    データフレームに標準偏差の列とオフセット列を追加します。
    
    各 (tilt_angle, prop_spacing, target_thrust) 組み合わせのデータにおいて、
    distance = distance_ref のデータ (col 列) を target_thrust 軸で平均化したものを
    バイアスとみなし、各データから引きます。
    
    Parameters:
        df (pd.DataFrame): 入力データフレーム。
        col (str): プロットする列名。
        distance_ref (float): オフセットの基準となる distance の値。
        invert_col (bool): プロットする列を反転するかどうか。
    Returns:
        pd.DataFrame: 標準偏差とオフセットを追加したデータフレーム。
    """
    # プロットする列を反転するかどうか
    if invert_col:
        df[col] = -df[col]

    # 1. 分散から標準偏差を計算
    df[f'std_{col}'] = np.sqrt(df[f'variance_{col}'])
    
    # 2. 基準データ (distance=distance_ref) の抽出と 平均化
    ref_df = (
        df[df['distance'] == distance_ref]
        .groupby(['tilt_angle', 'prop_spacing'])[col]
        .mean()
        .reset_index()
        .rename(columns={col: f'{col}_ref'})
    )
    
    # 3. 基準データを元のデータフレームにマージ
    df = pd.merge(
        df,
        ref_df,
        on=['tilt_angle', 'prop_spacing'],
        how='left'
    )
    
    # 4. オフセットデータの計算
    df[f'offset_{col}'] = df[col] - df[f'{col}_ref']
    
    return df

def calculate_correlations(df, col):
    """
    各グループごとに target_thrust と指定列の相関係数を計算します。
    
    Parameters:
        df (pd.DataFrame): 入力データフレーム。
        col (str): 相関を計算する列名。
    
    Returns:
        pd.DataFrame: グループごとの相関係数を含むデータフレーム。
    """
    correlation_df = df.groupby(['distance', 'tilt_angle', 'prop_spacing']).apply(
        lambda x: x['target_thrust'].corr(x[f'offset_{col}'])
    ).reset_index(name='correlation')
    return correlation_df

def calculate_regression_slopes(df, col):
    """
    各グループごとに target_thrust と指定列の回帰直線の傾きを計算します。
    
    Parameters:
        df (pd.DataFrame): 入力データフレーム。
        col (str): y軸にプロットする列名。
    
    Returns:
        pd.DataFrame: グループごとの回帰直線の傾きを含むデータフレーム。
    """
    slopes = []
    for name, group in df.groupby(['distance', 'tilt_angle', 'prop_spacing']):
        X = group['target_thrust'].values.reshape(-1, 1)
        y = group[f'offset_{col}'].values
        if len(X) > 1 and np.std(X) > 0:
            model = LinearRegression()
            model.fit(X, y)
            slope = model.coef_[0]
        else:
            slope = np.nan  # データポイントが不足している場合
        slopes.append({
            'distance': name[0],
            'tilt_angle': name[1],
            'prop_spacing': name[2],
            'slope': slope
        })
    slope_df = pd.DataFrame(slopes)
    return slope_df

def plot_facetgrid(df, col, col_unit, correlation_df, slope_df):
    """
    ファセットプロットを作成し、各サブプロットにエラーバーを追加します。
    タイトルは簡略化されています。
    
    Parameters:
        df (pd.DataFrame): プロットするデータフレーム。
        col (str): プロットする列名。
        col_unit (str): プロットする列の単位。
        correlation_df (pd.DataFrame): 相関係数を含むデータフレーム。
        slope_df (pd.DataFrame): 傾きを含むデータフレーム。
    
    Returns:
        seaborn.axisgrid.FacetGrid: 作成したファセットグリッドオブジェクト。
    """
    # プロットのスタイルを設定
    sns.set(style="whitegrid")
    
    # プロットのファセットを設定
    g = sns.FacetGrid(
        df,
        row='distance',
        col='tilt_angle',
        hue='prop_spacing',
        palette="Set2",
        margin_titles=True,
        height=2,      # 縦のサイズを適切に設定
        aspect=1.2,    # アスペクト比を調整
        sharex=True,
        sharey=True
    )
    
    # エラーバー付き散布図を追加
    g.map_dataframe(
        sns.scatterplot,
        x='target_thrust',
        y=f'{col}',
        alpha=0.7
    )

    # 凡例を追加
    g.add_legend(title='Prop Spacing')
    g.tight_layout()
    
    # 各サブプロットに回帰線を追加
    g.map_dataframe(
        sns.regplot,
        x='target_thrust',
        y=f'{col}',
        scatter=False,
        label='_nolegend_',
        ci=None,
        color='gray',
        line_kws={'linewidth':1}
    )
    
    # グループごとの統計情報を辞書にまとめる
    stats_dict = correlation_df.set_index(['distance', 'tilt_angle', 'prop_spacing']).to_dict('index')
    slopes_dict = slope_df.set_index(['distance', 'tilt_angle', 'prop_spacing']).to_dict('index')
    
    # 各サブプロットにエラーバーとタイトルの統計情報を追加
    for distance in g.row_names:
        for tilt_angle in g.col_names:
            ax = g.axes[g.row_names.index(distance), g.col_names.index(tilt_angle)]
            title_lines = [f"Distance: {distance}, Tilt Angle: {tilt_angle}"]
            for prop_spacing in sorted(df['prop_spacing'].unique()):
                group_key = (distance, tilt_angle, prop_spacing)
                if group_key in stats_dict:
                    corr = stats_dict[group_key]['correlation']
                    slope = slopes_dict[group_key]['slope'] if group_key in slopes_dict else np.nan
                    corr_text = f"Prop Spacing: {prop_spacing} - r = {corr:.2f}" if not pd.isna(corr) else f"Prop Spacing: {prop_spacing} - r = N/A"
                    slope_text = f"slope = {slope:.2f}" if not pd.isna(slope) else "slope = N/A"
                    title_lines.append(f"{corr_text}, {slope_text}")
                    
                    # エラーバーを追加
                    group = df[
                        (df['distance'] == distance) &
                        (df['tilt_angle'] == tilt_angle) &
                        (df['prop_spacing'] == prop_spacing)
                    ]
                    
                    agg_df = group.groupby('target_thrust').agg(
                        mean_y=(f'offset_{col}', 'mean'),
                        std_y=(f'std_{col}', 'mean')  # 標準偏差の平均値を使用
                    ).reset_index()
                    
                    ax.errorbar(
                        agg_df['target_thrust'],
                        agg_df['mean_y'],
                        yerr=agg_df['std_y'],
                        fmt='none',
                        ecolor='lightgray',
                        elinewidth=1,
                        capsize=3
                    )

            # タイトルに相関係数と傾きを追加
            #new_title = "\n".join(title_lines)
            #ax.set_title(new_title, fontsize=10)
            
    
    # グラフのラベルを設定
    g.set_axis_labels('Target Thrust', f'{col}')
    
    return g

def calc_mean_and_std_error(g, col):
    # gは特定の(distance, tilt_angle, prop_spacing)などの条件で絞られたDataFrame
    offsets = g[f'offset_{col}'].values
    stds = g[f'std_{col}'].values
    counts = g['sample_count'].values

    print(f"offsets: {offsets}, stds: {stds}, counts: {counts}")
    
    total_count = counts.sum()
    weights = counts / total_count  # 加重平均の重み
    
    # 加重平均
    weighted_mean = np.sum(offsets * weights)
    
    # 各行の平均値の分散: var_i = (std_i^2) / n_i
    variances = (stds**2) / counts
    
    # 加重平均の分散: Σ(w_i^2 * var_i)
    weighted_var = np.sum(weights**2 * variances)
    std_error = np.sqrt(weighted_var)

    weighted_mean = offsets.mean()
    std_error = stds.mean()

    print(f"weighted_mean: {weighted_mean}, std_error: {std_error}")

    return pd.Series({'mean_offset': weighted_mean, 'std_offset': std_error})


def plot_distance_vs_offset_grid(df, col, col_unit, col_display):
    unique_tilt_angles = sorted(df['tilt_angle'].unique())
    unique_prop_spacings = sorted(df['prop_spacing'].unique())

    # 回帰直線用の色とマーカーを設定
    total_conditions = len(unique_tilt_angles) * len(unique_prop_spacings)
    palette = sns.color_palette("tab10", n_colors=total_conditions)
    markers = ['o', 's', 'D', '^', 'v', 'P', '*', 'X', 'h', 'd']

    num_tilt_angles = len(unique_tilt_angles) + 1
    if num_tilt_angles <= 2:
        nrows, ncols = 1, 2
    elif num_tilt_angles <= 4:
        nrows, ncols = 2, 2
    elif num_tilt_angles <= 6:
        nrows, ncols = 2, 3
    else:
        nrows = 3
        ncols = (num_tilt_angles + 2) // 3  # ceil(num_tilt_angles / 3)

    fig, axes = plt.subplots(nrows, ncols, figsize=(7*ncols, 9*nrows)) # サイズ調整
    axes = axes.flatten()

    # 不要なグラフを削除
    for i in range(num_tilt_angles, len(axes)):
        fig.delaxes(axes[i])
    # axes = axes[:num_tilt_angles] #必要な要素数だけに絞る

    # 最後のサブプロットを作成
    if num_tilt_angles < len(axes): # グラフが余っているなら最後のサブプロットを使用
        ax_all = axes[-1]
    else: # グラフが足りないなら新規に作成
        fig, ax_all = plt.subplots(1, 1, figsize=(7, 9))

    condition_idx = 0  # カラーパレットのインデックス

    # 各 tilt_angle ごとにプロット
    for idx, tilt_angle in enumerate(unique_tilt_angles):
        ax = axes[idx]
        for prop_spacing in unique_prop_spacings:
            group = df[
                (df['tilt_angle'] == tilt_angle) &
                (df['prop_spacing'] == prop_spacing)
            ]
            if group.empty:
                print(f"警告: 条件 (tilt_angle={tilt_angle}, prop_spacing={prop_spacing}) に該当するデータがありません。")
                continue

            # 各distanceごとに加重平均と正しいエラーバーを計算
            agg_df = group.groupby('distance').apply(lambda g: calc_mean_and_std_error(g, col)).reset_index()

            # マーカーのインデックス
            try:
                marker_idx = unique_prop_spacings.index(prop_spacing) % len(markers)
            except ValueError:
                marker_idx = 0

            # 散布図
            sns.scatterplot(
                data=agg_df,
                x='distance',
                y='mean_offset',
                label=f'L={prop_spacing}R',
                color=palette[condition_idx % len(palette)],
                marker=markers[marker_idx],
                s=100,
                edgecolor='w',
                ax=ax
            )

            # エラーバー
            ax.errorbar(
                agg_df['distance'],
                agg_df['mean_offset'],
                yerr=agg_df['std_offset'],
                fmt='none',
                ecolor=palette[condition_idx % len(palette)],
                elinewidth=1,
                capsize=3
            )

            # 回帰直線
            X = agg_df['distance'].values.reshape(-1, 1)
            y_vals = agg_df['mean_offset'].values
            if len(X) > 1 and np.std(X) > 0:
                model = LinearRegression(fit_intercept=False)
                model.fit(5.0 - X, y_vals)
                slope = model.coef_[0]
                y_pred = model.predict(5.0 - X)

                linestyle = "--"
                if prop_spacing == 3.7:
                    linestyle = "-"

                ax.plot(agg_df['distance'], y_pred, color=palette[condition_idx % len(palette)], linestyle=linestyle, linewidth=2)
                ax.set_ylim(-0.3, 0.3)
                ax.text(
                    agg_df['distance'].max(),
                    y_pred[-1],
                    f"slope={slope:.2f}",
                    color=palette[condition_idx % len(palette)],
                    fontsize=9,
                    verticalalignment='bottom',
                    horizontalalignment='right'
                )
            else:
                print(f"警告: 条件 (tilt_angle={tilt_angle}, prop_spacing={prop_spacing}) のデータが不十分で回帰直線を描画できません。")

            condition_idx += 1

        ax.set_title(f'Tilt Angle: {tilt_angle}°', fontsize=16)
        ax.set_xlabel('Distance [R]', fontsize=14)
        ax.set_ylabel(f'Average {col_display} [{col_unit}]', fontsize=14)
        ax.legend(title='Prop Spacing')

    # 最後のサブプロットに全条件の回帰直線のみをプロット
    condition_idx = 0
    for tilt_angle in unique_tilt_angles:
        for prop_spacing in unique_prop_spacings:
            group = df[
                (df['tilt_angle'] == tilt_angle) &
                (df['prop_spacing'] == prop_spacing)
            ]
            if group.empty:
                print(f"警告: 条件 (tilt_angle={tilt_angle}, prop_spacing={prop_spacing}) に該当するデータがありません。")
                continue

            agg_df = group.groupby('distance').apply(lambda g: calc_mean_and_std_error(g, col)).reset_index()

            X = agg_df['distance'].values.reshape(-1, 1)
            y_vals = agg_df['mean_offset'].values
            if len(X) > 1 and np.std(X) > 0:
                model = LinearRegression(fit_intercept=False)
                model.fit(5.0 - X, y_vals)
                slope = model.coef_[0]
                y_pred = model.predict(5.0 - X)
                print(f"slope: {slope}, palette index: {condition_idx}")

                color = palette[condition_idx % len(palette)]
                label = f'θ={tilt_angle}°,L={prop_spacing}R'

                linestyle = "--"
                if prop_spacing == 3.7:
                    linestyle = "-"

                ax_all.plot(agg_df['distance'], y_pred, color=color, linestyle=linestyle, linewidth=2, label=label)
                ax_all.set_ylim(-0.3, 0.3)
                ax_all.text(
                    agg_df['distance'].max(),
                    y_pred[-1],
                    f"slope={slope:.2f}",
                    color=color,
                    fontsize=9,
                    verticalalignment='bottom',
                    horizontalalignment='right'
                )
            else:
                print(f"警告: 条件 (tilt_angle={tilt_angle}, prop_spacing={prop_spacing}) のデータが不十分で回帰直線を描画できません。")

            condition_idx += 1

    ax_all.set_title('Regression Lines', fontsize=16)
    ax_all.set_xlabel('Distance', fontsize=14)
    ax_all.set_ylabel(f'Average {col_display} [{col_unit}]', fontsize=14)
    ax_all.legend(ncol=1)

    plt.tight_layout()
    plt.show()


def plot_heatmaps(correlation_df, slope_df):
    """
    相関係数と回帰係数のヒートマップを別ウィンドウで表示します。
    
    Parameters:
        correlation_df (pd.DataFrame): 相関係数を含むデータフレーム。
        slope_df (pd.DataFrame): 回帰係数を含むデータフレーム。
    """
    # 相関係数のピボットテーブル作成
    corr_pivot = correlation_df.pivot_table(
        index=['tilt_angle', 'prop_spacing'],
        columns='distance',
        values='correlation',
        aggfunc='mean'
    )
    
    # 回帰係数のピボットテーブル作成
    slope_pivot = slope_df.pivot_table(
        index=['tilt_angle', 'prop_spacing'],
        columns='distance',
        values='slope',
        aggfunc='mean'
    )
    
    # ヒートマップの作成
    fig, axes = plt.subplots(1, 2, figsize=(20, 12))
    
    sns.heatmap(corr_pivot, annot=True, fmt=".2f", cmap="coolwarm", ax=axes[0], cbar_kws={'label': 'Correlation Coefficient'})
    axes[0].set_title('Average Correlation Coefficient', fontsize=16)
    axes[0].set_xlabel('Distance')
    axes[0].set_ylabel('Tilt Angle & Prop Spacing')
    
    sns.heatmap(slope_pivot, annot=True, fmt=".2f", cmap="viridis", ax=axes[1], cbar_kws={'label': 'Regression Slope'})
    axes[1].set_title('Average Regression Slope', fontsize=16)
    axes[1].set_xlabel('Distance')
    axes[1].set_ylabel('Tilt Angle & Prop Spacing')
    
    plt.tight_layout()
    plt.show()

def main():
    # コマンドライン引数の解析
    args = parse_arguments()
    
    # CSVファイルの読み込みと検証
    df = read_and_validate_csv(args.input)
    
    # データの処理
    df = process_data(df, args.col, args.distance_ref, invert_col=args.invert_col)
    
    # 相関係数と回帰直線の傾きを計算
    correlation_df = calculate_correlations(df, args.col)
    slope_df = calculate_regression_slopes(df, args.col)
    
    # ファセットプロットの作成
    g = plot_facetgrid(df, args.col, args.col_unit, correlation_df, slope_df)
    
    # ファセットプロットを保存
    if args.output:
        try:
            g.fig.savefig(args.output, dpi=300, bbox_inches='tight')
            print(f"ファセットプロットを '{args.output}' に保存しました。")
        except Exception as e:
            print(f"エラー: ファセットプロットの保存に失敗しました。\n詳細: {e}")
    
    # 相関係数と回帰係数のヒートマップを表示
    #plot_heatmaps(correlation_df, slope_df)
    
    # distance_vs_offset_combined のプロットを2x2グリッドで作成
    plot_distance_vs_offset_grid(df, args.col, args.col_unit, args.col_display)

if __name__ == "__main__":
    main()

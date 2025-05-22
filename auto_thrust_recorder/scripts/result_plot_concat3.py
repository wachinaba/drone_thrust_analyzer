import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルの読み込み
try:
    df = pd.read_csv("concat.csv")
except FileNotFoundError:
    print("エラー: concat.csv が見つかりません。スクリプトと同じディレクトリにファイルを配置してください。")
    exit()
except Exception as e:
    print(f"CSVファイルの読み込み中にエラーが発生しました: {e}")
    exit()

# 関連する列を数値型に変換（変換できない値はNaNになります）
cols_to_numeric = ['target_thrust', 'sample_count', 'control', 'force_x', 'force_y', 'force_z',
                   'torque_x', 'torque_y', 'torque_z', 'variance_force_x', 'variance_force_y',
                   'variance_force_z', 'variance_torque_x', 'variance_torque_y', 'variance_torque_z',
                   'distance', 'tilt_angle', 'fold_angle', 'prop_spacing', 'height']

for col in cols_to_numeric:
    if col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')

# グループ化のためのキー
grouping_keys = ['tilt_angle', 'fold_angle', 'prop_spacing', 'keyword', 'height']

# 指定されたキーでデータフレームをグループ化
grouped_data = df.groupby(grouping_keys)

if grouped_data.ngroups == 0:
    print("指定されたキーに基づくグループが見つかりませんでした。CSVファイルとキーを確認してください。")
    exit()

# 各グループに対してプロット処理
for name, group_df in grouped_data:
    tilt_angle, fold_angle, prop_spacing, keyword, height = name

    # グループ内で 'distance' のユニークな値を取得しソート
    unique_distances = sorted(group_df['distance'].unique())
    num_distances = len(unique_distances)

    if num_distances == 0:
        print(f"グループ {name} にはユニークな 'distance' の値がありません。")
        continue

    # サブプロットの行数と列数
    ncols = min(7, num_distances)
    nrows = (num_distances + ncols - 1) // ncols  # 切り上げ除算

    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(ncols * 2.5, nrows * 4), squeeze=False)
    axes_flat = axes.flatten()  # サブプロットを1次元配列で扱う

    # 図全体のタイトル
    fig_title = (f"Tilt: {tilt_angle}, Fold: {fold_angle}, Prop Spacing: {prop_spacing}, Keyword: {keyword}, Height: {height}\n"
                 f"Target Thrust vs Torque X (Grouped by Distance)")
    fig.suptitle(fig_title, fontsize=14)

    for i, dist_val in enumerate(unique_distances):
        ax = axes_flat[i]
        
        # 現在の 'distance' に対応するサブデータフレームを取得
        # SettingWithCopyWarningを避けるために .copy() を使用
        sub_group_df = group_df[group_df['distance'] == dist_val].copy()

        # 'target_thrust' と 'torque_x' の NaN を含む行を削除
        sub_group_df.dropna(subset=['target_thrust', 'torque_x'], inplace=True)

        if sub_group_df.empty:
            ax.set_title(f"Distance: {dist_val}\n(有効なデータなし)", fontsize=10)
            ax.text(0.5, 0.5, "有効なデータなし", ha='center', va='center', fontsize=10, color='gray')
            ax.set_xlabel("Target Thrust (N)", fontsize=9)
            ax.set_ylabel("Torque X (Nm)", fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.7)
            continue

        x_data = sub_group_df['target_thrust']
        y_data = sub_group_df['torque_x']

        # 散布図をプロット
        ax.scatter(x_data, y_data, label='data', alpha=0.6, s=30)
        ax.set_xlim(10, 25)
        ax.set_ylim(-0.5, 0.5)

        # 近似直線（一次式）を計算してプロット
        if len(x_data) >= 2:  # 近似には少なくとも2点が必要
            try:
                # numpy.polyfitで一次近似
                coeffs = np.polyfit(x_data, y_data, 1)
                poly_fn = np.poly1d(coeffs) # 係数から多項式関数を作成

                # 近似直線をプロットするためのx値の範囲を生成
                # x_dataの最小値と最大値から範囲を設定
                if x_data.nunique() == 1: # xが全て同じ値の場合
                    x_plot_min = x_data.min() - 0.5 # 少し範囲を広げる
                    x_plot_max = x_data.max() + 0.5
                else:
                    x_plot_min = x_data.min()
                    x_plot_max = x_data.max()
                
                x_fit = np.linspace(x_plot_min, x_plot_max, 100)
                y_fit = poly_fn(x_fit)
                
                ax.plot(x_fit, y_fit, color='red', linestyle='--', linewidth=2, 
                        label=f'fit line: y={coeffs[0]:.3f}x + {coeffs[1]:.3f}')
                ax.legend(fontsize=8)
            except (np.linalg.LinAlgError, TypeError) as e:
                print(f"グループ {name}, Distance {dist_val} で近似直線の計算に失敗しました: {e}")
                ax.text(0.5, 0.4, "近似失敗", ha='center', va='center', fontsize=9, color='orange')
        else:
            ax.text(0.5, 0.4, "近似データ不足 (<2点)", ha='center', va='center', fontsize=9, color='gray')

        ax.set_title(f"Distance: {dist_val}", fontsize=10)
        ax.set_xlabel("Target Thrust (N)", fontsize=9)
        ax.set_ylabel("Torque X (Nm)", fontsize=9)
        ax.tick_params(axis='both', which='major', labelsize=8)
        ax.grid(True, linestyle=':', alpha=0.7)

    # 使用しなかった余分なサブプロットを非表示にする
    for j in range(num_distances, nrows * ncols):
        fig.delaxes(axes_flat[j])

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # suptitleとの重なりを避けるため調整
    plt.show()

print("すべてのプロット処理が完了しました。")
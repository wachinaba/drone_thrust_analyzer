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

# keywordごとのグループ数を取得
unique_keywords = df['keyword'].unique()
num_keywords = len(unique_keywords)

# 各keywordのグループ数を取得
keyword_groups = {}
for keyword in unique_keywords:
    keyword_df = df[df['keyword'] == keyword]
    keyword_groups[keyword] = len(keyword_df.groupby(['tilt_angle', 'fold_angle', 'prop_spacing', 'height']))

# 結合データのグループ数を取得
combined_groups = len(df.groupby(['tilt_angle', 'fold_angle', 'prop_spacing', 'height']))

# 最大のグループ数を取得
max_groups = max(max(keyword_groups.values()), combined_groups)

# 全体のプロットを1つのウィンドウに表示
fig = plt.figure(figsize=(20, 5 * (num_keywords + 1)))
fig.suptitle("Target Thrust vs Torque X (Grouped by Distance and Keyword)", fontsize=16, y=0.95)

# グリッドの設定
gs = fig.add_gridspec(num_keywords + 1, max_groups * 7, hspace=0.4, wspace=0.3)

# keywordごとにプロット処理
for keyword_idx, keyword in enumerate(unique_keywords):
    # 現在のkeywordに対応するデータのみを抽出
    keyword_df = df[df['keyword'] == keyword]
    
    # 残りのキーでグループ化
    sub_grouped_data = keyword_df.groupby(['tilt_angle', 'fold_angle', 'prop_spacing', 'height'])
    
    # 各グループに対してプロット処理
    for group_idx, (name, group_df) in enumerate(sub_grouped_data):
        tilt_angle, fold_angle, prop_spacing, height = name

        # グループ内で 'distance' のユニークな値を取得しソート
        unique_distances = sorted(group_df['distance'].unique())
        num_distances = len(unique_distances)

        if num_distances == 0:
            print(f"グループ {name} にはユニークな 'distance' の値がありません。")
            continue

        # サブプロットの列数（1行に固定）
        ncols = min(7, num_distances)  # 最大7列まで

        # サブプロットを作成
        for i, dist_val in enumerate(unique_distances):
            # グリッドの位置を計算
            start_col = group_idx * 7 + i
            ax = fig.add_subplot(gs[keyword_idx, start_col])
            
            # 現在の 'distance' に対応するサブデータフレームを取得
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
                    coeffs = np.polyfit(x_data, y_data, 1)
                    poly_fn = np.poly1d(coeffs)

                    if x_data.nunique() == 1:
                        x_plot_min = x_data.min() - 0.5
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

            # サブプロットのタイトルとラベル
            if i == 0:  # 最初のサブプロットにのみグループ情報を表示
                ax.set_title(f"Keyword: {keyword}\nTilt: {tilt_angle}, Fold: {fold_angle}\n"
                           f"Prop Spacing: {prop_spacing}, Height: {height}\n"
                           f"Distance: {dist_val}", fontsize=10)
            else:
                ax.set_title(f"Distance: {dist_val}", fontsize=10)
            
            ax.set_xlabel("Target Thrust (N)", fontsize=9)
            ax.set_ylabel("Torque X (Nm)", fontsize=9)
            ax.tick_params(axis='both', which='major', labelsize=8)
            ax.grid(True, linestyle=':', alpha=0.7)

# 最後の行に全てのキーワードを結合したプロットを追加
last_row = num_keywords
combined_grouped_data = df.groupby(['tilt_angle', 'fold_angle', 'prop_spacing', 'height'])

for group_idx, (name, group_df) in enumerate(combined_grouped_data):
    tilt_angle, fold_angle, prop_spacing, height = name

    # グループ内で 'distance' のユニークな値を取得しソート
    unique_distances = sorted(group_df['distance'].unique())
    num_distances = len(unique_distances)

    if num_distances == 0:
        print(f"結合グループ {name} にはユニークな 'distance' の値がありません。")
        continue

    # サブプロットの列数（1行に固定）
    ncols = min(7, num_distances)  # 最大7列まで

    # サブプロットを作成
    for i, dist_val in enumerate(unique_distances):
        # グリッドの位置を計算
        start_col = group_idx * 7 + i
        ax = fig.add_subplot(gs[last_row, start_col])
        
        # 現在の 'distance' に対応するサブデータフレームを取得
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

        # target_thrustでグループ化して平均を計算
        averaged_data = sub_group_df.groupby('target_thrust').agg({
            'torque_x': ['mean', 'std', 'count']
        }).reset_index()
        
        # カラム名を整理
        averaged_data.columns = ['target_thrust', 'torque_x_mean', 'torque_x_std', 'count']
        
        # データポイントが2つ以上ある場合のみプロット
        if len(averaged_data) >= 2:
            # 平均値のプロット
            ax.scatter(averaged_data['target_thrust'], 
                      averaged_data['torque_x_mean'],
                      label='averaged data', 
                      color='blue',
                      alpha=0.8, 
                      s=50)
            
            # 標準偏差をエラーバーとして表示
            ax.errorbar(averaged_data['target_thrust'],
                       averaged_data['torque_x_mean'],
                       yerr=averaged_data['torque_x_std'],
                       fmt='none',
                       color='blue',
                       alpha=0.3,
                       capsize=3)

            # データポイントの数を表示
            for _, row in averaged_data.iterrows():
                ax.annotate(f'n={int(row["count"])}',
                          (row['target_thrust'], row['torque_x_mean']),
                          xytext=(0, 10),
                          textcoords='offset points',
                          ha='center',
                          fontsize=8)

            # 近似直線の計算とプロット
            try:
                coeffs = np.polyfit(averaged_data['target_thrust'], 
                                  averaged_data['torque_x_mean'], 
                                  1)
                poly_fn = np.poly1d(coeffs)

                x_fit = np.linspace(averaged_data['target_thrust'].min(),
                                  averaged_data['target_thrust'].max(),
                                  100)
                y_fit = poly_fn(x_fit)
                
                ax.plot(x_fit, y_fit, 
                       color='red', 
                       linestyle='--', 
                       linewidth=2,
                       label=f'fit line: y={coeffs[0]:.3f}x + {coeffs[1]:.3f}')
                
                # 決定係数（R²）の計算
                y_pred = poly_fn(averaged_data['target_thrust'])
                r2 = 1 - (np.sum((averaged_data['torque_x_mean'] - y_pred) ** 2) / 
                         np.sum((averaged_data['torque_x_mean'] - averaged_data['torque_x_mean'].mean()) ** 2))
                
                ax.text(0.05, 0.95, 
                       f'R² = {r2:.3f}', 
                       transform=ax.transAxes,
                       fontsize=8,
                       verticalalignment='top')
                
            except (np.linalg.LinAlgError, TypeError) as e:
                print(f"結合グループ {name}, Distance {dist_val} で近似直線の計算に失敗しました: {e}")
                ax.text(0.5, 0.4, "近似失敗", ha='center', va='center', fontsize=9, color='orange')
        else:
            ax.text(0.5, 0.4, "データ不足 (<2点)", ha='center', va='center', fontsize=9, color='gray')

        ax.set_xlim(10, 25)
        ax.set_ylim(-0.5, 0.5)

        # サブプロットのタイトルとラベル
        if i == 0:  # 最初のサブプロットにのみグループ情報を表示
            ax.set_title(f"Averaged Data\nTilt: {tilt_angle}, Fold: {fold_angle}\n"
                       f"Prop Spacing: {prop_spacing}, Height: {height}\n"
                       f"Distance: {dist_val}", fontsize=10)
        else:
            ax.set_title(f"Distance: {dist_val}", fontsize=10)
        
        ax.set_xlabel("Target Thrust (N)", fontsize=9)
        ax.set_ylabel("Torque X (Nm)", fontsize=9)
        ax.tick_params(axis='both', which='major', labelsize=8)
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(fontsize=8)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # suptitleとの重なりを避けるため調整
plt.show()

print("すべてのプロット処理が完了しました。")
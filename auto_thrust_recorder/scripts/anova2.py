import argparse
import pandas as pd
import numpy as np
import statsmodels.api as sm
from statsmodels.formula.api import ols
from tqdm import tqdm

def main():
    parser = argparse.ArgumentParser(description='Monte Carlo bootstrap to estimate multiple k samples for 2-way ANOVA.')
    parser.add_argument('--input', '-i', required=True, help='Input CSV file with distance, tilt_angle, prop_spacing, torque_x, variance_torque_x.')
    parser.add_argument('--output', '-o', default='mc_anova_result.txt', help='Output file to save ANOVA results.')
    parser.add_argument('--iterations', '-n', type=int, default=1000, help='Number of bootstrap iterations.')
    args = parser.parse_args()

    #=== データ読み込み ===
    df = pd.read_csv(args.input)
    # tilt_angle, prop_spacingをカテゴリとして扱う
    df['tilt_angle'] = df['tilt_angle'].astype('category')
    df['prop_spacing'] = df['prop_spacing'].astype('category')

    # 必要なカラム存在チェック
    for col in ['distance', 'tilt_angle', 'prop_spacing', 'torque_x', 'variance_torque_x']:
        if col not in df.columns:
            raise ValueError(f"{col} not found in the input data.")

    if (df['variance_torque_x'] <= 0).any():
        raise ValueError("All variance_torque_x must be > 0.")

    # Monte Carlo ループでkを推定
    results = []  # ここに各iterationで得たk値を蓄積

    # (tilt_angle, prop_spacing)を区別するためのグループ分け
    groups = df.groupby(['tilt_angle', 'prop_spacing'])

    # イテレーションを回す
    for iteration in tqdm(range(args.iterations)):
        # 元データから正規乱数を生成
        # mean = torque_x, var = variance_torque_x
        sampled_torque = np.random.normal(loc=df['torque_x'].values, scale=np.sqrt(df['variance_torque_x'].values))

        # sampled_torqueをdfに追加してこのiterationの仮想データセットにする
        df_sampled = df.copy()
        df_sampled['torque_x_sampled'] = sampled_torque

        # 各(tilt_angle, prop_spacing)ごとにkを求める
        # kは t = k(5 - d) より、WLSでフィット
        # 重み = 1/variance_torque_x
        # 切片なし回帰を想定。X = (5 - d)
        for (ta, ps), subgroup in df_sampled.groupby(['tilt_angle', 'prop_spacing']):
            # WLSフィット
            # y = torque_x_sampled, X = (5 - d)
            # 重み = 1/variance
            if len(subgroup) < 2:
                # データ点が1点だと回帰不可能なのでスキップ
                continue

            X = (5 - subgroup['distance'].values)
            y = subgroup['torque_x_sampled'].values
            w = 1.0 / subgroup['variance_torque_x'].values

            # 切片なしモデル
            # OLSで切片なし: sm.WLS(y, X[:, None], weights=w)
            X_design = X[:, None]  # 2D化
            wls_model = sm.WLS(y, X_design, weights=w)
            wls_res = wls_model.fit()
            k = wls_res.params[0]

            results.append({
                'iteration': iteration,
                'tilt_angle': ta,
                'prop_spacing': ps,
                'k': k
            })

    df_k = pd.DataFrame(results)
    # これで各iteration × 各(tilt_angle, prop_spacing)についてkが多数得られた

    # 2-way ANOVAを行う前に、tilt_angle, prop_spacingをカテゴリ化（再確認）
    df_k['tilt_angle'] = df_k['tilt_angle'].astype('category')
    df_k['prop_spacing'] = df_k['prop_spacing'].astype('category')

    # 2-way ANOVA (kを応答変数、tilt_angleとprop_spacingを因子)
    # 複数iterationでkが得られてるので、ANOVA可能
    model = ols('k ~ C(tilt_angle)*C(prop_spacing)', data=df_k).fit()
    anova_res = sm.stats.anova_lm(model, typ=2)

    print("ANOVA result for k from Monte Carlo approach:")
    print(anova_res)

    with open(args.output, 'w') as f:
        f.write("ANOVA result for k from Monte Carlo approach:\n")
        f.write(str(anova_res)+"\n\n")

    # 必要ならpost-hocテスト(Tukeyなど)も可能
    # 例: pingouinを使う場合（インストール済みを想定）
    # from pingouin import pairwise_tukey
    # tukey_tilt = pairwise_tukey(dv='k', between='tilt_angle', data=df_k)
    # f.write("Tukey tilt_angle:\n" + str(tukey_tilt) + "\n\n")

    # tukey_prop = pairwise_tukey(dv='k', between='prop_spacing', data=df_k)
    # f.write("Tukey prop_spacing:\n" + str(tukey_prop) + "\n\n")

if __name__ == '__main__':
    main()

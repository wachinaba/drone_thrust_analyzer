import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def parse_args():
    parser = argparse.ArgumentParser(description='Visualize CSV data with polynomial approximation and error bars using Pandas, color-coded by target_thrust.')
    parser.add_argument('csv_file', type=Path, help='Path to the CSV file.')
    parser.add_argument('--degree', type=int, default=1, help='Degree of the polynomial approximation.')
    return parser.parse_args()


def polynomial_with_derivative_constraint(x, y, degree, derivative_zero_at=2.5):
    """
    点(x, y)に対して、指定された点derivative_zero_atで導関数が0になる制約付き多項式近似を行う関数。
    点を通る制約は廃止.
    """
    def objective(coeffs):
        # 目的関数: データ点と多項式との二乗誤差
        y_predicted = np.polyval(coeffs, x)
        return np.sum((y - y_predicted)**2)

    def constraint_derivative(coeffs):
         # 制約条件: 指定された点での導関数が0
         poly = np.poly1d(coeffs)
         derivative = poly.deriv()
         return derivative(derivative_zero_at)
    # 初期値: ランダムな係数
    initial_coeffs = np.random.randn(degree + 1)

    # 制約条件の定義
    cons = ({'type': 'eq', 'fun': constraint_derivative},)  # 等式制約

    # 最適化
    res = minimize(objective, initial_coeffs, constraints=cons, method='SLSQP')  # SLSQP for constrained problems

    # 最適化された係数
    return res.x, res.success


def plot_group(group_name, group_data, degree, ax, target_thrust_colors):
    """Plots the data for a single group and its polynomial approximation with error bars, color-coded by target_thrust."""
    distance = group_data['distance']
    torque_x = group_data['torque_x']
    target_thrust = group_data['target_thrust']

    # Calculate standard deviation from variances.  Assumes sample_count is consistent within the group.
    sample_count = group_data['sample_count'].iloc[0]  # Take only the first value
    variance_torque_x = group_data['variance_torque_x']
    std_dev = np.sqrt(variance_torque_x)  # Standard error = std_dev / sqrt(n)

    # リストに格納
    handles = []
    labels = []

    dot_styles = {
        2.7: 'o',
        3.7: 'x',
    }
    line_styles = {
        2.7: '-',
        3.7: '--',
    }

    # Iterate through unique target_thrust values within the group
    for thrust_value in target_thrust.unique():
        thrust_data = group_data[group_data['target_thrust'] == thrust_value]
        thrust_distance = thrust_data['distance']
        thrust_torque_x = thrust_data['torque_x']
        thrust_std_dev = std_dev[group_data['target_thrust'] == thrust_value] #Corrected indexing

        color = target_thrust_colors[thrust_value]
        errorbar_plot = ax.errorbar(thrust_distance, thrust_torque_x, yerr=thrust_std_dev, fmt=dot_styles[float(group_name[2])], label=f'{group_name} (Thrust: {thrust_value})', capsize=5, color=color)
        handles.append(errorbar_plot)
        labels.append(f'{group_name} (Thrust: {float(thrust_value):.2f})')


         # Polynomial approximation WITH constraint
        try:  # Handle potential optimization errors
            coeffs, success = polynomial_with_derivative_constraint(thrust_distance, thrust_torque_x, degree)  # Pass only target_thrust-specific data
            if success:
                p = np.poly1d(coeffs)
                distance_range = np.linspace(min(thrust_distance), max(thrust_distance), 100)  # More points for smoother curve
                poly_plot = ax.plot(distance_range, p(distance_range), line_styles[float(group_name[2])], label=f'{group_name} (Thrust: {thrust_value}) Constrained Poly fit (deg {degree})', color=color)[0] #keep color same

                #Highlight the point where the derivative is zero.
                #ax.scatter(5.0,p(5.0), color='red', label='Derivative Zero Point') #Disable this plot
            else:
                print(f"Optimization failed for group {group_name}, target_thrust {thrust_value}")

        except Exception as e:
            print(f"Error during optimization for group {group_name}, target_thrust {thrust_value}: {e}")

    ax.set_xlabel('Distance')
    ax.set_ylabel('Torque_x')
    ax.set_title(f'Tilt Angle: {int(group_name[0])}')
    ax.legend(handles, labels)


def main():
    # Set the background color to white
    plt.rcParams['axes.facecolor'] = '#F7FFFB' #F7FFFB

    args = parse_args()
    csv_file = args.csv_file
    degree = args.degree

    # Load data with Pandas
    df = pd.read_csv(csv_file)

    # Rename columns
    df = df.rename(columns={
        'target_thrust': 'target_thrust',
        'sample_count': 'sample_count',
        'control': 'control',
        'force_x': 'force_x',
        'force_y': 'force_y',
        'force_z': 'force_z',
        'torque_x': 'torque_x',
        'torque_y': 'torque_y',
        'torque_z': 'torque_z',
        'variance_force_x': 'variance_force_x',
        'variance_force_y': 'variance_force_y',
        'variance_force_z': 'variance_force_z',
        'variance_torque_x': 'variance_torque_x',
        'variance_torque_y': 'variance_torque_y',
        'variance_torque_z': 'variance_torque_z',
        'distance': 'distance',
        'tilt_angle': 'tilt_angle',
        'fold_angle': 'fold_angle',
        'prop_spacing': 'prop_spacing',
        'keyword': 'keyword'
    })

    # Grouping using Pandas
    grouped = df.groupby(['tilt_angle', 'fold_angle', 'prop_spacing'])

    # Define color mapping for target_thrust values
    unique_thrusts = df['target_thrust'].unique()
    target_thrust_colors = {thrust: plt.cm.magma(i/len(unique_thrusts)) for i, thrust in enumerate(unique_thrusts)}


    # Plotting separate plots for each group
    num_groups = len(grouped)
    fig, axes = plt.subplots(1, num_groups, figsize=(5 * num_groups, 5), sharex=True, sharey=True, facecolor='#F7FFFB')
    if num_groups == 1:
        axes = [axes] # Make iterable even if it's a single subplot

    group_names = list(grouped.groups.keys()) # Get the group names

    for i, group_name in enumerate(group_names):
        group_data = grouped.get_group(group_name)  # Access group data using Pandas
        plot_group(group_name, group_data, degree, axes[i], target_thrust_colors)


    fig.suptitle('Torque_x vs Distance by Group')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    # Combined Plot
    fig_combined, ax_combined = plt.subplots(figsize=(10, 6), facecolor='#F7FFFB')

    unique_tilt_angles = df['tilt_angle'].unique()
    tilt_angle_colors = {tilt_angle: plt.cm.viridis(i/len(unique_tilt_angles)) for i, tilt_angle in enumerate(unique_tilt_angles)}

    handles = []
    labels = []

    dot_styles = {
        2.7: 'o',
        3.7: 'x',
    }
    line_styles = {
        2.7: '-',
        3.7: '--',
    }

    for group_name, group_data in grouped:  # Iterate through the groups directly

        target_thrust = group_data['target_thrust']
        distance = group_data['distance']
        torque_x = group_data['torque_x']
        # Calculate standard deviation from variances (combined plot).
        sample_count = group_data['sample_count'].iloc[0]
        variance_torque_x = group_data['variance_torque_x']

        each_data = group_data.groupby('distance').mean(numeric_only=True)
        thrust_distance = each_data.index
        # distanceが一番大きなデータのtorque_xが0になるようにシフト
        # thrust_torque_x = each_data['torque_x'] - each_data['torque_x'].iloc[-1]
        thrust_torque_x = each_data['torque_x']
        thrust_std_dev = np.sqrt(each_data['variance_torque_x'])

        color = tilt_angle_colors[group_name[0]]

        handle = ax_combined.scatter(thrust_distance, thrust_torque_x, label=f'{group_name}', color=color, marker=dot_styles[float(group_name[2])])
        handles.append(handle)
        labels.append(f'Tilt Angle: {int(group_name[0])}, Fold Angle: {int(group_name[1])}, Prop Spacing: {float(group_name[2])}')

        try:
            coeffs, success = polynomial_with_derivative_constraint(thrust_distance, thrust_torque_x, degree)
            if success:
                p = np.poly1d(coeffs)
                distance_range = np.linspace(min(distance), max(distance), 100)
                ax_combined.plot(distance_range, p(distance_range), line_styles[float(group_name[2])], label=f'{group_name} Constrained Poly fit (deg {degree})', color=color)
            else:
                print(f"Optimization failed for group {group_name} in combined plot.")
        except Exception as e:
            print(f"Error during optimization for combined plot, group {group_name}: {e}")

    ax_combined.set_xlabel('Distance')
    ax_combined.set_ylabel('Torque_x')
    ax_combined.set_title('Combined Plot: Torque_x vs Distance (All Groups)')
    ax_combined.legend(handles, labels)
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()
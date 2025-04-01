import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def parse_args():
    parser = argparse.ArgumentParser(description='Visualize CSV data with polynomial approximation using Pandas.')
    parser.add_argument('csv_file', type=Path, help='Path to the CSV file.')
    parser.add_argument('--degree', type=int, default=1, help='Degree of the polynomial approximation.')
    return parser.parse_args()


def polynomial_with_derivative_constraint(x, y, degree, derivative_zero_at=5.0):
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

def plot_group(group_name, group_data, degree, ax):
    """Plots the data for a single group and its polynomial approximation."""
    distance = group_data['distance']
    torque_x = group_data['torque_x']

    ax.scatter(distance, torque_x, label=f'{group_name}')

    # Polynomial approximation WITH constraint
    try:  # Handle potential optimization errors
        coeffs, success = polynomial_with_derivative_constraint(distance, torque_x, degree)
        if success:
            p = np.poly1d(coeffs)
            distance_range = np.linspace(min(distance), max(distance), 100)  # More points for smoother curve
            ax.plot(distance_range, p(distance_range), '--', label=f'Constrained Poly fit (deg {degree})')

            #Highlight the point where the derivative is zero.
            ax.scatter(5.0,p(5.0), color='red', label='Derivative Zero Point')
        else:
             print(f"Optimization failed for group {group_name}")

    except Exception as e:
        print(f"Error during optimization for group {group_name}: {e}")

    ax.set_xlabel('Distance')
    ax.set_ylabel('Torque_x')
    ax.set_title(f'Group: {group_name}')
    ax.legend()


def main():
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


    # Plotting separate plots for each group
    num_groups = len(grouped)
    fig, axes = plt.subplots(1, num_groups, figsize=(5 * num_groups, 5), sharex=True, sharey=True)
    if num_groups == 1:
        axes = [axes] # Make iterable even if it's a single subplot

    group_names = list(grouped.groups.keys()) # Get the group names

    for i, group_name in enumerate(group_names):
        group_data = grouped.get_group(group_name)  # Access group data using Pandas
        plot_group(group_name, group_data, degree, axes[i])


    fig.suptitle('Torque_x vs Distance by Group')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()



    # Combined Plot
    fig_combined, ax_combined = plt.subplots(figsize=(10, 6))

    for group_name, group_data in grouped:  # Iterate through the groups directly
        distance = group_data['distance']
        torque_x = group_data['torque_x']
        ax_combined.scatter(distance, torque_x, label=f'{group_name}')
        try:
            coeffs, success = polynomial_with_derivative_constraint(distance, torque_x, degree)
            if success:
                 p = np.poly1d(coeffs)
                 distance_range = np.linspace(min(distance), max(distance), 100)
                 ax_combined.plot(distance_range, p(distance_range), '--', label=f'{group_name} Constrained Poly fit (deg {degree})')
            else:
                print(f"Optimization failed for group {group_name} in combined plot.")
        except Exception as e:
             print(f"Error during optimization for combined plot, group {group_name}: {e}")

    ax_combined.set_xlabel('Distance')
    ax_combined.set_ylabel('Torque_x')
    ax_combined.set_title('Combined Plot: Torque_x vs Distance (All Groups)')
    ax_combined.legend()
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()
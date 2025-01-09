import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Load the CSV file to get an overview of its contents
parser = argparse.ArgumentParser(description="Plotting force_y vs force_z and torque_x vs force_z")
parser.add_argument('--file', type=str, required=True, help='Path to the combined CSV file')
args = parser.parse_args()

file_path = args.file
df = pd.read_csv(file_path)

# Flip the sign of torque_x
df["torque_x"] = -df["torque_x"]

# Filter the data based on force_z range (10 to 25)
df = df[(df['force_z'] >= 10) & (df['force_z'] <= 25)]

# Sorting by tilt_angle for consistent coloring
df = df.sort_values(by='tilt_angle')

# Defining the color map for tilt_angle values
def get_color(tilt_angle):
    if tilt_angle == -30:
        return 'red'
    elif tilt_angle == -15:
        return 'darkred'
    elif tilt_angle == 0:
        return 'black'
    elif tilt_angle == 15:
        return 'mediumpurple'
    elif tilt_angle == 30:
        return 'blue'
    return 'gray'

# Plotting function
def plot_force_and_moment(df, distances, filename=None):
    plt.rcParams.update({'font.size': 12})
    fig, axs = plt.subplots(len(distances), 2, figsize=(12, 8), facecolor="#F4F3F7", sharex='col')

    for i, distance in enumerate(distances):
        subset = df[df['distance'] == distance]

        # Fw vs Fv (force_y vs force_z)
        ax = axs[i, 0]
        for tilt_angle in subset['tilt_angle'].unique():
            data = subset[subset['tilt_angle'] == tilt_angle].sort_values(by='force_z')
            color = get_color(tilt_angle)
            
            # Scatter plot
            """
            ax.scatter(
                data['force_z'], 
                data['force_y'], 
                edgecolors=color,
                alpha=0.5,
                facecolors='none', 
                marker='o',
                s=100
            )
            """
            
            # Moving average
            force_y_ma = pd.DataFrame(data['force_y'].rolling(window=20, center=True).mean())
            force_y_ma["force_z"] = data["force_z"]
            force_y_ma = force_y_ma.dropna().sort_values(by='force_z')
            ax.set_ylim(0.0, 0.8)
            ax.plot(
                force_y_ma['force_z'], 
                force_y_ma['force_y'], 
                color=color,
                label=f'θ={tilt_angle}' + f"{'(baseline)' if tilt_angle == 0 else ''}",
                linewidth=2
            )
            

        ax.set_ylabel(f'Fw (N) at d={distance}')
        ax.grid(True)
        if i == len(distances) - 1:
            ax.set_xlabel('Fv (N)')
        if i == 0:
            ax.legend(ncol=2)

        # Mw vs Fv (torque_x vs force_z)
        ax = axs[i, 1]
        for tilt_angle in subset['tilt_angle'].unique():
            data = subset[subset['tilt_angle'] == tilt_angle].sort_values(by='force_z')
            color = get_color(tilt_angle)

            # Scatter plot
            """
            ax.scatter(
                data['force_z'], 
                data['torque_x'], 
                edgecolors=color,
                alpha=0.5,
                facecolors='none', 
                marker='o',
                s=100
            )
            """

            # Moving average
            torque_x_ma = pd.DataFrame(data['torque_x'].rolling(window=20, center=True).mean())
            torque_x_ma["force_z"] = data["force_z"]
            torque_x_ma = torque_x_ma.dropna().sort_values(by='force_z')
            ax.set_ylim(-0.4, 0.4)
            ax.plot(
                torque_x_ma['force_z'], 
                torque_x_ma['torque_x'], 
                color=color,
                label=f'θ={tilt_angle}' + f"{'(baseline)' if tilt_angle == 0 else ''}",
                linewidth=2
            )

        ax.set_ylabel(f'Mw (Nm) at d={distance}')
        ax.grid(True)
        if i == len(distances) - 1:
            ax.set_xlabel('Fv (N)')

    axs[0, 0].set_title("Fw (N) vs Fv (N)", fontsize=14)
    axs[0, 1].set_title("Mw (Nm) vs Fv (N)", fontsize=14)
    plt.tight_layout()

    if filename:
        plt.savefig(filename)
    plt.show()

# Distances to plot
distances = [0.5, 1.0, 2.0]
plot_force_and_moment(df, distances)

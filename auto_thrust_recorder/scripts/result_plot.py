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

# use target_thrust as force_z
df["force_z"] = df["target_thrust"]

# Filter the data where distance is 0.5
filtered_df = df[df['distance'] == 2.0]

# Further filter based on force_z range (10 to 25)
filtered_df = filtered_df[(filtered_df['force_z'] >= 10) & (filtered_df['force_z'] <= 25) & (filtered_df['prop_spacing'] == 3.7)]

# Sorting by tilt_angle for consistent coloring
filtered_df = filtered_df.sort_values(by='tilt_angle')

# Defining the color map for tilt_angle values
def get_color(tilt_angle):
    if tilt_angle == -30:
        return 'red'
    elif tilt_angle == -15:
        return 'darkred'
        #return "#0070C0"
    elif tilt_angle == 0:
        return 'black'
    elif tilt_angle == 15:
        #return "#0070C0"
        return 'mediumpurple'
    elif tilt_angle == 30:
        return 'blue'
    return 'gray'

# Set font size for the plots
plt.rcParams.update({'font.size': 15})

# Plotting force_y vs force_z
plt.figure(figsize=(14, 8), facecolor="#F4F3F7")
#plt.figure(figsize=(14, 8))

# Scatter plot and moving average for force_y vs force_z
for tilt_angle in filtered_df['tilt_angle'].unique():
    """
    if tilt_angle != 0 and tilt_angle != 15:
        continue
    """
    subset = filtered_df[filtered_df['tilt_angle'] == tilt_angle].sort_values(by='force_z')
    color = get_color(tilt_angle)
    
    # Scatter plot for raw data
    
    plt.scatter(
        subset['force_z'], 
        subset['force_y'], 
        edgecolors=color,
        alpha=0.5,
        facecolors='none', 
        marker='o',
        s=100
    )
    

    # Compute and plot moving average with a window size of 20
    force_y_ma = pd.DataFrame(subset['force_y'].rolling(window=20, center=True).mean())
    force_y_ma["force_z"] = subset["force_z"]
    force_y_ma = force_y_ma.dropna().sort_values(by='force_z')
    
    plt.plot(
        force_y_ma['force_z'], 
        force_y_ma['force_y'], 
        color=color,
        label=f'θ={tilt_angle}' + f"{'(baseline)' if tilt_angle == 0 else ''}",
        linewidth=3
    )

plt.xlabel('Fv (N)')
plt.ylabel('Fw (N)')
plt.xlim(10, 25)
plt.ylim(-0.8, 0.8)
plt.legend(ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()

# Plotting torque_x vs force_z
plt.figure(figsize=(12, 8), facecolor="#F4F3F7")
#plt.figure(figsize=(14, 8))

# Scatter plot and moving average for torque_x vs force_z
for tilt_angle in filtered_df['tilt_angle'].unique():
    """
    if tilt_angle != 0 and tilt_angle != 15:
        continue
    """
    subset = filtered_df[filtered_df['tilt_angle'] == tilt_angle].sort_values(by='force_z')
    color = get_color(tilt_angle)
    
    # Scatter plot for raw data
    
    plt.scatter(
        subset['force_z'], 
        subset['torque_x'], 
        edgecolors=color, 
        alpha=0.5,
        facecolors='none', 
        marker='o',
        s=100
    )
    

    # Compute and plot moving average with a window size of 20
    torque_x_ma = pd.DataFrame(subset['torque_x'].rolling(window=20, center=True).mean())
    torque_x_ma["force_z"] = subset["force_z"]
    torque_x_ma = torque_x_ma.dropna().sort_values(by='force_z')
    
    label = f'θ={tilt_angle}' + f"{'(baseline)' if tilt_angle == 0 else ''}"

    plt.plot(
        torque_x_ma['force_z'], 
        torque_x_ma['torque_x'], 
        color=color,
        label=label,
        linewidth=3
    )

plt.xlabel('Fv (N)')
plt.ylabel('Mw (Nm)')
plt.xlim(10, 25)
plt.ylim(-0.25, 0.5)
plt.legend(ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()

# Plotting distance vs tilt_angle with torque_x ratio as color

df_filtered = df[(df['force_z'] >= 10) & (df['force_z'] <= 25)]

distance_values = df_filtered['distance'].unique()
distance_values.sort()
tilt_angles = df_filtered['tilt_angle'].unique()
tilt_angles.sort()

ratios = []
for distance in distance_values:
    row = []
    for tilt_angle in tilt_angles:
        subset = df_filtered[(df_filtered['distance'] == distance) & (df_filtered['tilt_angle'] == tilt_angle)]
        torque_x_mean = subset['torque_x'].mean()
        baseline_subset = df_filtered[(df_filtered['distance'] == distance) & (df_filtered['tilt_angle'] == 0)]
        baseline_mean = baseline_subset['torque_x'].mean()
        ratio = torque_x_mean / baseline_mean if baseline_mean != 0 else np.nan
        row.append(torque_x_mean)
    ratios.append(row)

ratios = np.array(ratios)

#plt.figure(figsize=(14, 6), facecolor="#F4F3F7")
plt.figure(figsize=(14, 6))

distance_values = distance_values * 2 + 1
c = plt.pcolormesh(distance_values, tilt_angles, ratios.T, cmap='coolwarm', shading='auto', vmin=-0.5, vmax=0.5)
plt.colorbar(c, label='Mw (Nm)')

# Adjust the ticks interval for both axes
plt.yticks(ticks=np.arange(min(tilt_angles), max(tilt_angles) + 1, 15))  # Set interval for tilt_angle axis
plt.xticks(ticks=np.arange(min(distance_values), max(distance_values) + 0.5, 0.5))  # Set interval for distance axis

plt.gca().invert_xaxis()

# Adding annotations for each cell
for i in range(len(distance_values)):
    for j in range(len(tilt_angles)):
        plt.text(distance_values[i], tilt_angles[j], f'{ratios[i, j]:.2f}', ha='center', va='center', color='black')

plt.xlabel('Distance (R)')
plt.ylabel('Tilt Angle (°)')
plt.tight_layout()
plt.show()

# Plotting distance vs tilt_angle with force_y ratio as color

df_filtered = df[(df['force_z'] >= 10) & (df['force_z'] <= 25)]

distance_values = df_filtered['distance'].unique()
distance_values.sort()
tilt_angles = df_filtered['tilt_angle'].unique()
tilt_angles.sort()

ratios = []
for distance in distance_values:
    row = []
    for tilt_angle in tilt_angles:
        subset = df_filtered[(df_filtered['distance'] == distance) & (df_filtered['tilt_angle'] == tilt_angle)]
        force_y_mean = subset['force_y'].mean()
        baseline_subset = df_filtered[(df_filtered['distance'] == distance) & (df_filtered['tilt_angle'] == 0)]
        baseline_mean = baseline_subset['force_y'].mean()
        ratio = force_y_mean / baseline_mean if baseline_mean != 0 else np.nan
        row.append(force_y_mean)
    ratios.append(row)

ratios = np.array(ratios)

#plt.figure(figsize=(14, 6), facecolor="#F4F3F7")
plt.figure(figsize=(14, 6))

distance_values = distance_values * 2 + 1
c = plt.pcolormesh(distance_values, tilt_angles, ratios.T, cmap='coolwarm', shading='auto', vmin=-0.5, vmax=0.5)
plt.colorbar(c, label='Fw (N)')

# Adjust the ticks interval for both axes
plt.yticks(ticks=np.arange(min(tilt_angles), max(tilt_angles) + 1, 15))  # Set interval for tilt_angle axis
plt.xticks(ticks=np.arange(min(distance_values), max(distance_values) + 0.5, 0.5))  # Set interval for distance axis

plt.gca().invert_xaxis()

# Adding annotations for each cell
for i in range(len(distance_values)):
    for j in range(len(tilt_angles)):
        plt.text(distance_values[i], tilt_angles[j], f'{ratios[i, j]:.2f}', ha='center', va='center', color='black')

plt.xlabel('Distance (R)')
plt.ylabel('Tilt Angle (°)')
plt.tight_layout()
plt.show()
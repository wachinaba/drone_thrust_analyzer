import pandas as pd
import numpy as np
import argparse

def average_filter_csv(file_path):
    # Load the CSV file (assuming the file path is available)
    df = pd.read_csv(file_path)

    # Group rows where motor_0 values are considered equal, allowing for small differences
    tolerance = 0.00001
    df['motor_0_rounded'] = df['motor_0'].round(decimals=5)

    # Define the columns that need to be averaged
    columns_to_average = ['time', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']

    # Group by motor_0 and compute the mean for the relevant columns
    df_grouped = df.groupby('motor_0_rounded')[columns_to_average].mean().reset_index()

    # Add the motor column by aggregating motor_0, motor_1, motor_2, and motor_3
    df_grouped['motor'] = df.groupby('motor_0_rounded')[['motor_0', 'motor_1', 'motor_2', 'motor_3']].mean().sum(axis=1)

    # Drop the temporary rounded column
    df_grouped.drop('motor_0_rounded', axis=1, inplace=True)

    df_grouped.to_csv(file_path + "_filtered.csv", index=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Average filter for CSV file")
    parser.add_argument("file_path", type=str, help="Path to the CSV file")
    args = parser.parse_args()

    average_filter_csv(args.file_path)
import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime

class AveragePlotter:
    """Handles plotting of averaged data."""

    def __init__(self, avg_df: pd.DataFrame):
        """
        Args:
            avg_df (pd.DataFrame): Averaged data frame with columns:
                ['thrust_step', 'avg_force_x', 'avg_force_y', 'avg_force_z',
                 'avg_torque_x', 'avg_torque_y', 'avg_torque_z',
                 'avg_motor_0', 'avg_motor_1', 'avg_motor_2', 'avg_motor_3']
            filename_prefix (str): Prefix for the saved plot image.
        """
        self.avg_df = avg_df
        self.fig, self.axs = plt.subplots(2, 1, figsize=(12, 14), sharex=True)

    def plot(self):
        """Generates the plots based on the averaged data."""
        if self.avg_df.empty:
            raise ValueError("Average DataFrame is empty. Cannot plot data.")

        try:
            # 1つ目のサブプロット：Control vs Force XY and Force Z (Dual y-axes)
            ax1 = self.axs[0]
            ax2 = ax1.twinx()  # 右側のy軸を作成

            # Force XY
            ax1.plot(self.avg_df['control'], self.avg_df['force_x'], label='Force X', color='r', linestyle='-')
            ax1.plot(self.avg_df['control'], self.avg_df['force_y'], label='Force Y', color='g', linestyle='--')
            ax1.set_ylabel('Force XY (N)', color='k')
            ax1.tick_params(axis='y', labelcolor='k')
            ax1.set_ylim([-2, 2])

            # Force Z
            ax2.plot(self.avg_df['control'], self.avg_df['force_z'], label='Force Z', color='b', linestyle='-.')
            ax2.set_ylabel('Force Z (N)', color='b')
            ax2.tick_params(axis='y', labelcolor='b')
            ax2.set_ylim([0, 30])

            # タイトルと凡例
            ax1.set_title('Control vs Force XY and Force Z')
            ax1.legend(loc='upper left')
            ax2.legend(loc='upper right')
            ax1.grid(True)

            # 2つ目のサブプロット：Control vs Torque X, Y, Z
            ax3 = self.axs[1]
            ax3.plot(self.avg_df['control'], self.avg_df['torque_x'], label='Torque X', color='r', linestyle='-')
            ax3.plot(self.avg_df['control'], self.avg_df['torque_y'], label='Torque Y', color='g', linestyle='--')
            ax3.plot(self.avg_df['control'], self.avg_df['torque_z'], label='Torque Z', color='b', linestyle='-.')
            ax3.set_xlabel('Control')
            ax3.set_ylabel('Torque (Nm)')
            ax3.set_title('Control vs Torque X, Y, Z')
            ax3.legend()
            ax3.grid(True)
            ax3.set_ylim([-1, 1])

            # レイアウトを調整
            plt.tight_layout()

        except Exception as e:
            raise RuntimeError(f"Failed to generate plots: {e}")

    def show(self):
        """Displays the generated plots."""
        try:
            self.plot()
            plt.show()
        except Exception as e:
            print(f"Error displaying plot: {e}")

    def save(self, filename: str):
        """
        Saves the generated plots as a PNG file.

        Args:
            save_directory (str): Directory where the plot image will be saved.
                                   Defaults to the current directory.
        """
        try:
            self.fig.savefig(filename)
            print(f'Plots saved as {filename}')
        except Exception as e:
            print(f"Error saving plot: {e}")

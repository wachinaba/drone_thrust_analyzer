import numpy as np
import pandas as pd
from auto_thrust_recorder.logger.logger import Logger

class AverageLogger(Logger):
    def __init__(self):
        self.data = pd.DataFrame()
        self.accumulated_data = pd.DataFrame()

    def log(self, data: dict[str, float]): # override
        self.accumulated_data = pd.concat([self.accumulated_data, pd.DataFrame(data, index=[0])], ignore_index=True)

    def get_data(self) -> pd.DataFrame: # override
        return self.data

    def next(self):
        # 平均を計算, 次の行に追加
        print(self.accumulated_data.mean(axis=0).to_frame().T)

        self.data = pd.concat([self.data, self.accumulated_data.mean(axis=0).to_frame().T], ignore_index=True)
        # 累積データをリセット
        self.accumulated_data = pd.DataFrame()

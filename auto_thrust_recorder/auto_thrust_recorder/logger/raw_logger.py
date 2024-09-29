import numpy as np
import pandas as pd
from auto_thrust_recorder.logger.logger import Logger

class RawLogger(Logger):
    def __init__(self):
        self.data = pd.DataFrame()

    def log(self, data: dict[str, float]): # override
        self.data = pd.concat([self.data, pd.DataFrame(data, index=[0])], ignore_index=True)

    def get_data(self) -> pd.DataFrame: # override
        return self.data            
            
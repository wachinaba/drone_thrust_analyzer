from abc import ABC, abstractmethod
import pandas as pd
class Logger(ABC):
    @abstractmethod
    def log(self, data: dict[str, float]):
        pass

    @abstractmethod
    def get_data(self) -> pd.DataFrame:
        pass

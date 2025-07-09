import pandas as pd
from auto_thrust_recorder.logger.logger import Logger
# from auto_thrust_recorder.exporter.exporter import Exporter

class CSVExporter:
    def __init__(self, logger: Logger, file_path: str):
        self.file_path = file_path
        self.logger = logger

    def export(self):
        data = self.logger.get_data()
        data.to_csv(self.file_path, index=False)

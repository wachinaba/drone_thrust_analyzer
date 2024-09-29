# scheduler.py

from abc import ABC, abstractmethod
from typing import Callable
import numpy as np

class Scheduler(ABC):
    """Abstract base class for different scheduling strategies."""

    @abstractmethod
    def initialize(self):
        """Initialize the scheduler. Called at the start of scheduling."""
        pass

    @abstractmethod
    def finalize(self):
        """Finalize the scheduler. Called at the end of scheduling."""
        pass

    @abstractmethod
    def set_on_change_thrust(self, callback: Callable[[np.ndarray], None]):
        pass

    @abstractmethod
    def set_on_complete_callback(self, callback: Callable[[], None]):
        pass

    @abstractmethod
    def get_current_control(self) -> np.ndarray:
        pass
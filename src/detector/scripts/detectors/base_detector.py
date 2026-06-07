from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

@dataclass
class Detection:
    bbox: Tuple[int, int, int, int]
    confidence: float
    class_id: int


class BaseDetector(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        ...

    @abstractmethod
    def load_model(self) -> None:
        ...
from typing import List

import numpy as np

from object_detection.config import DetectionConfig
from object_detection.detectors.base import BaseDetector, Detection
from object_detection.detectors.yolo import YoloDetector

from object_detection.visualize import draw_center, draw_detections

class Detector:
    def __init__(self, model: str, config: DetectionConfig):
        self._config = config
        self._engine: BaseDetector = self._create_engine(model)

    def _create_engine(self, model: str) -> BaseDetector:
        if model == "yolo":
            return YoloDetector(self._config)
        raise ValueError(f"Unknown model: {model}")

    def load_model(self) -> None:
        self._engine.load_model()

    def detect(self, image: np.ndarray) -> List[Detection]:
        return self._engine.detect(image)

    def __call__(self, image: np.ndarray) -> List[Detection]:
        return self.detect(image)
    
    def draw(
        self,
        image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:
        annotated = draw_detections(image, detections, self._config)
        return draw_center(annotated, detections)
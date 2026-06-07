from typing import List

import os
import sys

import numpy as np
from ultralytics import YOLO

from detectors.base_detector import BaseDetector, Detection

cur_dir = os.path.dirname(os.path.abspath(__file__))
dets_dir = os.path.dirname(cur_dir)
pkg_dir = os.path.dirname(dets_dir)
model_dir = os.path.join(pkg_dir, "models")

class YoloDetector(BaseDetector):
    def __init__(self, model_config: dict) -> None:
        self._model: YOLO = None

        self._model_version = model_config.get("version", "11")
        self._model_scale = model_config.get("scale", "n")
        self._model_weight = model_config.get("weight", "pt")

        self._model_path = os.path.join(model_dir, f"yolo{self._model_version}{self._model_scale}.{self._model_weight}")

        self._conf_threshold = model_config.get("conf_threshold", 0.25)
        self._iou_threshold = model_config.get("iou_threshold", 0.45)
        self._device = model_config.get("device", "cpu")

        self._classes = model_config.get("classes", [0, 1, 2, 3])

    def load_model(self) -> None:
        self._model = YOLO(self._model_path)

    def detect(self, image: np.ndarray) -> List[Detection]:
        if self._model is None:
            raise RuntimeError("Model not loaded.")
        
        model = self._model

        results = model(
            image, 
            conf=self._conf_threshold, 
            iou=self._iou_threshold,
            device=self._device,
            classes=self._classes,
            verbose=False
        )

        detections: List[Detection] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                # 确保 x1 < x2 且 y1 < y2
                x1, x2 = min(x1, x2), max(x1, x2)
                y1, y2 = min(y1, y2), max(y1, y2)
                confidence = float(boxes.conf[i])
                class_id = int(boxes.cls[i])
                detections.append(
                    Detection(
                        bbox=(x1, y1, x2, y2),
                        confidence=confidence,
                        class_id=class_id
                    )
                )

        return detections

from typing import List, Optional

import numpy as np
from ultralytics import YOLO

from object_detection.detectors.base import BaseDetector, Detection
from object_detection.config import DetectionConfig

import os
model_dir = os.path.join(os.path.dirname(__file__), "models")

class YoloDetector(BaseDetector):
    def __init__(self, config: DetectionConfig) -> None:
        self._model: Optional[YOLO] = None

        self._model_version = config.model_version
        self._model_scale = config.model_scale
        self._model_weight = config.model_weight

        self._model_path = os.path.join(model_dir, f"yolo{self._model_version}{self._model_scale}.{self._model_weight}")

        self._conf_threshold = config.conf_threshold
        self._iou_threshold = config.iou_threshold
        self._imgsz = config.imgsz
        self._device = config.device

        self._classes = config.classes

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
            imgsz=self._imgsz, 
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
                confidence = float(boxes.conf[i])
                class_id = int(boxes.cls[i])
                class_name = result.names[class_id] if result.names else str(class_id)
                detections.append(
                    Detection(
                        bbox=(x1, y1, x2, y2),
                        confidence=confidence,
                        class_id=class_id,
                        class_name=class_name,
                    )
                )

        return detections

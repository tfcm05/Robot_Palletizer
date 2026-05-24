from typing import List, Optional, Tuple

import numpy as np
import cv2

from object_detection.detectors.base import Detection
from object_detection.config import DetectionConfig


_COLORS = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
    (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
]

def _get_color(class_id: int) -> Tuple[int, int, int]:
    return _COLORS[class_id % len(_COLORS)]


def draw_detections(
    image: np.ndarray,
    detections: List[Detection],
    cfg: DetectionConfig
) -> np.ndarray:
    lw = cfg.line_width
    fs = cfg.font_scale
    ft = cfg.font_thickness

    show_conf = cfg.show_confidence

    output = image.copy()
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        color = _get_color(det.class_id)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, lw)

        label = det.class_name
        if show_conf:
            label += f" {det.confidence:.2f}"

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, ft)
        cv2.rectangle(output, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(
            output, label, (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), ft,
        )

    return output


def draw_center(
    image: np.ndarray,
    detections: List[Detection],
    color: Tuple[int, int, int] = (0, 0, 255),
    radius: int = 4,
) -> np.ndarray:
    output = image.copy()
    for det in detections:
        cx = (det.bbox[0] + det.bbox[2]) // 2
        cy = (det.bbox[1] + det.bbox[3]) // 2
        cv2.circle(output, (cx, cy), radius, color, -1)
        cv2.line(output, (cx - 8, cy), (cx + 8, cy), color, 1)
        cv2.line(output, (cx, cy - 8), (cx, cy + 8), color, 1)
    return output
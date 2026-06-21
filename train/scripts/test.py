#!/usr/bin/env python3
import os
import sys
import random

import cv2
import numpy as np
from ultralytics import YOLO

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
MODEL_PATH = os.path.join(ROOT_DIR, "outputs", "yolo11n.pt")
IMAGES_DIR = os.path.join(ROOT_DIR, "dataset", "images")
OUTPUT_DIR = os.path.join(ROOT_DIR, "outputs", "test")

COLORS = [
    (0, 255, 0),    # green_large
    (0, 200, 200),  # green_small
    (0, 0, 255),    # red_large
    (0, 160, 255),  # red_small
]


def draw_boxes(image: np.ndarray, results, names: dict) -> np.ndarray:
    boxes = results.boxes
    if boxes is None:
        return image

    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i])
        conf = float(boxes.conf[i])
        x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
        color = COLORS[cls_id % len(COLORS)]
        label = f"{names[cls_id]} {conf:.2f}"

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(image, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return image


def main() -> None:
    if not os.path.isfile(MODEL_PATH):
        print(f"[ERROR] model not found: {MODEL_PATH}")
        sys.exit(1)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    model = YOLO(MODEL_PATH)

    images = [os.path.join(IMAGES_DIR, f)
              for f in os.listdir(IMAGES_DIR) if f.endswith(".jpg")]
    test_images = random.sample(images, min(5, len(images)))

    print(f"model: {MODEL_PATH}")
    print(f"testing {len(test_images)} images...\n")

    for img_path in test_images:
        name = os.path.basename(img_path)
        image = cv2.imread(img_path)
        if image is None:
            print(f"[{name}] failed to read")
            continue

        results = model(image, conf=0.25, verbose=False)
        names = results[0].names

        vis_image = draw_boxes(image.copy(), results[0], names)

        out_path = os.path.join(OUTPUT_DIR, name)
        cv2.imwrite(out_path, vis_image)

        boxes = results[0].boxes
        count = len(boxes) if boxes is not None else 0
        print(f"[{name}] {count} objects -> saved to {out_path}")


if __name__ == "__main__":
    main()

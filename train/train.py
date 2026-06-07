from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a YOLO model for colored block detection."
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=Path(__file__).with_name("data.yaml"),
        help="Path to the Ultralytics dataset yaml file.",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).with_name("yolo11n.pt"),
        help="Path to the pretrained checkpoint used to start training.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=100,
        help="Number of training epochs.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Training image size.",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=16,
        help="Batch size.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="0",
        help="Training device, for example 0, cpu, or 0,1.",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=Path(__file__).with_name("runs"),
        help="Directory where Ultralytics saves training outputs.",
    )
    parser.add_argument(
        "--name",
        type=str,
        default="blocks",
        help="Training run name.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="Number of dataloader workers.",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=50,
        help="Early stopping patience.",
    )
    parser.add_argument(
        "--cache",
        action="store_true",
        help="Cache images in memory during training.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume the last training run if possible.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.data.exists():
        raise FileNotFoundError(
            f"Dataset yaml not found: {args.data}. Create it first, for example train/data.yaml."
        )

    if not args.model.exists():
        raise FileNotFoundError(
            f"Initial model checkpoint not found: {args.model}"
        )

    model = YOLO(str(args.model))
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        cache=args.cache,
        project=str(args.project),
        name=args.name,
        resume=args.resume,
        pretrained=True,
    )


if __name__ == "__main__":
    main()
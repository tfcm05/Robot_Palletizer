import argparse
import os
import shutil

from ultralytics import YOLO

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_YAML = os.path.join(ROOT_DIR, "data.yaml")
DEFAULT_MODEL = os.path.join(ROOT_DIR, "models", "yolo11n.pt")
OUTPUT_DIR = os.path.join(ROOT_DIR, "outputs")


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOLO11 on custom dataset")
    parser.add_argument("--data", default=DATA_YAML, help="path to data.yaml")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="pretrained model name or path (e.g. yolo11n.pt / yolo11s.pt)")
    parser.add_argument("--epochs", type=int, default=100, help="number of training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="input image size")
    parser.add_argument("--batch", type=int, default=16, help="batch size")
    parser.add_argument("--device", default="cuda", help="device (cuda / cpu / 0,1)")
    parser.add_argument("--lr0", type=float, default=0.01, help="initial learning rate")
    parser.add_argument("--patience", type=int, default=10, help="early stopping patience")
    parser.add_argument("--resume", action="store_true", help="resume from last checkpoint")
    parser.add_argument("--output", default=OUTPUT_DIR,
                        help="directory to save trained model")
    parser.add_argument("--workers", type=int, default=8, help="number of dataloader workers")
    args = parser.parse_args()

    runs_dir = os.path.join(ROOT_DIR, "outputs", "runs")
    project_dir = os.path.join(runs_dir, "detect")
    run_name = "train"

    model = YOLO(args.model)

    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        lr0=args.lr0,
        patience=args.patience,
        resume=args.resume,
        workers=args.workers,
        project=project_dir,
        name=run_name,
        exist_ok=True,
    )

    best_pt = os.path.join(project_dir, run_name, "weights", "best.pt")
    model_name = os.path.basename(args.model)
    os.makedirs(args.output, exist_ok=True)
    dst = os.path.join(args.output, model_name)
    if os.path.isfile(best_pt):
        shutil.copy2(best_pt, dst)
        print(f"\n[OK] model saved to {dst}")


if __name__ == "__main__":
    main()

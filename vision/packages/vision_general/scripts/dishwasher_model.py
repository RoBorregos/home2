"""
Training and inference pipeline for detecting empty slots and cutlery in dishwashers.
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List, Any

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect empty slots and cutlery in dishwashers using YOLOv8"
    )

    # Dataset
    parser.add_argument(
        "--dataset_root",
        default="vision/packages/vision_general/Dishwasher-empty-slots.v2i.yolov8",
        help="Path to local YOLOv8 dataset (must contain data.yaml)",
    )

    # Training
    parser.add_argument("--model", default="yolov8n.pt", help="YOLOv8 base model")
    parser.add_argument("--epochs", type=int, default=20, help="Number of training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")

    # Inference
    parser.add_argument(
        "--test_source",
        default="",
        help="Folder or image for inference (uses dataset_root/test/images if empty)",
    )
    parser.add_argument("--conf", type=float, default=0.35, help="Confidence threshold")
    parser.add_argument("--only_class", default="empty_slot", help="Class name to export")
    parser.add_argument("--save_predict_images", action="store_true", help="Save predicted images")

    # Output
    parser.add_argument("--out_dir", default="outputs", help="Output directory")
    parser.add_argument("--out_json", default="empty_slots.json", help="Output JSON filename")
    parser.add_argument("--print_best", action="store_true", help="Print best detection per image")

    return parser.parse_args()


def count_files(path: Path) -> int:
    return len([p for p in path.glob("*") if p.is_file()]) if path.exists() else 0


def validate_dataset(dataset_root: str) -> str:
    dataset_root = Path(dataset_root).expanduser().resolve()
    data_yaml = dataset_root / "data.yaml"

    if not data_yaml.exists():
        raise FileNotFoundError(f"data.yaml not found in {dataset_root}")

    # Verify required splits
    for split in ["train", "valid"]:
        for folder in ["images", "labels"]:
            path = dataset_root / split / folder
            if not path.exists():
                raise FileNotFoundError(f"Missing {split}/{folder} in {dataset_root}")

    # Print dataset summary
    print("\n[Dataset Information]")
    print(f"Location: {dataset_root}")
    print(f"  train/images: {count_files(dataset_root / 'train' / 'images')} files")
    print(f"  train/labels: {count_files(dataset_root / 'train' / 'labels')} files")
    print(f"  valid/images: {count_files(dataset_root / 'valid' / 'images')} files")
    print(f"  valid/labels: {count_files(dataset_root / 'valid' / 'labels')} files")

    test_dir = dataset_root / "test" / "images"
    test_count = count_files(test_dir)
    print(f"  test/images:  {test_count} files" if test_count else "  test/images:  (not found)")

    return str(data_yaml)


def calculate_area(bbox: List[float]) -> float:
    """Calculate bounding box area from [x1, y1, x2, y2]."""
    x1, y1, x2, y2 = bbox
    return max(0.0, (x2 - x1) * (y2 - y1))


def get_inference_source(dataset_root: Path, test_source_arg: str) -> str:
    if test_source_arg:
        source = Path(test_source_arg).expanduser().resolve()
        if source.exists():
            return str(source)
        raise FileNotFoundError(f"--test_source not found: {source}")

    default_test = dataset_root / "test" / "images"
    if default_test.exists():
        return str(default_test)

    raise FileNotFoundError(
        "No test source found. "
        f"{default_test} exists."
    )


def find_best_weights() -> Path:
    """
    Find the most recent best.pt from training runs.
    """
    runs_dir = Path("runs/detect")
    if not runs_dir.exists():
        raise FileNotFoundError("No training runs found. Train a model first (runs/detect/)")

    best_models = sorted(
        runs_dir.glob("train*/weights/best.pt"),
        key=lambda p: p.stat().st_mtime,
        reverse=True
    )

    return best_models[0]


def process_detections(
    results: List,
    only_class: str
) -> Dict[str, List[Dict[str, Any]]]:

    detections: Dict[str, List[Dict[str, Any]]] = {}

    for result in results:
        img_path = getattr(result, "path", None)
        img_name = Path(img_path).name if img_path else "unknown"
        detections[img_name] = []

        boxes = getattr(result, "boxes", None)
        if not boxes or len(boxes) == 0:
            continue

        # Process each detection
        for box in boxes:
            class_id = int(box.cls[0].item())
            class_name = result.names.get(class_id, str(class_id))

            # Filter by requested class
            if class_name != only_class:
                continue

            # Extract coordinates and confidence
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            confidence = float(box.conf[0].item())
            bbox_area = calculate_area([x1, y1, x2, y2])

            detections[img_name].append({
                "class": class_name,
                "confidence": round(confidence, 4),
                "bbox": [round(x, 2) for x in [x1, y1, x2, y2]],
                "area": round(bbox_area, 2),
            })

        # Sort by confidence (highest first)
        detections[img_name].sort(key=lambda d: d["confidence"], reverse=True)

    return detections


def save_detections(detections: Dict, output_dir: str, output_file: str) -> Path:
    output_path = (Path(output_dir).expanduser().resolve() / output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(detections, f, indent=2)

    return output_path


def print_summary(detections: Dict, class_name: str) -> None:
    print(f"\n[Best '{class_name}' per image (by area)]")
    
    for img_name, boxes in detections.items():
        if not boxes:
            print(f"  {img_name}: (no detections)")
        else:
            best = max(boxes, key=lambda d: d["area"])
            area = best["area"]
            conf = best["confidence"]
            print(f"  {img_name}: area={area}, confidence={conf}")


def main() -> None:
    args = parse_args()
    dataset_root = Path(args.dataset_root).expanduser().resolve()

    # Validate dataset
    data_yaml = validate_dataset(str(dataset_root))

    # Train model
    model = YOLO(args.model)
    model.train(
        data=data_yaml,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
    )

    # Load best model
    best_weights = find_best_weights()

    # Run inference
    test_source = get_inference_source(dataset_root, args.test_source)
    print(f"Running inference on: {test_source}")

    model = YOLO(str(best_weights))
    results = model.predict(
        source=test_source,
        conf=args.conf,
        save=args.save_predict_images,
        verbose=False,
    )

    detections = process_detections(results, args.only_class)
    output_file = save_detections(detections, args.out_dir, args.out_json)
    print(f"\nDetections: {output_file}")

    if args.print_best:
        print_summary(detections, args.only_class)


if __name__ == "__main__":
    main()
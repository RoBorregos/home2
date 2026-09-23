#!/usr/bin/env python3
"""One-off converter: RCW2026_v2 (Ultralytics YOLO-seg export, the real
training data behind robocup2026_v1.pt) -> this benchmark's data/ layout.

KNOWN LIMITATION (read before trusting any number this produces): per
RCW2026_v2/imported_classes.json, every class's source images come from a
SINGLE "from_repo" identifier (e.g. apple's images are all
"apple_RCWarmUp2.0"). dataset/data.yaml's train/valid/test split is a random
split of frames WITHIN that one capture session, not a split across
independent sessions. So held_out/ built from this script is a
different-FRAME split, not a different-SESSION split — recall@1 measured on
it is an optimistic sanity check (same lighting/background/backdrop as
gallery_photos/), not the field number the plan's Phase 1 gate actually
requires. A real gate pass still needs a second, independently-shot photo
session for held_out/. This script exists to validate the matching CODE end
to end on real objects, and to get a rough DINOv2-vs-CLIP signal — not to
sign off on recall@1/unknown-rejection targets.

Labels are YOLO-SEG format (class_id x1 y1 x2 y2 ... xn yn, normalized
polygon), not plain bbox — bbox here is the polygon's axis-aligned bounding
box.

Usage:
    python3 prepare_dataset.py --source ~/Downloads/RCW2026_v2
"""

import argparse
import json
import random
import shutil
from pathlib import Path

from PIL import Image

HERE = Path(__file__).parent
DATA_DIR = HERE / "data"
TRANSLATION_PATH = (
    HERE.parents[1]
    / "packages"
    / "object_detector_2d"
    / "scripts"
    / "detectors"
    / "robocup2026_translation.json"
)

# Held out of gallery_photos/ entirely, so their held_out crops can serve as
# genuine "not in the gallery" negatives for unknown_rejection_rate.
# Names are PUBLISHED labels (post-translation) — see load_translation().
OUT_OF_GALLERY_CLASSES = {"mangostane", "rubiks_cube", "seaweed"}

# Curated because they're visually close within this 28-class set (per the
# plan's critique: hard negatives must be *chosen*, not random) — cutlery,
# kitchenware silhouettes, and round fruit are the closest look-alikes here.
# coca_cola/coca_cola_zero and blue/brown_cereal_box are deliberately
# excluded: robocup2026_translation.json already collapses each pair to one
# published label (coke / cornflakes), so confusing them isn't a real error
# for this matcher — the gallery is built on published labels below, so
# those pairs never even become two separate classes to confuse.
HARD_NEGATIVE_CLASSES = {
    "fork",
    "knife",
    "spoon",
    "cup",
    "bowl",
    "plate",
    "red_bellpepper",
    "yellow_bellpepper",
    "apple",
    "peach",
}


def load_translation() -> dict[str, str]:
    if not TRANSLATION_PATH.exists():
        return {}
    return json.loads(TRANSLATION_PATH.read_text())


GALLERY_PHOTOS_PER_CLASS = 30
HELD_OUT_PER_CLASS = 12
HARD_NEGATIVE_PER_CLASS = 10
OUT_OF_GALLERY_PER_CLASS = 15
BOX_RECALL_IMAGES = 25

random.seed(0)


def load_class_names(source: Path) -> list[str]:
    import re

    text = (source / "dataset" / "data.yaml").read_text()
    names = []
    in_names = False
    for line in text.splitlines():
        if line.startswith("names:"):
            in_names = True
            continue
        if in_names:
            m = re.match(r"-\s*(.+)", line.strip())
            if m:
                names.append(m.group(1))
            else:
                break
    return names


def parse_label_file(path: Path) -> list[tuple[int, list[float]]]:
    """Returns [(class_id, [x1,y1,x2,y2,...normalized polygon]), ...]."""
    objects = []
    if not path.exists():
        return objects
    for line in path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        class_id = int(parts[0])
        coords = [float(v) for v in parts[1:]]
        objects.append((class_id, coords))
    return objects


def polygon_bbox_px(coords: list[float], img_w: int, img_h: int) -> list[int]:
    xs = [c * img_w for c in coords[0::2]]
    ys = [c * img_h for c in coords[1::2]]
    return [round(min(xs)), round(min(ys)), round(max(xs)), round(max(ys))]


def iter_split(source: Path, split: str, names: list[str], translation: dict[str, str]):
    """Yields (image_path, [(published_label, bbox_px), ...]) for every image
    with at least one label in this split. Labels are already translated
    through robocup2026_translation.json — this is what the production
    detector's Detection.label_ would carry, and it's the identity the
    acceptance criteria (recall@1, unknown-rejection) actually cares about."""
    images_dir = source / "dataset" / split / "images"
    labels_dir = source / "dataset" / split / "labels"
    for img_path in sorted(images_dir.iterdir()):
        label_path = labels_dir / (img_path.stem + ".txt")
        objects = parse_label_file(label_path)
        if not objects:
            continue
        with Image.open(img_path) as im:
            w, h = im.size
        boxes = [
            (translation.get(names[cid], names[cid]), polygon_bbox_px(coords, w, h))
            for cid, coords in objects
        ]
        yield img_path, boxes


def crops_by_class(
    source: Path,
    split: str,
    names: list[str],
    translation: dict[str, str],
    wanted_classes: set[str],
    per_class: int,
):
    """published_label -> list of (source_image_path, bbox_px), capped per class."""
    buckets: dict[str, list] = {c: [] for c in wanted_classes}
    for img_path, boxes in iter_split(source, split, names, translation):
        for label, bbox in boxes:
            if label in wanted_classes and len(buckets[label]) < per_class * 3:
                buckets[label].append((img_path, bbox))
    for label in buckets:
        random.shuffle(buckets[label])
        buckets[label] = buckets[label][:per_class]
    return buckets


def save_crop(img_path: Path, bbox: list[int], out_path: Path):
    with Image.open(img_path) as im:
        im = im.convert("RGB")
        x1, y1, x2, y2 = bbox
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(im.width, x2), min(im.height, y2)
        if x2 <= x1 or y2 <= y1:
            return False
        im.crop((x1, y1, x2, y2)).save(out_path, quality=95)
    return True


def _reset_dir(out_dir: Path):
    """Wipe out_dir entirely before rebuilding. Both report.py and
    box_recall_eval.py discover gallery_photos/*/ and out_of_gallery/* by
    directory listing, not by an explicit manifest — a stale subdirectory or
    file left over from a previous run (e.g. under an old, pre-translation
    class name) would silently get counted as gallery content forever."""
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)


def build_gallery_photos(
    source: Path, names: list[str], translation: dict[str, str], published: set[str]
):
    wanted = published - OUT_OF_GALLERY_CLASSES
    buckets = crops_by_class(
        source, "train", names, translation, wanted, GALLERY_PHOTOS_PER_CLASS
    )
    out_dir = DATA_DIR / "gallery_photos"
    _reset_dir(out_dir)
    for label, items in buckets.items():
        class_dir = out_dir / label
        class_dir.mkdir(parents=True, exist_ok=True)
        n = 0
        for img_path, bbox in items:
            if save_crop(img_path, bbox, class_dir / f"{n:03d}.jpg"):
                n += 1
        print(f"[gallery_photos] {label}: {n} crops")


def build_held_out(
    source: Path, names: list[str], translation: dict[str, str], published: set[str]
):
    wanted = published - OUT_OF_GALLERY_CLASSES
    buckets = crops_by_class(
        source, "test", names, translation, wanted, HELD_OUT_PER_CLASS
    )
    out_dir = DATA_DIR / "held_out"
    _reset_dir(out_dir)
    annotations = {}
    for label, items in buckets.items():
        for i, (img_path, bbox) in enumerate(items):
            filename = f"{label}_{i:03d}.jpg"
            if save_crop(img_path, bbox, out_dir / filename):
                annotations[filename] = label
    (out_dir / "annotations.json").write_text(
        json.dumps(annotations, indent=2, sort_keys=True) + "\n"
    )
    print(f"[held_out] {len(annotations)} crops across {len(buckets)} classes")


def build_hard_negatives(
    source: Path, names: list[str], translation: dict[str, str], published: set[str]
):
    wanted = HARD_NEGATIVE_CLASSES & published
    buckets = crops_by_class(
        source, "valid", names, translation, wanted, HARD_NEGATIVE_PER_CLASS
    )
    out_dir = DATA_DIR / "hard_negatives"
    _reset_dir(out_dir)
    annotations = {}
    for label, items in buckets.items():
        for i, (img_path, bbox) in enumerate(items):
            filename = f"{label}_{i:03d}.jpg"
            if save_crop(img_path, bbox, out_dir / filename):
                annotations[filename] = label
    (out_dir / "annotations.json").write_text(
        json.dumps(annotations, indent=2, sort_keys=True) + "\n"
    )
    print(f"[hard_negatives] {len(annotations)} crops across {sorted(buckets)}")


def build_out_of_gallery(
    source: Path, names: list[str], translation: dict[str, str], published: set[str]
):
    buckets = crops_by_class(
        source,
        "test",
        names,
        translation,
        OUT_OF_GALLERY_CLASSES & published,
        OUT_OF_GALLERY_PER_CLASS,
    )
    out_dir = DATA_DIR / "out_of_gallery"
    _reset_dir(out_dir)
    n = 0
    for label, items in buckets.items():
        for img_path, bbox in items:
            if save_crop(img_path, bbox, out_dir / f"{label}_{n:03d}.jpg"):
                n += 1
    print(
        f"[out_of_gallery] {n} crops from held-out classes {sorted(OUT_OF_GALLERY_CLASSES)}"
    )


def build_box_recall(source: Path, names: list[str], translation: dict[str, str]):
    out_dir = DATA_DIR / "box_recall"
    _reset_dir(out_dir)
    samples = list(iter_split(source, "train", names, translation))
    random.shuffle(samples)
    annotations = {}
    for img_path, boxes in samples[:BOX_RECALL_IMAGES]:
        filename = img_path.name
        shutil.copy2(img_path, out_dir / filename)
        annotations[filename] = [
            {"bbox": bbox, "label": label} for label, bbox in boxes
        ]
    (out_dir / "annotations.json").write_text(json.dumps(annotations, indent=2) + "\n")
    print(
        f"[box_recall] {len(annotations)} images, {sum(len(v) for v in annotations.values())} boxes"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--source", required=True, help="path to RCW2026_v2")
    args = parser.parse_args()
    source = Path(args.source).expanduser()

    names = load_class_names(source)
    translation = load_translation()
    published = {translation.get(n, n) for n in names}
    print(
        f"[prepare_dataset] {len(names)} raw classes -> {len(published)} published classes (via translation.json)"
    )
    print(f"[prepare_dataset] raw: {names}")
    print(f"[prepare_dataset] published: {sorted(published)}")

    build_gallery_photos(source, names, translation, published)
    build_held_out(source, names, translation, published)
    build_hard_negatives(source, names, translation, published)
    build_out_of_gallery(source, names, translation, published)
    build_box_recall(source, names, translation)

    print(
        "\n[prepare_dataset] DONE. Remember the session-leakage caveat in this "
        "script's docstring before trusting recall@1/unknown-rejection numbers "
        "computed from this data as a real Phase 1 gate pass."
    )


if __name__ == "__main__":
    main()

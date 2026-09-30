# embedding_gallery benchmark

Benchmark behind the few-shot object recognition in `ObjectDetect2D` (objects added
from photos, no retraining). It chose the box proposer and the DINOv2 backbone and
calibrated the match thresholds. Same shape as `hri/benchmarks/{nlp,stt}/`: `run.sh` →
`tasks.py` (task registry) → `report.py` (tables and JSON), with `models.json` as the
registry.

- **Want to add an object to the robot?** That is not this folder: see
  [Adding an object to the gallery](../../README.md#adding-an-object-to-the-gallery-few-shot)
  in `vision/README.md`.
- Background, accuracy numbers and troubleshooting:
  [`docs/ai/embedding_gallery.md`](../../../docs/ai/embedding_gallery.md).

## Before you start

`data/` is gitignored, so it is empty on a fresh clone and every task below fails until you
build it. It is generated once from the YOLO-seg export of the `RCW2026_v2` dataset (about
22 GB, not in this repo). The export must contain `dataset/data.yaml` and
`dataset/{train,valid,test}/{images,labels}`; pass the folder that contains `dataset/`:

```bash
./run.sh prepare --source /path/to/RCW2026_v2
```

`prepare` **deletes and rebuilds** `data/`, and the numbers change with the sampled images.
`e2e_eval` and `e2e_calibrate` also need `--source` on every run (they read images from the
export itself, not from `data/`), except `e2e_calibrate --from-cache`.

## What the tasks measure

| Task | Question it answers |
|---|---|
| `boxes` (Phase 0) | Does the class-agnostic box proposer find the objects at all? Recall at IoU 0.5 |
| `embeddings` (Phase 1) | Given hand-labeled crops, does backbone + gallery pick the right label (recall@1), reject look-alikes (`hard_negatives/`) and reject objects not in the gallery (`out_of_gallery/`)? |
| `e2e_eval` | The same, but with crops from the real proposer, which are looser than hand-labeled ones |
| `e2e_calibrate` | Finds the similarity and margin thresholds (global and per class) on those real crops |

**Gated recall** is recall@1 excluding `known_limitation_classes` (cutlery, kitchenware and
cans, which only confuse each other and are already handled by `yolo_finetuned`). The gate
is gated recall@1 ≥ 80% and unknown-rejection ≥ 80%. `embeddings` prints PASS or FAIL per
backbone; `boxes` prints `GATE PASSED` or `GATE NOT MET`.

## Running

```bash
./run.sh                                      # interactive task menu
./run.sh --tasks boxes,embeddings             # Phase 0 + Phase 1 on data/
./run.sh --tasks embeddings --backbones dinov2_vitb14
./run.sh --tasks e2e_eval --source /path/to/RCW2026_v2 --n-images 150
./run.sh --tasks e2e_calibrate --source /path/to/RCW2026_v2 --n-images 149
./run.sh --tasks e2e_calibrate --from-cache   # re-optimize without re-embedding
./run.sh experiment finetune-head             # optional, not used in production
./run.sh experiment finetune-arcface --source /path/to/RCW2026_v2   # or --from-cache
./run.sh --help                               # every flag
```

Needs Python with `torch` and `timm` (plus `ultralytics`/`cv2` for the box tasks, `clip` for
CLIP backbones): run it in the vision container, or set `EMBEDDING_PYTHON`. A `.venv/` in this
folder is picked up automatically.

Python runs from `~/.cache/embedding_gallery` (override with `EMBEDDING_WORKDIR`), so files
ultralytics downloads into the cwd, such as the 570 MB `mobileclip_blt.ts`, stay out of the
repo. Relative `--source`, `--data` and `--results-dir` are resolved first; use absolute paths
with `prepare` and `experiment`.

Measured on a Jetson Orin (GPU): `e2e_eval` (150 images) and `e2e_calibrate` (149 images) take
about 15-20 minutes each, and `finetune-arcface` (400 train images) about 35. On a laptop CPU
they are several times slower.

## Layout

| File | Role |
|---|---|
| `run.sh` | Entry point: picks Python, sets `PYTHONPATH`, runs tasks |
| `tasks.py` | `TASK_REGISTRY`: `boxes`, `embeddings`, `e2e_eval`, `e2e_calibrate` |
| `report.py` | Terminal tables and `results/*.json` |
| `lib/metrics.py` | IoU, gated recall, rejection, global / per-class threshold search |
| `lib/embed.py` | Embeds `data/` crops and real proposer crops (+ crop cache) |
| `lib/proposers.py` | Production box proposer and the Phase 0 candidates |
| `lib/dataset.py` | Paths, gate targets, `dataset_config.json` loader |
| `lib/prepare_dataset.py` | YOLO-seg export → `data/` |
| `experiments/` | Optional fine-tunes (`finetune_head`, `finetune_arcface`) |

## Configuration

- `models.json`: `box_proposers` (Phase 0 candidates) and `backbones` (timm id, dim, optional `img_size`).
- `dataset_config.json`: `out_of_gallery_classes`, `hard_negative_classes`, `known_limitation_classes`. The only place dataset-specific class names live; see the docstring in `lib/dataset.py`.

## Data

```
data/
  box_recall/             # Phase 0: images + annotations.json
  gallery_photos/<obj>/   # Phase 1: enrollment crops
  held_out/               # Phase 1: recall@1 eval crops + annotations.json
  hard_negatives/         # Phase 1: look-alike crops + annotations.json
  out_of_gallery/         # Phase 1: crops of objects not in the gallery
```

`gallery_photos/` must not overlap `held_out/`, or recall@1 measures memorization. All
`RCW2026_v2` images come from one capture session, so treat the numbers as optimistic.

## Results

Written to `results/` (gitignored):

- `benchmark_<ts>.json` (`embeddings`) and `thresholds.json` when a backbone passes the gate. Nothing reads it: the live defaults are `DEFAULT_MIN_SIMILARITY` / `DEFAULT_MARGIN_MIN` in `gallery_matcher.py`, which `gallery_build.py` writes into each new object's `manifest.json`.
- `box_recall.json` (`boxes`).
- `e2e_eval_<ts>.json`, `e2e_calibrate_perclass_<ts>.json`, `e2e_crops_cache.npz`, and `e2e_thresholds_perclass.json` when per-class thresholds win. The last one is a different, non-interchangeable file from `thresholds.json`; production does not load it either, so per-class values have to be copied into `gallery/manifest.json` by hand.

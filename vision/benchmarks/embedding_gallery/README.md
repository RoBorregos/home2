# embedding_gallery benchmark

Benchmark behind the few-shot object recognition in `ObjectDetect2D`: it chose
the box proposer and the DINOv2 backbone and calibrated the match thresholds.
Same shape as `hri/benchmarks/{nlp,stt}/`: `run.sh` → `tasks.py` (task
registry) → `report.py` (tables and JSON), with `models.json` as the registry.

Background, results and the production workflow (adding an object, swapping the
detector) are in [`docs/ai/embedding_gallery.md`](../../../docs/ai/embedding_gallery.md).

## Running

```bash
./run.sh                                      # interactive task menu
./run.sh --tasks boxes,embeddings             # Phase 0 + Phase 1 on data/
./run.sh --tasks embeddings --backbones dinov2_vitb14
./run.sh --tasks e2e_eval --source /path/to/export --n-images 150
./run.sh --tasks e2e_calibrate --source /path/to/export --n-images 149
./run.sh --tasks e2e_calibrate --from-cache   # re-optimize without re-embedding
./run.sh prepare --source /path/to/export     # build data/ from a YOLO-seg export
./run.sh experiment finetune-head             # optional, not used in production
./run.sh experiment finetune-arcface --from-cache
./run.sh --help                               # every flag
```

Needs Python with `torch` and `timm` (plus `ultralytics`/`cv2` for the box
tasks, `clip` for CLIP backbones): run it in the vision container, or set
`EMBEDDING_PYTHON`. A `.venv/` in this folder is picked up automatically.

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

All gitignored. `gallery_photos/` must not overlap `held_out/`, or recall@1
measures memorization. All `RCW2026_v2` images come from one capture session,
so treat the numbers as optimistic.

## Results

Written to `results/` (gitignored):

- `benchmark_<ts>.json` (`embeddings`) and `thresholds.json` when a backbone passes the gate, which seeds the defaults in `gallery_build.py`.
- `box_recall.json` (`boxes`).
- `e2e_eval_<ts>.json`, `e2e_calibrate_perclass_<ts>.json`, `e2e_crops_cache.npz`, and `e2e_thresholds_perclass.json` when per-class thresholds win. The last one is a different, non-interchangeable file from `thresholds.json`.

The gate is recall@1 ≥ 80% and unknown-rejection ≥ 80%, excluding
`known_limitation_classes`.

Python runs from `~/.cache/embedding_gallery` (override with
`EMBEDDING_WORKDIR`) so files ultralytics downloads into the cwd, such as the
570 MB `mobileclip_blt.ts`, stay out of the repo. Relative `--source`, `--data`
and `--results-dir` are resolved first; use absolute paths with `prepare` and
`experiment`.

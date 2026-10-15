# Embedding gallery (few-shot object recognition)

A class-agnostic box proposer finds candidates, a frozen DINOv2 backbone embeds
each crop, and cosine similarity against a per-object gallery decides the
label. Adding an object needs photos, not retraining.

The benchmark that chose the backbone and calibrated the thresholds lives in
`vision/benchmarks/embedding_gallery/` (see its [README](../../../../benchmarks/embedding_gallery/README.md) for how to run it). This
page holds the background and the production workflow.

- **Adding an object?** → [Production workflow](#production-workflow-adding-an-object)
- **Swapping `yolo_finetuned`?** → [Portability](#portability)
- Everything else is background on how the defaults were chosen.

## How it works

![Setup and runtime flow for the embedding gallery](../../../../../docs/ai/diagrams/embedding_gallery_process.png)

`add_object.sh` lives in this folder and the Python code (`gallery_build.py`, `gallery_matcher.py`, `constants.py`) in `core/`; `image_embedder.py` is shared from `vision_general/scripts/utils/models/`. `embedding.py`, `yolo_e.py` and `registry.py` stay in `../detectors/` (the node's plugin layer, where `EmbeddingModel` registers as a `DetectorModel`), and `fetch_models.py` is `vision/scripts/fetch_models.py`.

| File | Role |
|---|---|
| `add_object.sh` | Entry point for setup: takes object names (or `--all`), calls `gallery_build.py` then `fetch_models.py`. |
| `gallery_build.py` | Crops the main object out of each enrollment photo (same box proposer + oversized-box filter as runtime) and writes `gallery/<object>.npy` + `manifest.json`. |
| `fetch_models.py` | `sync_gallery()` copies the freshly-built gallery into every `detectors/` directory found (source, `install/`, other checkouts) — without this, a node reading a different copy never sees the new object. |
| `embedding.py` | `EmbeddingModel` — the runtime detector. Calls the box proposer, applies the `max_box_area_frac` clutter filter, batches the backbone forward pass, and turns matches into `Detection`s. |
| `yolo_e.py` | `YoloEModel` — wraps YOLOE in prompt-free mode as the class-agnostic box proposer. |
| `image_embedder.py` | `ImageEmbedder` — loads the frozen DINOv2 ViT-B/14 (TensorRT-accelerated: 8 crops take 209 ms instead of 932 ms on the Orin, 0.99995 cosine similarity vs PyTorch) and embeds a batch of crops. |
| `gallery_matcher.py` | `Gallery` — cosine similarity against the gallery with a per-class floor + top1-vs-top2 margin; also owns `DEFAULT_MAX_BOX_AREA_FRAC`. |
| `registry.py` | Wires `embedding_gallery` in `MODEL_CONFIGS`, loaded by the node at startup. |

The two phases share only the files `gallery_build.py` writes and `EmbeddingModel.load()` reads at startup.

**Oversized boxes:** the proposer sometimes returns a box covering most of the frame, which can match a small gallery with a deceptively high score (0.79 vs. 0.55 for the correct box, seen with `screwdriver`). `EmbeddingModel` drops boxes covering more than `max_box_area_frac` of the frame (default `0.5`, overridable per model in `registry.py`) before embedding them.

## Results

Benchmarked on `RCW2026_v2` (the training set behind `robocup2026_v1.pt`; 28 raw classes → 26 published after `robocup2026_translation.json`). `core/prepare_dataset.py` in the benchmark folder slices it into `data/`.

**Phase 0: box proposer** (IoU 0.5, 25 held-out images / 104 boxes):

| Candidate | Recall@0.5 | FP/image |
|---|---|---|
| `yolo_generic` (yolo26n, agnostic_nms) | 8.7% | 1.16 |
| YOLOE broad text prompt | 4.8% | 0.0 |
| YOLOE prompt-free, conf 0.25 | 89.4% | 10.92 |
| **YOLOE prompt-free, conf 0.10** (chosen) | **97.1%** | 22.76 |

The high FP rate is acceptable: the matcher's job is to reject them as unknown.

These Phase 0 figures come from the `data/box_recall/` generated at the time. Re-running the benchmark on a regenerated `data/` (103 boxes) gave 87.4% for conf 0.25 and 92.2% for conf 0.10, with the same ranking of candidates, so expect the absolute numbers to move with the sampled images.

**Phase 1: backbone** (oracle crops: 575 gallery / 690 held-out / 100 hard-negative / 45 out-of-gallery):

| Backbone | Recall@1 | Hard-neg precision | Unknown-rejection |
|---|---|---|---|
| DINOv2 ViT-S/14 | 76% | 77% | 76% |
| **DINOv2 ViT-B/14** | **76-81%** (~84% gated) | 85-94% | 80-84% |
| DINOv2 ViT-L/14 | 70% | 85% | 82% |
| CLIP ViT-B/32 | 62% | 64% | 56% |
| DINOv2-B + fine-tuned head | 79% | 94% | 82% |

**Chosen: frozen DINOv2 ViT-B/14.** The triplet fine-tuned head (the benchmark's `experiments/finetune_head.py`) adds ~3pt recall, not worth the training/versioning cost; it stays as an optional tool.

**Real proposer crops vs. oracle crops** (`e2e_eval` and `e2e_calibrate` tasks). Real boxes are looser than hand-labeled ones, so recall drops; thresholds in `gallery_matcher.py` (`DEFAULT_MIN_SIMILARITY`/`DEFAULT_MARGIN_MIN`) are calibrated on real crops. That file is the source of truth for live values.

| Config | Recall (gated) | Unknown-rejection |
|---|---|---|
| Oracle crops, global threshold | ~82% | ~84% |
| Real crops, oracle threshold (0.5/0.02) | 75-76% | 76-79% |
| Real crops, recalibrated (0.4/0.04), **production default** | 83.0% | 80.6% |
| Real crops, per-class thresholds | 84.9% | 84.7% |
| Real crops, ArcFace head + per-class thresholds | 92.0% | 91.7% |

The ArcFace head (the benchmark's `experiments/finetune_arcface.py` → its `results/arcface_head.pt`) is the only config that meets the original targets (recall ≥90%, rejection ≥80%), but it is **not wired into production**. To adopt it, load the head in `embedding.py` and project gallery and query embeddings through it before matching.

## Acceptance gate

The original target (recall@1 ≥ 90%, rejection ≥ 80%) was not reached with a frozen backbone. The adjusted gate is **recall@1 ≥ 80%, rejection ≥ 80%**, excluding classes that are only confused with each other and are already handled by `yolo_finetuned`: cutlery (fork/knife/spoon), kitchenware (cup/bowl/plate), cans (coke/red_bull). Other weak classes (e.g. `milk`, ~37%) are not excluded. The numbers live in the benchmark's `core/dataset.py` (`RECALL_TARGET`, `REJECTION_TARGET`) and its `config/dataset_config.json` (`known_limitation_classes`).

**Caveat:** all `RCW2026_v2` images come from one capture session, so `held_out/` is a different-*frame* split, not a different-*session* split. Treat the numbers as optimistic (same backdrop and lighting as the gallery); a perceptual-hash check found ~3% train/test frame overlap.

## Production workflow: adding an object

**Where the photos go.** One folder per object, named after it (the name becomes the label).
`gallery_photos/` is gitignored and does not exist on a fresh clone, so create it:

| Where | Path |
|---|---|
| Repo (host) | `vision/packages/object_detector_2d/scripts/embedding_gallery/gallery_photos/<object_name>/` |
| Inside `home2-vision` | `/workspace/src/vision/packages/object_detector_2d/scripts/embedding_gallery/gallery_photos/<object_name>/` |

The repo is bind-mounted into the container (`../../:/workspace/src`), so photos copied into the
host path show up inside the container without `docker cp`.

```
embedding_gallery/
├── add_object.sh
├── core/                        # the Python code
└── gallery_photos/
    └── ps5_controller/          # <object_name>
        ├── 001.jpg              # 10-30 photos
        ├── 002.jpg
        └── _crops/              # created by add_object.sh: the crop taken from each photo
```

**Capture tips:** use the robot camera (not a phone), arena-like lighting, at least 4 angles, 2-3 distances, 2-3 shots with occlusion or clutter, 10-30 photos total.

Then, inside the `home2-vision` container:

```bash
cd /workspace/src/vision/packages/object_detector_2d/scripts/embedding_gallery
mkdir -p gallery_photos/<object_name>    # then copy the photos in (.jpg, .jpeg or .png, any letter case)
./add_object.sh <object_name>            # or several names, or --all for every folder
```

Look at `gallery_photos/<object_name>/_crops/` afterwards: if a crop is not the object, retake that photo with the object front and centre.

Then restart the node. No code change or rebuild is needed (`embedding_gallery` is already in `object_detector_2d/config/parameters.yaml`). Takes ~30 s on the Orin. After a fresh setup the first node start also builds the TensorRT engine, which takes several minutes: run `./run.sh vision --warmup` beforehand to avoid it.

`add_object.sh`:
1. Builds the entry into `$TENSORRT_CACHE_DIR/gallery` (default `/workspace/trt_cache/gallery`, on the host `docker/vision/trt_cache/gallery/`), a mount that persists across containers and fresh clones. `gallery/` in the source tree is gitignored, so writing there would leave a fresh checkout with zero objects.
2. Runs `fetch_models.py`, whose `sync_gallery()` copies the gallery next to every `detectors/registry.py` it finds (source, `install/`, other checkouts).

The backbone is always `MODEL_CONFIGS["embedding_gallery"]["backbone"]` in `registry.py` (there is no option to change it), and the script refuses to write if the embedding dimension differs from existing objects. With several objects, one failure does not stop the rest: they are still built and synced, and the script exits with an error listing the ones that failed. `--no-crop` embeds the photos as they are, only for photos that are already tight crops.

**Gotchas:**
- `fetch_models.py` may exit 1 even when the gallery sync worked (it also checks unrelated custom weights). `add_object.sh` ignores that code; look for `[sync]  gallery/...` lines instead.
- If a node reads a `detectors/` directory that `fetch_models.py` doesn't scan, it starts normally, logs `gallery=N objects`, and the new object silently never appears. Check what is really loaded by pointing the camera at the object and running `ros2 topic echo /vision/detections`.

## Portability

**Swapping `yolo_finetuned`:** `embedding_gallery` is independent of it (own proposer, own gallery). Only `registry.py` references the model:

```python
"yolo_finetuned": {
    "filename": "robocup2026_v1.pt",                # swap this
    "type": "yolo",
    "conf": 0.6,
    "translation": "robocup2026_translation.json",  # and this (or drop it)
    "use_trt": True,
},
```

Drop the new `.pt` beside `registry.py`, update `filename`, and write a new translation JSON (raw class → published label) or remove the key. Thresholds in `gallery_matcher.py` and `known_limitation_classes` in the benchmark's `config/dataset_config.json` were tuned on RCW2026_v2; treat them as a starting point and re-run the `embeddings` and `e2e_calibrate` tasks on your own data.

**Using both:** to keep the old model too, add a second entry in `MODEL_CONFIGS` instead of replacing this one, and list both under `models:` in `object_detector_2d/config/parameters.yaml`. `ObjectDetect2D` runs every listed model and IoU-dedupes across them (threshold 0.6), which is how `yolo_finetuned` and `embedding_gallery` already run together.

**Swapping the benchmark dataset:** run `./run.sh prepare --source` on the new export and edit the benchmark's `config/dataset_config.json`, the only place dataset-specific class names live (`out_of_gallery_classes`, `hard_negative_classes`, `known_limitation_classes`; see "Configuration" in the [benchmark README](../../../../benchmarks/embedding_gallery/README.md)). `known_limitation_classes` can only be found from a run's confusion breakdown.

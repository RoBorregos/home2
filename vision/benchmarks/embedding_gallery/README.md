# embedding_gallery benchmark

Benchmark behind the few-shot object recognition in `ObjectDetect2D` (objects added
from photos, no retraining). It chose the box proposer and the DINOv2 backbone and
calibrated the match thresholds. Same shape as `hri/benchmarks/{nlp,stt}/`: `run.sh` →
`core/tasks.py` (task registry) → `core/report.py` (tables and JSON), with `config/models.json`
as the registry.

- **Want to add an object to the robot?** That is not this folder: see
  [Adding an object to the gallery](../../README.md#adding-an-object-to-the-gallery-few-shot)
  in `vision/README.md`.
- Background, accuracy numbers and troubleshooting:
  [`embedding_gallery/README.md`](../../packages/object_detector_2d/scripts/embedding_gallery/README.md).

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
| `core/tasks.py` | Entry point run by `run.sh`; `TASK_REGISTRY`: `boxes`, `embeddings`, `e2e_eval`, `e2e_calibrate` |
| `core/report.py` | Terminal tables and `results/*.json`; `tasks.py` calls it after each task |
| `core/metrics.py` | IoU, gated recall, rejection, global / per-class threshold search |
| `core/embed.py` | Embeds `data/` crops and real proposer crops (+ crop cache) |
| `core/proposers.py` | Production box proposer and the Phase 0 candidates |
| `core/dataset.py` | Paths, gate targets, `config/dataset_config.json` loader |
| `config/` | `models.json` (proposers and backbones) and `dataset_config.json` (class lists) |
| `core/prepare_dataset.py` | YOLO-seg export → `data/` |
| `experiments/` | Optional fine-tunes (`finetune_head`, `finetune_arcface`) |

## Configuration

- `config/models.json`: `box_proposers` (Phase 0 candidates) and `backbones` (timm id, dim, optional `img_size`).
- `config/dataset_config.json`: the only place dataset-specific class names live (published labels, after translation).
  - `out_of_gallery_classes`: held out of `gallery_photos/` to serve as "not in gallery" negatives for unknown-rejection.
  - `hard_negative_classes`: visually close classes curated into `hard_negatives/` (chosen, not random).
  - `known_limitation_classes`: excluded from the recall gate because they only confuse each other, never an unrelated class. Start empty and add a class only when a run's confusion breakdown shows evidence.

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

## Image and text embeddings

The shared `ImageEmbedder` supports CLIP text queries in the same embedding space
as image crops. With `vision/packages/vision_general/scripts` on `PYTHONPATH`:

```python
from utils.models.image_embedder import ImageEmbedder

embedder = ImageEmbedder("clip:ViT-B/32").load()
assert embedder.dim == 512
queries = embedder.embed_text(["a red cup", "a cereal box"], normalize=True)
images = embedder.embed_batch(crops, normalize=True)  # RGB PIL images
cosine_scores = images @ queries.T
```

Both methods return float32 arrays and default to raw embeddings. Existing
gallery callers keep that default. `dim` loads the model lazily if necessary;
timm backbones expose their image dimension but reject `embed_text`. Empty
batches return `[0, dim]` arrays. Text beyond CLIP's context limit raises an
error instead of silently truncating the query.

The CPU, CUDA and Orin vision images install the same pinned official OpenAI
CLIP revision from `vision/requirements/clip.txt`. Existing images need rebuilding.
For an independent environment, install that requirements file alongside the
appropriate torch/torchvision versions; do not install the unrelated PyPI `clip`
package. CLIP uses its standard `~/.cache/clip` weight cache: load ViT-B/32 once
while online under the same runtime user, and retain that cache for offline runs.

### Compatibility checks

From the repository root, with `pytest` and `numpy` installed:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q vision/packages/vision_general/tests
```

The API tests cover raw output preservation, normalization, backend selection,
lazy loading, batching, empty inputs and unsupported text models. Real encoder
tests run when `torch`, `timm` and official `clip` are installed. They use small
random models without downloading weights; they validate API compatibility,
not recognition accuracy. `PYTEST_DISABLE_PLUGIN_AUTOLOAD` avoids unrelated ROS
pytest plugins affecting these standalone tests.

For pretrained ViT-B/32 validation, set `CLIP_TEST_WEIGHTS` to an existing local
checkpoint. To also compare raw outputs with the exact branch baseline, export
the original module and set `IMAGE_EMBEDDER_BASELINE`:

```bash
git show 816fca5f:vision/packages/vision_general/scripts/utils/models/image_embedder.py > /tmp/image_embedder_baseline.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  CLIP_TEST_WEIGHTS="$HOME/.cache/clip/ViT-B-32.pt" \
  IMAGE_EMBEDDER_BASELINE=/tmp/image_embedder_baseline.py \
  python3 -m pytest -q vision/packages/vision_general/tests
```

This compares the baseline and candidate with identical loaded weights and
inputs. It does not replace the dataset benchmark below.

### Orin memory and latency

Run inside the rebuilt vision container with the production model weights and
a populated gallery available. Use a representative crop and the verified
power/clock settings. From this benchmark directory:

```bash
python3 profile_clip.py --image /path/to/crop.jpg --text "a red cup" \
  --batch-size 8 --warmup 5 --iterations 30 \
  --power-mode "record actual nvpmodel mode and clock settings here" \
  --output results/clip_orin_batch8.json
```

Repeat in a fresh process for batch sizes 1 and 32. The runner loads the
production `embedding_gallery` registry entry (including its box proposer),
warms its GPU embedding backend, then loads and warms CLIP. It refuses an empty
gallery or unavailable GPU backend. Reports include synchronized wall latency
(preprocessing, transfers and output conversion included), mean/p50/p95, memory
snapshots, versions, hardware, model settings and Git revision/status.

Gallery latency covers crop embedding and matching; it excludes box proposal
and ROS. Both models stay loaded, but calls are sequential. PyTorch memory
counters exclude TensorRT/ORT allocations, so CUDA free memory and system
available RAM are recorded separately. Jetson uses shared memory: do not add
these counters together. Other processes can affect the global readings.

### Gallery regression check

Run the following on the baseline and candidate in separate checkouts, using
the same environment, weights and an identical prepared `data/` directory:

```bash
./run.sh --tasks embeddings --backbones dinov2_vitb14,clip_vit_b32 \
  --results-dir /absolute/path/to/separate-results
./run.sh --tasks e2e_eval --source /absolute/path/to/RCW2026_v2 \
  --n-images 150 --seed 42 --results-dir /absolute/path/to/separate-results
```

Use distinct result directories for each revision. Compare recall@1, gated
recall, hard-negative precision, unknown rejection, selected thresholds and
per-image predictions, ignoring timestamps. Investigate any changes rather
than accepting rounded headline metrics. Do not regenerate the sampled data
between runs or use `--from-cache`, which can bypass the modified embedder.
The `--data` option only applies to the boxes task; embeddings reads this
directory's `data/`. Orin measurements and dataset regression results must be
attached before treating issue #1348's acceptance criteria as complete.

## Results

Written to `results/` (gitignored):

- `benchmark_<ts>.json` (`embeddings`) and `thresholds.json` when a backbone passes the gate. Nothing reads it: the live defaults are `DEFAULT_MIN_SIMILARITY` / `DEFAULT_MARGIN_MIN` in `embedding_gallery/core/gallery_matcher.py`, which `gallery_build.py` writes into each new object's `manifest.json`.
- `box_recall.json` (`boxes`).
- `e2e_eval_<ts>.json`, `e2e_calibrate_perclass_<ts>.json`, `e2e_crops_cache.npz`, and `e2e_thresholds_perclass.json` when per-class thresholds win. The last one is a different, non-interchangeable file from `thresholds.json`; production does not load it either, so per-class values have to be copied into `gallery/manifest.json` by hand.

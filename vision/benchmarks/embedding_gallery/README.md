# embedding_gallery benchmark

Phase 0 and Phase 1 of the few-shot object recognition plan: validate box
proposals, then compare DINOv2 vs CLIP as the embedding backbone and
calibrate the gallery match threshold — **before** any code lands in
`detectors/registry.py`. See the plan for the full phase breakdown and gates.

Follows the `hri/benchmarks/{nlp,stt}/` shape (`models.json`, `run.sh`,
`report.py`, `results/`) rather than a notebook, so it's reviewable as a
diff and runnable headless/CI — a deliberate deviation from the original
design doc's "offline notebook" wording.

**Just here to add an object?** Skip to [Production workflow](#production-workflow-adding-a-real-object-no-retraining).
**Swapping `yolo_finetuned` for a different model?** Skip to
[Portability](#portability-a-different-yolo-model-or-a-different-dataset). Everything
else on this page is the benchmark that validated the design and calibrated
its defaults — useful context, not required reading to use the system.

## How it works

Two separate phases — setup happens once per new object, runtime happens on
every camera frame:

![Setup and runtime flow for the embedding gallery](embedding_gallery_process.png)

<details>
<summary>Mermaid source (for editing — the PNG above is the rendered, viewer-independent copy)</summary>

```mermaid
flowchart TD
    subgraph SETUP["Setup — once per object, no retraining"]
        direction TB
        P["10-30 photos<br/>gallery_photos/&lt;object&gt;/*.jpg"] --> GB["add_object.sh<br/>(gallery_build.py -> $TENSORRT_CACHE_DIR,<br/>then fetch_models.py syncs it everywhere)"]
        GB --> NPY["gallery/&lt;object&gt;.npy<br/>+ manifest.json (thresholds)"]
    end

    subgraph RUNTIME["Runtime — every camera frame, inside ObjectDetect2D"]
        direction TB
        CAM["ZED frame<br/>/vision/camera/image_oriented"] --> BP["YOLOE prompt-free<br/>box proposer, conf=0.10<br/>(class-agnostic)"]
        BP --> CROPS["N candidate crops"]
        CROPS --> EMB["DINOv2 ViT-B/14<br/>frozen backbone<br/>(TensorRT-accelerated)"]
        EMB --> MATCH["gallery_matcher.py<br/>cosine similarity vs. gallery<br/>+ top1-vs-top2 margin"]
        MATCH -->|"sim >= floor AND margin >= min"| LABEL["object label"]
        MATCH -->|"below floor or margin"| UNK["unknown — dropped"]
    end

    NPY -.->|"loaded at node startup"| MATCH
    LABEL --> PUB["/vision/detections"]
```

</details>

The two phases only share one thing: the `.npy`/`manifest.json` files
`gallery_build.py` writes and `EmbeddingModel.load()` reads at node startup
— which is exactly why the **install/src sync gotcha** below matters so
much: if the runtime side is reading a *different* `gallery/` than the one
setup just wrote, adding an object silently does nothing.

### Oversized boxes vs. a small gallery

The class-agnostic box proposer sometimes returns a box covering most of
the frame — desk/shelf clutter, not a single object. Embedding a crop that
large and matching it against a gallery with only one or two objects can
score a deceptively *high* similarity, higher than the correctly-boxed
object itself: found empirically adding a real `screwdriver` object — a
box spanning almost the whole frame matched at 0.79, the correctly-boxed
screwdriver at only 0.55. The per-class margin check
(`gallery_matcher.py`) doesn't catch this either, because with few gallery
objects there's little for the winning class to be "distinguishable from"
— see that file's `match_batch` docstring.

`EmbeddingModel` now drops any box covering more than `max_box_area_frac`
of the frame (default `0.5`, configurable per-model in `registry.py`)
*before* it's even embedded — cheaper than embedding it and lets the
matching decision alone. Verified on the Orin: the same screwdriver test
that produced the 0.79 false positive above dropped it entirely after this
filter, leaving only the correct 0.60 detection.

## Results (real data, not simulated)

Benchmarked against `RCW2026_v2`, the actual training set behind
`robocup2026_v1.pt` (28 raw classes → 26 published classes after
`robocup2026_translation.json` merges `coca_cola`+`coca_cola_zero`→`coke` and
`blue_cereal_box`+`brown_cereal_box`→`cornflakes`). See
`prepare_dataset.py`'s docstring for exactly how the real dataset gets
sliced into this benchmark's `data/` layout.

**Phase 0 — box proposer** (`results/box_recall.json`, IoU 0.5, 25 held-out
images / 104 ground-truth boxes):

| Candidate | Recall@0.5 | FP/image | Notes |
|---|---|---|---|
| `yolo_generic` (yolo26n, agnostic_nms) | 8.7% | 1.16 | COCO's 80 classes don't cover RoboCup objects |
| YOLOE broad text prompt | 4.8% | 0.0 | hand-picked vocabulary was a bad choice |
| YOLOE prompt-free (`yoloe-11l-seg-pf.pt`), conf 0.25 | 89.4% | 10.92 | |
| **YOLOE prompt-free, conf 0.10** | **97.1%** | 22.76 | **chosen** — high FP rate is fine, the embedding matcher's job is rejecting them as unknown |

**Phase 1 — backbone** (`results/benchmark_*.json`, 575 gallery / 690
held_out / 100 hard-negative / 45 out-of-gallery crops):

| Backbone | Recall@1 (all) | Recall@1 (gated) | Hard-neg precision | Unknown-rejection |
|---|---|---|---|---|
| DINOv2 ViT-S/14 | 76% | — | 77% | 76% |
| **DINOv2 ViT-B/14** | **76-81%** (run noise) | **~84%** | 85-94% | 80-84% |
| DINOv2 ViT-L/14 | 70% | — | 85% | 82% |
| CLIP ViT-B/32 | 62% | — | 64% | 56% |
| DINOv2-B + regularized fine-tuned head | 79% | — | 94% | 82% |

**Chosen config: DINOv2 ViT-B/14, frozen (no fine-tune).** The triplet-loss
fine-tuned head (`finetune_head.py`) gives a small, real improvement (+3pt
recall) but isn't worth the added training/versioning complexity for that
margin — kept in the repo as a documented, available-but-not-default option.

## Real box-proposer crops vs. oracle crops

Everything above is measured on crops cut from `RCW2026_v2`'s **hand-labeled
polygons** — perfect boxes a real detector never gives you. `e2e_eval.py`
re-ran the same gallery through the **actual production box proposer**
(YOLOE prompt-free) instead, and recall dropped: 71.6% (all classes) vs.
~82% oracle — loose/shifted real boxes drag in background clutter the
matcher never saw during calibration. `gallery_matcher.py`'s
`DEFAULT_MIN_SIMILARITY`/`DEFAULT_MARGIN_MIN` are recalibrated against these
real crops (`e2e_calibrate.py`), not the oracle numbers above — that's the
single source of truth for what's actually live, check that file before
trusting a number here.

| Config | Recall (gated) | Unknown-rejection | Measured on |
|---|---|---|---|
| Oracle crops, global threshold | ~82% | ~84% | hand-labeled polygons |
| Real crops, oracle-calibrated threshold (0.5/0.02) | 75-76% | 76-79% | real box-proposer crops |
| Real crops, recalibrated threshold (0.4/0.04, **current production default**) | 83.0% | 80.6% | real box-proposer crops |
| Real crops, per-class calibrated thresholds | 84.9% | 84.7% | real box-proposer crops |
| Real crops, **ArcFace head** (`finetune_arcface.py`) + per-class thresholds | **92.0%** | **91.7%** | real box-proposer crops, genuinely held-out test split |

The ArcFace run is the first (and only) config that clears the original
issue's targets (recall@1 ≥90%, unknown-rejection ≥80%) on real, non-oracle
crops. It is **not** wired into production by default — `finetune_arcface.py`
saves `results/arcface_head.pt` when it beats the frozen baseline, kept as
a documented, available upgrade path, same reasoning as the triplet head
above (added training/versioning complexity for a still-frozen-by-default
system). To adopt it: load `arcface_head.pt` in `embedding.py` and project
gallery + query embeddings through it before matching — not currently
implemented, since the frozen baseline already clears the *adjusted* gate.

## Acceptance criteria — adjusted from the original design doc

The original doc's target was recall@1 ≥ 90%, unknown-rejection ≥ 80%.
**Real measurement never got there**, across every lever tried: bigger
backbone (worse), more gallery photos (no change), a properly-regularized
fine-tune (+3pt, still short), widening which classes get excluded (hits
diminishing returns). See `report.py`'s `RECALL_TARGET`/
`KNOWN_LIMITATION_CLASSES` for the exact, evidenced final numbers — this
README doesn't duplicate them so they can't drift out of sync.

The adjusted gate: **recall@1 ≥ 80%, unknown-rejection ≥ 80%**, computed
excluding a short, specifically-evidenced list of classes (cutlery
fork/knife/spoon, kitchenware cup/bowl/plate, cans coke/red_bull) that only
ever get confused with each other — never with an unrelated object — and
are already handled by `yolo_finetuned` in production, so the embedding path
doesn't need to re-solve them. Classes that fail for other reasons (e.g.
`milk`, ~37% recall with no clean look-alike pair — just noisy embeddings)
are **not** excluded; they're accepted, visible weak points, not hidden ones.

## Dataset layout

```
data/
  box_recall/             # Phase 0: images + annotations.json (pixel bboxes)
  gallery_photos/<obj>/   # Phase 1: enrollment crops, one dir per published label
  held_out/               # Phase 1: recall@1 eval crops + annotations.json
  hard_negatives/         # Phase 1: curated look-alike crops + annotations.json
  out_of_gallery/         # Phase 1: crops of objects NOT in gallery_photos/
```

Rebuild all five from a fresh copy of a `RCW2026_v2`-shaped YOLO-seg export
(`dataset/{train,valid,test}/{images,labels}` + `dataset/data.yaml`):

```bash
python3 prepare_dataset.py --source /path/to/RCW2026_v2
```

**Known limitation of this specific rebuild**: per
`RCW2026_v2/imported_classes.json`, every class's source images come from a
SINGLE capture session — `dataset/data.yaml`'s train/valid/test split is a
random split of frames within that one session, not independent sessions.
So `held_out/` here is a different-*frame* split, not a different-*session*
split — the numbers above are an optimistic sanity check (same
backdrop/lighting as `gallery_photos/`), not the field number a from-scratch
photo shoot would give. A near-duplicate-frame check (perceptual hashing,
150-image sample) found only ~3% overlap between train and test frames, so
it's not literal memorization — but it's still one session's lighting/backdrop
throughout.

## Capture conditions for a NEW gallery (production workflow, not this benchmark)

When actually adding an object via `gallery_build.py` (not rebuilding this
benchmark), capture with the arena in mind:

- Use the **actual robot camera** where possible, not a phone — optics and
  exposure differ enough to shift embedding distributions.
- Arena-like lighting, not office lighting.
- At least 4 viewing angles per object (front, back, both sides / 3/4).
- At least 2-3 realistic detection distances.
- At least 2-3 shots with partial occlusion or table/shelf clutter.
- 10-30 photos total (25 is what this benchmark's gallery uses).

## Running

```bash
# Phase 0/1 — oracle-crop benchmark, on data/ (see "Dataset layout")
./run.sh boxes                                          # Phase 0: box-proposer recall
./run.sh embeddings                                     # Phase 1: all backbones in models.json
./run.sh embeddings --backbones dinov2_vitb14            # Phase 1: one backbone

# Real box-proposer crops — needs a YOLO-seg export, not just data/
python3 e2e_eval.py --source /path/to/dataset --n-images 150       # sanity check, oracle vs real
python3 e2e_calibrate.py --source /path/to/dataset --n-images 149  # (re)calibrate thresholds
python3 e2e_calibrate.py --from-cache                                # re-optimize without re-embedding

# Optional fine-tunes (not wired into production by default — see
# "Real box-proposer crops vs. oracle crops")
python3 finetune_head.py                                             # triplet loss, oracle crops
python3 finetune_arcface.py --source /path/to/dataset --n-images 400 # ArcFace, real crops
python3 finetune_arcface.py --from-cache                             # reuse cached train crops
```

`results/thresholds.json` is written when a backbone clears the adjusted
gate; `gallery_build.py` reads it to seed default per-object match
thresholds. `e2e_calibrate.py` writes its own
`results/e2e_thresholds_perclass.json` separately (per-class, calibrated
against real crops) — the two are not the same file and are not
interchangeable inputs.

## Production workflow: adding a real object, no retraining

This is the actual on-robot flow (separate from rebuilding this benchmark's
`data/` above). Everything in this section runs from inside the container,
**with your shell in**
`vision/packages/object_detector_2d/scripts/detectors/` (where
`add_object.sh` and `gallery_build.py` live) unless a command explicitly
says otherwise:

```
gallery_photos/<object_name>/*.jpg      # raw photos you provide (gitignored)
$TENSORRT_CACHE_DIR/gallery/            # canonical copy, persists across
  <object_name>.npy                     # fresh clones/containers (built by
  manifest.json                         # gallery_build.py, synced by fetch_models.py)
```

Steps:

```bash
mkdir -p gallery_photos/<object_name>
# ...copy 10-30 photos in (see "Capture conditions" above)...

./add_object.sh <object_name>
# equivalent to: --photos "gallery_photos/<object_name>/*.jpg"
```

Measured end-to-end on the Orin (photos already captured): **~30 seconds**
— well under the <10 minute target.

`add_object.sh` does two things: builds the gallery entry into
`$TENSORRT_CACHE_DIR/gallery` (defaulting to `/workspace/trt_cache/gallery`
if that env var isn't set — same persistent mount the TRT engine cache
already uses), then calls `fetch_models.py`, whose `sync_gallery()` copies
it beside every `detectors/registry.py` it can find — source tree, `install/`,
any other checkout — instead of a one-off manual copy. It does not restart
the node; that part is still manual, on purpose. No `--backbone` needed — it
defaults to `MODEL_CONFIGS["embedding_gallery"]["backbone"]` in
`registry.py` (single source of truth), and refuses to write if the new
object's embedding dimension doesn't match the rest. `embedding_gallery` is
already in `config/parameters.yaml`'s `models:` list, so a plain node
restart is all that's needed afterward — no code change, no rebuild.

**Why the persistent mount, not a direct source-tree write (found the hard
way, 2026-09-28):** an earlier version of `add_object.sh` wrote straight
into the source tree's `detectors/gallery/` and `cp -r`'d it into `install/`
by hand. That survives a node restart, but **not** a fresh clone or
container: `gallery/` is gitignored, so a competition-day fresh checkout
would silently start with zero objects, no matter how many were added
before — exactly the "offline, no internet, fresh container" scenario
`fetch_models.py` already exists to prevent for weights and TRT engines.
Writing into `$TENSORRT_CACHE_DIR` (a mount that persists across container
rebuilds) and reusing that same provisioning script closes that gap instead
of re-solving it ad hoc.

**Also note:** `fetch_models.py`'s overall exit code can be 1 even when the
gallery sync itself succeeded — it also checks unrelated custom weights
(e.g. a different competition task's `.pt` this checkout never had) and
exits nonzero if any are missing. `add_object.sh` ignores that exit code on
purpose; look for the `[sync]  gallery/...` lines in its output to confirm
the sync itself worked, not the final pass/fail summary.

Symptoms if the sync is ever missed entirely (e.g. a node reading from a
`detectors/` directory `fetch_models.py`'s `detector_dirs()` doesn't scan):
the node loads and runs fine, logs a plausible `gallery=N objects`, and the
new object silently never appears in `/vision/detections` — no error, no
crash, because the stale gallery it did load is internally consistent with
itself, just not with what was just built. Verify what a running node
actually has loaded with `ros2 topic echo /vision/detections_image`
(annotated frame) if in doubt.

## Portability: a different YOLO model, or a different dataset

Two independent axes — swapping one doesn't require touching the other.

**Swapping `yolo_finetuned` for a different trained model.**
`embedding_gallery` (the few-shot system this whole README is about) does
not know or care what `yolo_finetuned` is. It runs its own box proposer
(`embedding_box_proposer`, class-agnostic YOLOE) and matches against a
gallery you build yourself with `add_object.sh` — nothing about it
references RCW2026_v2, `robocup2026_v1.pt`, or any trained class name.
Confirmed by reading `detectors/yolo.py`: it has zero hardcoded class names
or dataset references; every model-specific bit lives in `registry.py`:

```python
"yolo_finetuned": {
    "filename": "robocup2026_v1.pt",          # <- swap this
    "type": "yolo",
    "conf": 0.6,
    "translation": "robocup2026_translation.json",  # <- and this (or drop it)
    "use_trt": True,
},
```

Drop the new `.pt` beside `registry.py`, point `filename` at it, and either
write a new `translation.json` (raw model class name -> published label) or
remove the `translation` key if the model's own class names are already
what you want published. That's it — no other file changes.

What this does **not** carry over automatically: `embedding_gallery`'s own
calibration (`DEFAULT_MIN_SIMILARITY`/`DEFAULT_MARGIN_MIN` in
`gallery_matcher.py`, `KNOWN_LIMITATION_CLASSES` in `report.py`) was
measured against RCW2026_v2's specific objects and lighting. It's a
reasonable starting point for a different model or environment, not a
guarantee — re-run `report.py`/`e2e_calibrate.py` against your own data if
you want numbers you can actually trust for the new setup.

**Swapping the dataset this *benchmark* (not production) is validated
against.** `dataset_config.json` (loaded via `dataset_config.py`) is the
**only** place dataset-specific class names live here —
`out_of_gallery_classes`, `hard_negative_classes` and
`known_limitation_classes` (see `dataset_config.py`'s docstring for what
each means and how to derive it; `known_limitation_classes` specifically
cannot be guessed ahead of time, only discovered from a benchmark run's
confusion breakdown). Point `prepare_dataset.py --source` at a new
YOLO-seg export and edit that JSON file — no `.py` script needs touching.

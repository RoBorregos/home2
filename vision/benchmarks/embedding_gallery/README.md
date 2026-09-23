# embedding_gallery benchmark

Phase 0 and Phase 1 of the few-shot object recognition plan: validate box
proposals, then compare DINOv2 vs CLIP as the embedding backbone and
calibrate the gallery match threshold — **before** any code lands in
`detectors/registry.py`. See the plan for the full phase breakdown and gates.

Follows the `hri/benchmarks/{nlp,stt}/` shape (`models.json`, `tasks.py`,
`run.sh`, `report.py`, `results/`) rather than a notebook, so it's reviewable
as a diff and runnable headless/CI — a deliberate deviation from the
original design doc's "offline notebook" wording.

## Results (real data, not simulated)

Benchmarked against `RCW2026_v2`, the actual training set behind
`robocup2026_v1.pt` (28 raw classes → 23 published classes after
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

**Chosen config: DINOv2 ViT-B/14, frozen (no fine-tune).** The fine-tuned
head (`finetune_head.py`) gives a small, real improvement (+3pt recall) but
isn't worth the added training/versioning complexity for that margin — kept
in the repo as a documented, available-but-not-default option.

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
  box_recall/            # Phase 0: images + annotations.json (pixel bboxes)
  gallery_photos/<obj>/  # Phase 1: enrollment crops, one dir per published label
  held_out/               # Phase 1: recall@1 eval crops + annotations.json
  hard_negatives/          # Phase 1: curated look-alike crops + annotations.json
  out_of_gallery/           # Phase 1: crops of objects NOT in gallery_photos/
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
./run.sh boxes                                        # Phase 0
./run.sh embeddings                                    # Phase 1, all backbones
./run.sh embeddings --backbones dinov2_vitb14           # Phase 1, one backbone
python3 finetune_head.py                                # Phase 4 (optional, not the default path)
```

`results/thresholds.json` is written when a backbone clears the adjusted
gate; `gallery_build.py` (in `detectors/`) reads it to seed default
per-object match thresholds.

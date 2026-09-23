# embedding_gallery benchmark

Phase 0 and Phase 1 of the few-shot object recognition plan: validate box
proposals, then compare DINOv2 vs CLIP as the embedding backbone and
calibrate the gallery match threshold — **before** any code lands in
`detectors/registry.py`. See the plan for the full phase breakdown and gates.

Follows the `hri/benchmarks/{nlp,stt}/` shape (`models.json`, `tasks.py`,
`run.sh`, `report.py`, `results/`) rather than a notebook, so it's reviewable
as a diff and runnable headless/CI — flagged as a deliberate deviation from
the original design doc's "offline notebook" wording.

## What's blocked on real data

Nothing here can produce a real number without actual photos of the
competition objects. All of `data/` is currently empty scaffolding
(`.gitkeep` only). Until it's filled in, `box_recall_eval.py` and
`report.py` will refuse to run (they raise with a message pointing back
here) rather than fabricate results.

## Dataset layout to provide

```
data/
  box_recall/                      # Phase 0
    image_001.jpg, image_002.jpg, ...
    annotations.json               # {"image_001.jpg": [{"bbox":[x1,y1,x2,y2],"label":"coke"}, ...]}

  gallery_photos/                  # Phase 1 — enrollment set (builds the temp gallery)
    coke/*.jpg
    mug/*.jpg
    ...

  held_out/                        # Phase 1 — recall@1 (MUST NOT overlap gallery_photos/)
    image_010.jpg, ...
    annotations.json               # {"image_010.jpg": "coke", ...}

  hard_negatives/                  # Phase 1 — curated visually-similar pairs
    image_020.jpg, ...             # e.g. a fork photo, a spoon photo, a mug vs cup, ...
    annotations.json               # {"image_020.jpg": "fork", ...}

  out_of_gallery/                  # Phase 1 — objects NOT in gallery_photos/, any filenames
    image_030.jpg, ...             # no annotations.json needed — must all come back "unknown"
```

## Capture conditions (per critique point 6 of the plan)

Numbers measured on clean setup-day photos will not transfer to the arena.
For every directory above, capture with these conditions in mind:

- Use the **actual robot camera** where possible, not a phone — optics and
  exposure differ enough to shift embedding distributions.
- Arena-like lighting, not office lighting.
- At least 4 viewing angles per object (front, back, both sides / 3/4).
- At least 2-3 realistic detection distances.
- At least 2-3 shots with partial occlusion or table/shelf clutter.
- `held_out/` and `hard_negatives/` crops should come from *different*
  physical photo sessions than `gallery_photos/`, not just different frames
  of the same session — otherwise recall@1 measures memorization of lighting
  conditions, not generalization.

## Running

```bash
./run.sh boxes                                      # Phase 0
./run.sh embeddings                                  # Phase 1, all backbones
./run.sh embeddings --backbones dinov2_vits14         # Phase 1, one backbone
```

`results/thresholds.json` is only written if some backbone clears both
acceptance targets (recall@1 >= 90%, unknown-rejection >= 80%). `gallery_build.py`
(in `detectors/`) reads those defaults when it builds a real gallery.

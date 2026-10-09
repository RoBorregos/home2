# Issue #1348 validation

Branch base: `816fca5f` (fetched `origin/main`). Implementation commit:
`e0af9ead` on `codex/1348-text-embeddings-from-main`. Remote logs include a
subsequent source-path fix to `fetch_models.py`. The transferred source snapshots
and diff are retained in [the evidence bundle](validation/README.md).

## Local checks completed

Environment: x86_64 CPU, Python 3.10.12, NumPy 1.26.4, PyTorch 2.6.0+cpu,
torchvision 0.21.0+cpu, timm 1.0.30, official OpenAI CLIP revision
`d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`.

- 16 tests passed with real model dependencies, a local pretrained ViT-B/32
  checkpoint, and the exported baseline module enabled.
- Pretrained ViT-B/32 image and text outputs are `[2, 512]` float32 and match
  direct native encoder calls. Normalized outputs have unit norm; their cosine
  scores match normalization of the native outputs.
- Raw image outputs match the original baseline exactly for the tested inputs
  with shared weights: pretrained ViT-B/32, a small random CLIP architecture,
  and a random DINOv2 ViT-S/14 at 28px. These synthetic inputs validate numerical
  compatibility, not recognition accuracy or production-resolution latency.
- Unit tests cover image backend selection (including missing-session fallback),
  text batching/order, lazy loading, zero-vector normalization, input errors,
  dimensions, empty batches, timing synchronization and separate memory counters.
- The minimal host environment passes 13 tests and skips the three real-model
  tests when their dependencies/checkpoint are unavailable.
- Ruff 0.8.4 lint and formatting, Python compilation, `pip check`, and
  `git diff --check` pass.
- The profiling CLI help works and CPU-only execution is rejected before model
  loading. No Orin report was produced.

ViT-B/32 checkpoint SHA-256:
`40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`, matching the
checksum in the official CLIP model URL.

## Orin checks reported by the user (2026-10-08)

The [saved compatibility transcript](validation/orin_compatibility_2026-10-08.txt)
records **16 passed, 2 warnings in 8.96 seconds**, with no skipped tests, on
Python 3.12.3 / pytest 7.4.4. The command enables the local pretrained CLIP
checkpoint and the exported baseline comparison. This is user-supplied remote
execution evidence, not a local rerun. The transcript does not record its Git SHA.

The warnings concern PyTorch's Orin compute-capability support and upstream
`torch.jit.load` deprecation. Neither prevented the tested operations from
passing; this does not establish compatibility for every GPU operation.

Earlier user-provided logs also report a successful vision-l4t image build,
completed embedding warmup, and completed profiling runs with five gallery
objects and CLIP resident together. Batch sizes 1, 8 and 32 were measured under
MAXN mode 0 with dynamic CPU/GPU/EMC clocks. Those runs exercise the production
gallery session separately from the unit tests' mocked backend checks. The full
final profiling JSON files have now been transferred and verified in
`validation/orin_evidence/performance/`.

| Batch | Gallery before / after CLIP residency, mean ms | CLIP images mean / p95 ms | CLIP text mean / p95 ms | Peak PyTorch allocated MiB |
| --- | --- | --- | --- | --- |
| 1 | 30.73 / 25.98 | 28.74 / 29.93 | 25.85 / 26.35 | 718.0 |
| 8 | 181.36 / 180.14 | 49.66 / 51.29 | 28.00 / 28.57 | 742.2 |
| 32 | 779.12 / 780.92 | 128.65 / 131.65 | 31.01 / 31.45 | 784.7 |

CLIP added 385.4 MiB of live PyTorch allocations in each run. Counters exclude
TensorRT/ORT allocations. Timings are per batch with sequential calls and both
models resident, using repeated copies of one image/query. Dynamic clocks and
execution order prevent attributing the batch-1 improvement to CLIP.

## Gallery accuracy regression reported by the user

The [comparison output](validation/orin_accuracy_comparison.txt) reports that
prepared evaluation-file checksums were unchanged and both complete result JSON
payloads were identical after removing only the top-level timestamp. This
includes metrics, selected thresholds and every recorded prediction, similarity
and margin. Stored scores/metrics are rounded by the benchmark; this is not an
assertion of bitwise equality of all raw embeddings. After transfer, the JSON
equality was also independently verified locally from the complete baseline and
candidate files in `validation/orin_evidence/accuracy/`.

The existing `embeddings` task ran on DINOv2 ViT-B/14 and CLIP ViT-B/32. Baseline
used the module exported from `816fca5f`; candidate used the source module with
`embed_text`. Both processes used the same benchmark harness/configuration and
`PYTHONHASHSEED=1348`, recomputing embeddings rather than loading cached results.

The available RCW2026_v2 export lacked its original validation split. An
alternative split was generated once with seed 1348: 319 training, 80 validation
and 149 unchanged test images. Selection required at least 10 images per class
in validation and 25 in training, and rejected byte-identical duplicate source
images. The original export was not modified. Source copies and their manifest
were stored at `/tmp/issue1348-eval-mf1160be/RCW2026_v2` on the Orin.

Each backbone evaluated 575 gallery crops, 542 held-out crops, 100 hard negatives
and 45 out-of-gallery crops. The console results were identical:

| Backbone | Recall@1 all / gated | Hard-negative precision | Unknown rejection | Similarity / margin threshold |
| --- | --- | --- | --- | --- |
| DINOv2 ViT-B/14 | 54% / 67% | 95% | 69% | 0.5 / 0.1 |
| CLIP ViT-B/32 | 56% / 62% | 54% | 53% | 0.85 / 0.0 |

Both revisions miss the accuracy gate. This demonstrates no measured regression
on this alternative split, not achievement of the recognition accuracy target
or reproduction of historical results. Same-session data limits generalization.
The console's 90% recall gate message is inconsistent with the code's 80% gated
recall target; both revisions fall below either. Full end-to-end proposal
evaluation (`e2e_eval`) has not been run as part of this comparison.

## Remaining validation limits

- Build the changed CPU/CUDA vision images. Docker daemon access was unavailable
  locally; a successful CPU dependency install does not prove container builds.
- The captured Orin `fetch_models.py` import-path fix is now incorporated in the
  source branch; unrelated manipulation changes were not applied. All requested
  JSON reports, logs and manifests are now retained,
  with archive and internal checksums verified (see `validation/README.md`).

See [README.md](README.md#compatibility-checks) for commands and measurement
limitations. The gallery crop accuracy comparison passed with user-reported
evidence and independently verified JSON equality. Additional CPU/CUDA container
build checks remain unverified; the Orin image build was user-verified.

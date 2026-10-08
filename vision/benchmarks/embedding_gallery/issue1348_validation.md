# Issue #1348 validation

Branch base: `816fca5f` (fetched `origin/main`). Results below describe the
uncommitted implementation on `codex/1348-text-embeddings-from-main`.

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

## Still required before closing the issue

- Build the changed CPU/CUDA and Orin vision images. Docker daemon access was
  unavailable locally; source inspection and a successful CPU dependency install
  do not prove that all container builds succeed.
- Run the integration checks on the Orin's Python 3.12/PyTorch 2.11 stack,
  including GPU dtype/device behavior and the production TensorRT backend.
- Run `profile_clip.py` on the Orin with a populated production gallery and CLIP
  resident together. Retain JSON reports for representative batch sizes and
  record the actual power/clock configuration.
- Run the existing gallery dataset benchmarks on baseline and candidate with
  identical data and settings. The RCW2026_v2 dataset was unavailable locally,
  so unchanged benchmark accuracy has not yet been demonstrated.

See [README.md](README.md#compatibility-checks) for commands and measurement
limitations. Hardware and dataset checks remain pending, not passed.

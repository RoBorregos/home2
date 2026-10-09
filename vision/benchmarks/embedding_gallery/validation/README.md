# Orin evidence for issue #1348

`issue1348-orin-evidence.tar.gz` was transferred from the Orin. Its SHA-256 is:

```text
feb52f4122307e49d233182c24b99a42a72c8c3274a2f1b83275cde42156b06f
```

The original archive is retained. Its 19 regular files were extracted to
`orin_evidence/` after rejecting unsafe paths, links and duplicate member names.
All 18 payload checksums match `orin_evidence/SHA256SUMS.json` (the checksum
manifest does not hash itself). Extracted evidence has not been edited.

Verified locally from the transferred files:

- Baseline and candidate accuracy JSON payloads are identical after removing
  only the top-level timestamp.
- The exported baseline embedder matches Git revision `816fca5f` exactly.
- Candidate `image_embedder.py` and `profile_clip.py` match this workspace exactly.
- The compatibility log records 16 passed, 2 warnings in 8.96 seconds.
- Performance reports cover batches 1, 8 and 32, with 512-dimensional CLIP,
  five gallery objects and MAXN mode 0 with dynamic clocks.
- Manifests contain 1,290 prepared evaluation files and 1,096 source image/label
  records. Images were not transferred, so their content hashes cannot be
  independently recomputed here; the user previously verified them on the Orin.

The captured Git HEAD is `e0af9eadc11b1552dbcbd46363ecd41ed5d873d2`.
`working_tree.diff` records the Orin warmup import-path fix plus unrelated
manipulation submodule changes. It is evidence, not a patch to apply wholesale.
The source snapshots and metadata capture export-time state; the compatibility
and accuracy logs do not independently record a per-run Git SHA.

See [the validation report](../issue1348_validation.md) for measured results,
the alternative dataset split, and remaining limitations.

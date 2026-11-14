# Live shelf geometry

The detector reads registered depth, `CameraInfo`, live arm joints and the exact
camera-to-`base_link` transform at the depth timestamp. Its coordinate contract
assumes `base_link` is gravity-aligned at the robot floor. It does not move the
robot or correct the hand-eye transform.

## Running

After synchronizing the Python changes, build through the existing area entry
point:

```bash
./run.sh manipulation --ppc --build
```

In another manipulation container shell with the ROS workspace sourced:

```bash
ros2 run xarm_utils shelf_live_calibrator.py \
  --arena 1 \
  --timeout 30 \
  --geometry-json /tmp/shelf_geometry.json
```

The normal calibration file still contains the arena's ordered heights and the
existing compartment height. The optional geometry JSON is separate. They must
not be the same path. Proposed height calibration is validated before atomically
replacing the previous file; other arenas are preserved.

`--dry-run` continues to suppress the height calibration write. An explicitly
requested `--geometry-json` is still written for inspection.

## Detection and structure

Small connected normal patches can seed a plane. Nearby coplanar patches are
merged before applying the final measured size and point-count requirements.
Growth uses measured point-to-plane distances, vetoes observed vertical faces
and undersides, and stops at 3-D discontinuities. Missing depth is never filled.

Matching lateral/depth footprints select a coherent stack, reducing dependence
on the single widest plane in the image. Vertical side/back panels are fitted
near the measured stack. A narrow vertical front face can also be fitted at
each level. There is no manually selected shelf ROI or fixed arena height list.
The existing broad image crop remains.

Missing boundaries are reported as `null`. Where both sides and the back were
measured, their intersections with the support plane estimate a bounded outline.
An unobserved front limit remains explicitly estimated. Otherwise the outline
falls back to the convex hull of measured support. A hull can span holes; its
interior is not thereby measured or clear.

Large flat objects with cabinet-like geometry can remain ambiguous. This is a
geometric detector, not an object or furniture classifier.

## Geometry API and JSON

`detect_shelf_levels(...)` retains the list-of-heights API.
`detect_shelf_geometry(...)` returns a `ShelfObservation` containing:

- The plane normal and offset, representative height, residual error and support
  point count for each visible level.
- The observed support hull and the separately labelled estimated outline.
- Measured side/back panels and per-level front faces when available.
- A heuristic quality score and lateral/depth coverage. The score is not a
  calibrated probability.
- A surface grid with its origin, in-plane axes and actual cell resolution.
- Capture timestamp and geometry/calibration diagnostics.

Grid states:

| Value | Meaning |
|---|---|
| 0 | Unknown: no measured support or obstacle in this cell |
| 1 | Observed support points |
| 2 | Measured points above the fitted plane |
| 3 | Outside the estimated outline |

Obstacle observations take precedence over support. The inspected height band
is explicitly reported; it is at most 0.30 m above the plane and is reduced when
another observed shelf lies below that limit. No grid state certifies a clear
grasp trajectory or placement volume. Ceiling thickness and collision objects
are not inferred from the normal calibration's fixed compartment height.

The confirmed JSON records a median height plus the source timestamp and plane
geometry from the best observed frame for each level. Geometry from different
frames is not silently fused. Visible level indices are not guaranteed global
competition level labels if some shelves are outside the view.

## Adaptive confirmation

The default window is 12 distinct depth frames, with repeated heights allowed
to be absent from some views. With high-quality observations the detector can
confirm from 6 frames (`--min-valid-frames`), bounded by `--valid-frames`.

Early confirmation requires low plane residual, broad measured coverage in both
dimensions, three observed cabinet panels, consistent normals and no calibration
warning. An unresolved repeatedly observed extra level defers early completion.
Partial or questionable geometry retains the full window. Height-only users of
`ShelfLevelConsensus.add()` retain the full-window behavior.

Lost/stale arm feedback resets both height and quality history. Negative support
heights below the floor-aligned frame are rejected for automatic calibration.
Depth stamps are deduplicated, and TF reception continues while geometry is
processed.

## Physical accuracy checks

A common lean across support normals is reported. Nonorthogonality between a
support and an observed panel is reported separately: a rigid camera TF rotation
cannot change the angle between measured planes. Such a discrepancy can involve
depth fitting, panel association or actual shelf geometry, not just hand-eye
calibration. These diagnostics do not identify the physical cause or adjust TF.

`--reference-heights` accepts externally measured heights in `base_link`, ordered
low to high. It checks them against the result using `--max-reference-error`
(default 0.02 m). A failed check prevents the height calibration write; the
requested geometry report retains the measured errors. Do not use the
detector's own estimates as a physical reference.

Proper camera-to-arm calibration still requires physical reference observations
and robot poses. It cannot be derived from the saved shelf photo alone.

## Local validation

The geometry, temporal and persistence tests do not need ROS or a robot:

```bash
PYTHONPATH=manipulation/packages/xarm_utils \
  python3 -m pytest -q manipulation/packages/xarm_utils/test
```

New behavior must still be checked on the Orin with live RGB-D and measured
reference geometry. Timing from earlier detector versions does not establish the
runtime of this version.

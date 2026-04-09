# Deep SORT — Multi-Object Tracking

Core tracking files extracted from
[nwojke/deep_sort](https://github.com/nwojke/deep_sort)
(`deep_sort/` directory of that repository).

## Origin

The original repository implements the paper:

> N. Wojke, A. Bewley, D. Paschke,
> *"Simple Online and Realtime Tracking with a Deep Association Metric"*,
> ICIP 2017.

Only the **library modules** needed at runtime were copied. Application-level
scripts (`deep_sort_app.py`, `generate_detections.py`, etc.) and the CNN
feature-extraction model were **not** included — feature vectors are supplied
externally by our pipeline.

## Files

All files below come from the original `deep_sort/` package unless noted
otherwise.

| File | Original path | Description |
|---|---|---|
| `tracker.py` | `deep_sort/tracker.py` | **Main entry point.** `Tracker` class that orchestrates prediction, matching, and track life-cycle management (creation / confirmation / deletion). |
| `track.py` | `deep_sort/track.py` | `Track` and `TrackState` — represents a single tracked object with its Kalman state, hit/miss counters, and feature cache. |
| `detection.py` | `deep_sort/detection.py` | `Detection` — lightweight wrapper around a bounding box, confidence score, and appearance feature vector. |
| `kalman_filter.py` | `deep_sort/kalman_filter.py` | `KalmanFilter` — constant-velocity Kalman filter in `(x, y, a, h)` space used to predict and update track states. Also provides Mahalanobis gating distances (`chi2inv95`). |
| `linear_assignment.py` | `deep_sort/linear_assignment.py` | Matching logic: `matching_cascade` (priority matching by track age), `min_cost_matching` (Hungarian / linear-sum assignment), and `gate_cost_matrix` (Kalman gating). |
| `nn_matching.py` | `deep_sort/nn_matching.py` | `NearestNeighborDistanceMetric` — appearance-based distance metric (cosine or Euclidean) with a per-target feature budget. |
| `iou_matching.py` | `deep_sort/iou_matching.py` | IoU-based cost matrix used as a fallback for unconfirmed tracks and recently unseen tracks. |
| `preprocessing.py` | `deep_sort/preprocessing.py` | `non_max_suppression` — NMS utility for suppressing overlapping detections before they enter the tracker. |
| `visualization.py` | `deep_sort/visualization.py` | `Visualization` / `NoVisualization` — OpenCV viewer helpers for debugging. **Not used in production**; kept for reference. |

## Files NOT included from the original repo

- `deep_sort_app.py` — standalone demo application
- `generate_detections.py` — offline feature generation script
- `tools/` — evaluation and dataset utilities
- CNN model weights (`resources/networks/`) — we use our own feature extractor

## How tracking works (high-level)

```
detections ──► Tracker.predict()  (Kalman predict for every track)
           ──► Tracker.update()
                 ├─ matching cascade   (appearance features, confirmed tracks)
                 ├─ IOU matching       (unconfirmed + recently lost tracks)
                 ├─ update matched tracks  (Kalman update)
                 ├─ mark missed tracks
                 └─ initiate new tracks from unmatched detections
```

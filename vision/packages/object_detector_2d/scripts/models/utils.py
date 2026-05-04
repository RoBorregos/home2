def _iou(det1, det2) -> float:
    xA = max(det1.bbox_.x1, det2.bbox_.x1)
    yA = max(det1.bbox_.y1, det2.bbox_.y1)
    xB = min(det1.bbox_.x2, det2.bbox_.x2)
    yB = min(det1.bbox_.y2, det2.bbox_.y2)
    inter = max(0.0, xB - xA) * max(0.0, yB - yA)
    a1 = (det1.bbox_.x2 - det1.bbox_.x1) * (det1.bbox_.y2 - det1.bbox_.y1)
    a2 = (det2.bbox_.x2 - det2.bbox_.x1) * (det2.bbox_.y2 - det2.bbox_.y1)
    return inter / (a1 + a2 - inter + 1e-6)


def iou_deduplicate(detections: list, threshold: float = 0.6) -> list:
    """Remove detections that overlap an already-kept detection above `threshold`."""
    kept = []
    for det in detections:
        if not any(_iou(det, k) > threshold for k in kept):
            kept.append(det)
    return kept

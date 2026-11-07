"""IoU-based deduplication for merging detections from multiple models."""


def iou(det1, det2) -> float:
    xA = max(det1.bbox_.x1, det2.bbox_.x1)
    yA = max(det1.bbox_.y1, det2.bbox_.y1)
    xB = min(det1.bbox_.x2, det2.bbox_.x2)
    yB = min(det1.bbox_.y2, det2.bbox_.y2)
    inter = max(0.0, xB - xA) * max(0.0, yB - yA)
    a1 = (det1.bbox_.x2 - det1.bbox_.x1) * (det1.bbox_.y2 - det1.bbox_.y1)
    a2 = (det2.bbox_.x2 - det2.bbox_.x1) * (det2.bbox_.y2 - det2.bbox_.y1)
    return inter / (a1 + a2 - inter + 1e-6)


def iou_msg(det1, det2) -> float:
    """IoU between two ObjectDetection messages (normalized xmin/ymin/xmax/ymax)."""
    xA = max(det1.xmin, det2.xmin)
    yA = max(det1.ymin, det2.ymin)
    xB = min(det1.xmax, det2.xmax)
    yB = min(det1.ymax, det2.ymax)
    inter = max(0.0, xB - xA) * max(0.0, yB - yA)
    a1 = (det1.xmax - det1.xmin) * (det1.ymax - det1.ymin)
    a2 = (det2.xmax - det2.xmin) * (det2.ymax - det2.ymin)
    return inter / (a1 + a2 - inter + 1e-6)


def iou_deduplicate(detections: list, threshold: float = 0.6) -> list:
    """Remove detections that overlap an already-kept detection above `threshold`."""
    kept = []
    for det in detections:
        if not any(iou(det, k) > threshold for k in kept):
            kept.append(det)
    return kept

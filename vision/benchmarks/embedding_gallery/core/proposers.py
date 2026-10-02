"""Class-agnostic box proposers: the production one (YOLOE prompt-free) and
the Phase 0 candidates listed in models.json."""

import numpy as np

from core.dataset import DETECTORS_DIR

BOX_PROPOSER_WEIGHT = DETECTORS_DIR / "yoloe-11l-seg-pf.pt"
BOX_CONF = 0.10

# DINOv2's own timm normalization mean, in 0-255 RGB. Fills the background
# with a color that contributes ~zero signal, instead of an
# out-of-distribution black square.
NEUTRAL_FILL_RGB = (124, 116, 104)


def make_box_proposer():
    """Production proposer. The returned fn gives [(bbox_px, polygon|None), ...]."""
    from ultralytics import YOLOE

    model = YOLOE(str(BOX_PROPOSER_WEIGHT))

    def propose(image):
        results = model.predict(image, conf=BOX_CONF, verbose=False)
        out_list = []
        for out in results:
            if out.boxes is None:
                continue
            masks = out.masks
            for i, box in enumerate(out.boxes):
                bbox = [round(v) for v in box.xyxy[0].tolist()]
                poly = None
                if masks is not None and i < len(masks):
                    xy = masks[i].xy
                    if xy and len(xy[0]) >= 3:
                        poly = np.asarray(xy[0], dtype=np.int32)
                out_list.append((bbox, poly))
        return out_list

    return propose


def masked_crop(image, bbox, poly):
    """Crop with the background outside `poly` filled with NEUTRAL_FILL_RGB.

    `image` is BGR, so the fill is reversed to match.
    """
    import cv2

    x1, y1, x2, y2 = bbox
    crop = image[y1:y2, x1:x2].copy()
    if poly is None or len(poly) < 3:
        return crop
    full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(full_mask, [poly], 255)
    crop_mask = full_mask[y1:y2, x1:x2]
    crop[crop_mask == 0] = NEUTRAL_FILL_RGB[::-1]
    return crop


def _xyxy_boxes(results) -> list[list[int]]:
    boxes = []
    for out in results:
        if out.boxes is None:
            continue
        for box in out.boxes:
            boxes.append([round(v) for v in box.xyxy[0].tolist()])
    return boxes


def make_yolo_agnostic_proposer(filename: str, conf: float):
    """Phase 0 candidate: a generic YOLO with agnostic NMS."""
    from detectors.registry import MODELS_PATH
    from ultralytics import YOLO

    model = YOLO(MODELS_PATH + filename)

    def propose(image):
        return _xyxy_boxes(
            model.predict(image, conf=conf, agnostic_nms=True, verbose=False)
        )

    return propose


def make_yoloe_proposer(filename: str, conf: float, prompt_classes: list[str] | None):
    """Phase 0 candidate: YOLOE, text-prompted or prompt-free."""
    from detectors.registry import MODELS_PATH
    from ultralytics import YOLOE

    model = YOLOE(MODELS_PATH + filename)
    if prompt_classes:
        model.set_classes(prompt_classes, model.get_text_pe(prompt_classes))

    def propose(image):
        return _xyxy_boxes(model.predict(image, conf=conf, verbose=False))

    return propose

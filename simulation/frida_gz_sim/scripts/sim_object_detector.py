#!/usr/bin/env python3
"""Runs the real object_detector_node with TensorRT export disabled (no TensorRT in the sim image)."""

import os
import sys

from ament_index_python.packages import get_package_prefix


def main():
    # The detector's modules are siblings of its node script, not an installed package
    sys.path.insert(
        0,
        os.path.join(
            get_package_prefix("object_detector_2d"), "lib", "object_detector_2d"
        ),
    )

    from detectors.registry import MODEL_CONFIGS

    for config in MODEL_CONFIGS.values():
        config["use_trt"] = False
    # The nano COCO model misses rendered objects; the small one detects them reliably
    MODEL_CONFIGS["yolo_generic"]["filename"] = os.environ.get(
        "SIM_YOLO_MODEL", "yolo26s.pt"
    )

    import object_detector_node

    object_detector_node.main()


if __name__ == "__main__":
    main()

# Vision

The Vision area handles all robot perception: 2D/3D object detection, person tracking with re-identification, face recognition, pose estimation, and vision-language queries via the Moondream 2 model. It sits between the camera hardware and every other area that needs to interpret the world visually.

## Architecture

```
ZED Camera (RGB + Depth)
        │
  [image_orienter]
        │ /vision/camera/image_oriented
        ├─────────────────────────────────────┐
        │                                     │
[object_detector_node]     [zero_shot_object_detector_node]
 yolo_finetuned + cutlery   yoloe-11l-seg (runtime classes)
        │                                     │
 /vision/detections           /vision/zero_shot_detections
        │
        └──────────────────────────────────────────────┐
                                                       │
         /vision/camera/image (raw)                    │
                  │                                    │
          [moondream_node]         [tracker_node]      │
          [trash_detection]        [face_recognition]
          [customer_node]          [pointing_detection]
          [dishwasher_node]
```

`image_orienter` must be running before any detector. It rotates the raw image before publishing to `/vision/camera/image_oriented`. Application nodes (tracker, face recognition, etc.) subscribe to the raw topic; detector nodes subscribe to the oriented one.

## Packages

### object_detector_2d

Provides continuous 2D object detection with optional 3D projection via ZED depth.

- **object_detector_node**: Runs multiple YOLO models simultaneously (finetuned objects + cutlery). Publishes 2D bounding boxes and, when depth is active, 3D poses in the ZED frame. Detections are deduplicated by IoU and filtered by max range (2.0 m default). Other areas query detections by label via a service rather than subscribing to the raw stream.

- **zero_shot_object_detector_node**: Uses YOLO-E to detect arbitrary object classes that can be updated at runtime via a service call — useful when the target object is only known at task time. Its output can be remapped to `/vision/detections` for a unified interface.

Model registry (`scripts/detectors/registry.py`):

| Name | File | Confidence |
|------|------|------------|
| yolo_finetuned | abril9.pt | 0.6 |
| cutlery | cutlery.pt | 0.3 |
| zero_shot | yoloe-11l-seg.pt | 0.25 |

Configuration is set in `config/parameters.yaml` (models to load, depth projection on/off, max range, image flip).

### moondream_run

Provides a ROS 2 interface to the Moondream 2 vision-language model, which runs as a gRPC server on `localhost:50052`. Used for open-ended image queries, beverage location, person posture classification, and pixel-coordinate localization of described objects. Internally runs YOLOv8n to crop the person region before person-specific queries.

### vision_general

Application-level nodes and shared utilities. Each node is specialized for a perception task:

- **image_orienter**: Rotates the raw camera image by 0/90/180/270°. Required by all detector nodes.

- **tracker_node**: Tracks a single person using DeepSORT + ReID embeddings. The target can be selected by largest area, body pose, hand gesture, or clothing color. Publishes the tracked person's 3D position and a normalized 2D centroid. ReID embeddings allow re-acquiring the person after occlusion (up to 128 embeddings stored).

- **face_recognition_node**: Detects and recognizes faces using CNN-based encoding. Saves known faces to disk. Publishes normalized coordinates for arm following and the detected person's name. Names are assigned via service call.

- **trash_detection_node**: Detects trash and trashcans by querying Moondream and back-projects the result to 3D using depth. Publishes into the standard detections topic.

- **customer_node**: Detects a sitting customer who is raising their hand (HRIC task) using YOLO11m pose keypoints. Returns a 3D position via service.

- **dishwasher_node**: Three separate TensorRT YOLO models detect dishwasher layout, rack compartments, and detergent tablet. Each has its own service endpoint.

- **pointing_detection**: Combines YOLO11m-pose (wrist + shoulder keypoints) with active zero-shot detections to determine which object a person is pointing at. Object classes can be updated at runtime.

Shared utilities (`vision_general/utils/`): `deep_sort/` (DeepSORT tracker), `reid_model.py` (embedding extraction), `trt_utils.py` (TensorRT auto-export for Orin), `calculations.py` (2D→3D deprojection helpers).

## Launch Files

| File | When to Use |
|------|-------------|
| `object_detector_node.launch.py` | Standard YOLO detection only |
| `zero_shot_object_detector_node.launch.py` | Zero-shot detection only, output remapped to `/vision/detections` |
| `object_detector_combined.launch.py` | Both detectors + image_orienter in parallel |
| `gpsr_launch.py` | Full GPSR stack: combined detectors + face + trash + moondream + tracker |
| `ppc_launch.py` | Pick & Place: object detection + trash + moondream |
| `hric_launch.py` / `restaurant_launch.py` | Task-specific stacks |

## Hardware Notes

- **ZED camera** provides RGB + depth. Run it in its own container with `./run.sh zed`. For development without hardware, use `zed_simulator.py` to stream from a USB webcam.
- **GPU (CUDA)** is required for real-time YOLO. On Orin AGX, models are auto-exported to TensorRT `.engine` format on first run via `trt_utils.py`.
- All 3D operations use TF frame `zed_left_camera_optical_frame`.
- The **Moondream gRPC server** must be started before `moondream_node` (use `--download-model` on first run to pull the weights).

## Running

Start the ZED camera:
```bash
./run.sh zed
```

Start the vision area container:
```bash
./run.sh vision
```

With flags:
```bash
./run.sh vision --build          # build ROS 2 packages inside the container
./run.sh vision --build-image    # rebuild the Docker image
./run.sh vision --open-display   # open graphical interface
```

To find a camera's video device id for the simulator:
```bash
python3 vision/packages/vision_general/Utils/camera_test.py <video_id>
```

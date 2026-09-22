# Resource Consumption Map

Approximate CPU, GPU, VRAM, RAM, energy footprint per software area, based on a static read of the models, frameworks, and launch configuration in this repo (no profiling was run). Target platform: **NVIDIA Jetson Orin AGX** (12-core ARM CPU, ~64GB shared LPDDR5, Ampere GPU, 15–60W configurable power envelope). Numbers are engineering estimates.

---

## Summary Table

| Area             | CPU                                                       | GPU (compute)                                                                                                 | VRAM                                                                  | RAM                                  | Power (approx.)                                |
| ---------------- | --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- | ------------------------------------ | ---------------------------------------------- |
| **Vision**       | Low–Med (4-6 cores for I/O + pre/post-proc)               | **High**, near-continuous                                                                                     | ~2 GB (detectors) + 4-5 GB (Moondream2 VLM, separate container)       | ~2-3 GB                              | 30-50 W                                        |
| **Manipulation** | **High** (MoveIt/OMPL/VAMP planning is CPU-bound)         | Low (GPD inference is brief, CPU by default)                                                                  | ~50-100 MB (GPD) + upstream YOLOE/CLIP if counted                     | ~1-2 GB                              | 15-25 W                                        |
| **Navigation**   | **High**, sustained (Nav2 MPPI + costmaps + SLAM @ 20 Hz) | None by default; optional 0-25% (RTABMap SuperPoint loop closure, off by default)                             | 0 (0 with RTABMap off)                                                | 400 MB-1.2 GB                        | 15-25 W                                        |
| **HRI**          | Med (audio I/O, embeddings)                               | **High** when speech/LLM active (STT+TTS+noise-cancel+embeddings, all local; LLM optionally local via Ollama) | 7-15 GB if Ollama runs locally; 2-4 GB if LLM is an external API call | 4-8 GB                               | 40-80 W local LLM; 15-25 W with API-backed LLM |
| **Integration**  | Very low (1-2 cores: FSM + DDS/iceoryx)                   | None                                                                                                          | 0                                                                     | ~1.5-2 GB (mostly iceoryx SHM pools) | 5-10 W                                         |

**Read this table relatively, not absolutely**: these areas don't all run at full load simultaneously (e.g. GPD only spikes during a pick attempt; RTABMap and local-Ollama are both opt-in). Concurrent worst case (everything active at once) plausibly exceeds the Orin AGX's realistic sustained budget — see [Contention Risks](#contention-risks-and-caveats).

---

## Vision

**Packages:** `object_detector_2d`, `vision_general`, `moondream_run`

| Component                  | Model                                  | Footprint                                                 | Notes                                                                      |
| -------------------------- | -------------------------------------- | --------------------------------------------------------- | -------------------------------------------------------------------------- |
| Fine-tuned object detector | `robocup2026_v1.pt` (custom YOLO)      | ~50-200 MB VRAM                                           | TensorRT-compiled                                                          |
| Generic detector           | `yolo26n.pt` (YOLO nano)               | ~150-200 MB VRAM                                          | Lightweight                                                                |
| Zero-shot detector         | `yoloe-11l-seg.pt` (YOLOE-large, seg)  | ~600-800 MB VRAM                                          | Heaviest detector                                                          |
| Tracker                    | `yolov8n` + `yolo11m-pose` + SWIN ReID | ~400-500 MB VRAM (pose) + ~300-400 MB (ReID, lazy-loaded) | 15-30 ms/frame detection, 30-100 ms ReID extraction                        |
| Face recognition           | InsightFace `buffalo_sc`               | ~200-300 MB VRAM                                          | onnxruntime-gpu + TensorRT provider; ~10 min one-time TRT warmup on Orin   |
| VLM (separate container)   | Moondream2 (2B params)                 | ~4-5 GB VRAM (fp32) / ~2-2.5 GB (int8)                    | gRPC server, called by tracker/moondream nodes for color/pose/crop queries |

**Total concurrent detector stack: ~1.5-2 GB VRAM**, plus **4-5 GB more** if the Moondream2 container is up. All detectors run through TensorRT/onnxruntime-gpu — this is the area most consistently pinned to the GPU. CPU load is comparatively light (frame I/O, NMS, pre/post-processing).

---

## Manipulation

**Packages:** `pick_and_place`, `arm_pkg`, `perception_3d`, `gpd`, `vamp_moveit_plugin`, `frida_pymoveit2`, `xarm6_ikfast_plugin`

| Component                  | Type                                                                   | Footprint                                                            | Notes                                                                                           |
| -------------------------- | ---------------------------------------------------------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| GPD (Grasp Pose Detection) | Small CNN (2 conv + 2 FC, LeNet-style)                                 | 14.5 MB (fp32) / 6.9 MB (fp16) weights, ~50 MB VRAM if GPU path used | **Defaults to CPU** via OpenVINO (`device = 0`); ~15 ms/batch of 100 grasp images, ~2 cores     |
| Flat grasp estimator       | Geometry only (PCA on point cloud)                                     | negligible                                                           | Pure CPU, 10-15 ms/frame                                                                        |
| MoveIt2 motion planning    | OMPL (RRTConnect), VAMP, CHOMP, Pilz                                   | ~500 MB RAM                                                          | **CPU-bound**, this is the heaviest sustained cost in the area — up to ~6 cores during planning |
| IK                         | IKFast (precomputed)                                                   | negligible                                                           | Fast CPU lookup                                                                                 |
| Upstream perception        | YOLOE + CLIP (used by grasp estimator, technically vision-area models) | ~2-4 GB VRAM                                                         | Counted once under Vision to avoid double-counting                                              |

Manipulation is **CPU-dominant**, not GPU-dominant — the bottleneck is motion planning (MoveIt2/OMPL sampling), not grasp inference. GPD's neural net is small and brief. GPU is essentially idle here unless GPD or the upstream detector is explicitly configured to use it.

---

## Navigation

**Packages:** `nav_main` (Nav2 stack), `sllidar_ros2`, `dashgo_driver`, `omnidriver`, `map_context`, `ira_laser_tools`

| Component                                                                    | Rate                | Resource                                  | Notes                                       |
| ---------------------------------------------------------------------------- | ------------------- | ----------------------------------------- | ------------------------------------------- |
| Nav2 controller (MPPI)                                                       | 20 Hz               | ~25% CPU                                  | batch_size=1200, sampling-based             |
| Nav2 planner (A\*)                                                           | 20 Hz               | ~10% CPU                                  | navfn                                       |
| Local/global costmaps (obstacle + STVL 3D voxel layer)                       | 5 Hz / 2 Hz         | ~10% CPU combined                         | LIDAR + point cloud fusion                  |
| EKF localization                                                             | 20 Hz               | ~5% CPU                                   | robot_localization                          |
| SLAM (slam_toolbox)                                                          | ~30 Hz (LIDAR rate) | ~8% CPU                                   | Async online SLAM, no GPU                   |
| RTABMap + SuperPoint (**optional, off by default**, legacy dashgo base only) | 2 Hz loop closure   | ~15-25% GPU, +30-40% CPU, +500-600 MB RAM | TorchScript SuperPoint on CUDA when enabled |

Default (omnibase) navigation stack is **pure classical CPU control** — no learned planners, no semantic segmentation. Sustained load is estimated at **40-70% of the Orin's CPU** (3-4 cores) with **0% GPU**. The only GPU path is the optional RTABMap visual loop-closure on the legacy dashgo base, disabled by default.

---

## HRI

**Microservices:** STT (Faster-Whisper), TTS (Kokoro), noise cancellation (DeepFilterNet3), embeddings, NLP (sentiment/coherence + LLM wrapper), display

| Component                | Model                                                     | Footprint                                                       | Local vs. API                                                           |
| ------------------------ | --------------------------------------------------------- | --------------------------------------------------------------- | ----------------------------------------------------------------------- |
| STT                      | `distil-large-v3` (Jetson)                                | ~1-2 GB VRAM                                                    | Always local, CUDA-enabled CTranslate2                                  |
| TTS                      | Kokoro                                                    | ~1-2 GB VRAM                                                    | Always local, CUDA if available                                         |
| Noise cancellation       | DeepFilterNet3 (~60M params)                              | ~256 MB VRAM                                                    | Always local, auto-falls back to CPU if it can't keep up with real-time |
| Embeddings               | `all-MiniLM-L12-v2` (33M params)                          | ~256 MB VRAM                                                    | Always local                                                            |
| Sentiment/coherence      | `tasksource/deberta-small-long-nli` (~180M params)        | ~360 MB+                                                        | Always local                                                            |
| LLM (dialogue/reasoning) | `qwen3` via Ollama, **or** external OpenAI-compatible API | **4-8 GB VRAM if local Ollama**, near-zero local compute if API | **Configurable** — biggest swing factor in this area                    |

**This is the area with the widest resource range**, entirely dependent on one config flag (`base_url` for the LLM wrapper). With everything local (STT+TTS+noise-cancel+embeddings+deberta+Ollama), VRAM demand is **7-15 GB** — a large fraction of the Orin AGX's shared memory pool, competing directly with Vision's Moondream2 and detector stack. Routing the LLM to an external API cuts that to **2-4 GB** and removes the single largest local GPU consumer in the whole system.

---

## Integration

**Components:** `task_manager` (FSM/behavior-tree orchestration), `docker/integration` compose profile, CycloneDDS + iceoryx (DDS/shared-memory middleware)

| Component                                  | Resource                                                                                | Notes                                                                                                                         |
| ------------------------------------------ | --------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| task_manager FSM node(s)                   | ~2-5% CPU, ~100 MB RAM                                                                  | Pure orchestration logic (py_trees behavior tree for GPSR), no ML inference. Calls into other areas via ROS services/actions. |
| CycloneDDS                                 | ~1-2% CPU, ~50 MB RAM                                                                   | Message-passing middleware                                                                                                    |
| iceoryx (roudi daemon)                     | ~1-2% CPU, ~700-800 MB RAM (shared memory pools, mostly for camera/depth frame buffers) | Shared across _all_ areas, not integration-specific compute — it's the IPC backbone                                           |
| BAML (LLM-based code-gen for task parsing) | ~10-30% CPU, ~500 MB-1 GB RAM                                                           | CPU-only text processing, not vision-scale                                                                                    |

Integration is **negligible compute** on its own — it's coordination glue plus the DDS/iceoryx transport layer that every other area's messages ride on. The `docker/integration` compose profile pins itself to CPU cores 4-11, explicitly reserved to avoid contention with navigation (pinned to cores 0-3).

---

## Contention Risks and Caveats

- **GPU/VRAM is the tightest shared resource.** Vision's detector stack (~2 GB) + Moondream2 (~4-5 GB) + HRI's local models (~2-8 GB depending on Ollama) can together approach or exceed what's comfortable to share with manipulation/navigation's occasional GPU bursts. If all of Vision + HRI's local-LLM path run concurrently, VRAM pressure is the most likely failure mode — worth watching if the Orin starts throttling or OOM-killing containers.
- **CPU is the tightest resource for navigation + manipulation running together**: Nav2's MPPI/costmaps (sustained, ~40-70% of CPU) plus MoveIt2 planning bursts (up to ~6 cores) can compete for the Orin's 12 cores, especially with integration's task_manager, BAML, and iceoryx also active.
- **The LLM backend choice in HRI (local Ollama vs. external API) is the single biggest lever** for total system resource pressure — it swings HRI's footprint by several GB of VRAM and tens of watts.
- **RTABMap (navigation) is off by default** — if enabled it adds a real but modest GPU load (~15-25%) on top of an otherwise GPU-free navigation stack.
- These are static estimates from reading code/config, not measurements. If tighter numbers are needed, the natural next step is running `tegrastats` (Jetson) or `nvidia-smi` + `htop` while each area is active standalone, then combined, to get ground truth instead of estimates.

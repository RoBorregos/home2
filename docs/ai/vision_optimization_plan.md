# Vision optimization plan — Orin-first architecture

Drafted 2026-07-03 from the full vision audit (see `hric_vision_architecture.md` +
`diagrams/vision_architecture.svg`). Goal: less VRAM/CPU on the Orin, deterministic
model provisioning, a subtask manager that can't drift, and debug/display output that
demonstrates vision working per task step without eating bandwidth.

## 0. Current state (findings that motivate the plan)

**Models** — 3 different storage conventions coexist:
1. `vision_general` nodes: `load_yolo_trt("<name>.pt")` — ultralytics downloads the `.pt`
   to the container CWD (`/workspace`, NOT a persistent mount) on first run, exports a
   TRT engine into `TENSORRT_CACHE_DIR` → host `docker/vision/trt_cache`. The same
   `trt_cache` folder is also overloaded as the **InsightFace** cache
   (`./trt_cache:/home/ros/.insightface`).
2. `object_detector_2d`: registry expects `.pt` files beside `scripts/detectors/`
   (`MODELS_PATH`), e.g. `robocup2026_v1.pt`, `yolo26n.pt`, `yoloe-11l-seg.pt`.
3. `moondream_run`: HF cache mount `./moondream_cache:/home/ros/.cache/huggingface`.

`.gitignore` excludes all `.pt`/`.engine` (except `tmr2025.pt`), so a fresh machine or a
recreated container **needs internet** to fetch weights, then minutes of TRT export.
`ultralytics==8.4.19` is pinned; `insightface` is not. `Dockerfile.l4t` still clones the
legacy **yolov5** repo (vestigial — `/workspace/yolov5`).

**Duplication** — `yolo11m-pose` loaded 5× across the area (hric_commands, tracker,
gpsr_commands, customer_node, pointing_detection); `yolov8n` 2× (tracker, moondream_node).
Each duplicate = its own CUDA context + engine in VRAM + startup export/load.

**Runtime issues** — single-threaded executors with blocking in-callback service waits
(hric_commands, person_in_map); `ros_utils.wait_for_future` is a 100 %-CPU busy loop;
face_recognition + object_detector run inference always-hot (activity flags exist but
half-wired — object detector's `DETECTIONS_ACTIVE_TOPIC` constant is literally `"asd"`);
restaurant launch starts **two** image_orienters; trash_detection_node launched for
gpsr/ppc/dlc with **zero** clients; moondream_node consumes the **raw** camera topic
(no rotation compensation).

**Subtask manager** — 25 near-identical `call_async + spin_until_future_complete` blocks;
per-task service dicts drift from reality (HRIC uses `track_person` but doesn't check it);
clients exist for services that have no server (`read_qr`, `detect_shelf`).

---

## 1. Centralized model store + deterministic provisioning

Target layout (host `docker/vision/models/`, mounted at `/workspace/models`):

```
docker/vision/models/
├── weights/            # all .pt, one place, synced between machines
│   ├── yolo11m-pose.pt, yolov8n.pt, yolo26n.pt, yoloe-11l-seg.pt
│   ├── robocup2026_v1.pt, tmr2025.pt, dishwasher_{layout,rack,tablet}.pt
│   └── MANIFEST.json   # name → sha256 + source URL + ultralytics version
├── engines/<device>/   # TRT engines are DEVICE- and TRT-version-specific:
│   └── orin-agx/…      # never sync engines between laptop and Orin
├── insightface/        # own subdir — stop overloading trt_cache
└── (moondream_cache stays as-is)
```

Steps:
1. `vision/scripts/fetch_models.py` — downloads every weight in MANIFEST with checksum
   verification. Run once with internet; after that the stack is **offline-safe**.
2. `./run.sh vision --warmup` — fetch + `load_yolo_trt` every model once so all TRT
   exports happen before competition day, not during a task's first service call.
3. `trt_utils.load_yolo_trt`: resolve `.pt` from `MODELS_DIR/weights`, engines from
   `MODELS_DIR/engines/$DEVICE`; keep the current fallback chain.
4. `object_detector_2d/registry.py`: `MODELS_PATH` → same `MODELS_DIR/weights`.
5. Pin `insightface` in `requirements/face_recognition.txt`; drop the yolov5 clone from
   `Dockerfile.l4t` (verify nothing imports it first).
6. Sync between machines = rsync `weights/` only; engines rebuild per device via warmup.

## 2. Runtime consolidation (VRAM + CPU on the Orin)

1. **One pose model instance.** Create a `pose_node` (or promote tracker's PoseDetection)
   that owns the single `yolo11m-pose` engine and serves keypoints via one service
   (`GetPose`: image region → keypoints) + a low-rate topic. Migrate detect_hand
   (hric_commands), gpsr_commands, customer_node, pointing_detection to it.
   5 engines → 1: saves roughly 1–2 GB VRAM (engine + CUDA workspace per process) and
   4 × the export/startup cost.
2. **Drop moondream_node's yolov8n** — its person-crop can come from the YoloDetect
   service (class 0) it already lives next to, or from tracker detections.
3. **Executor hygiene** — MultiThreadedExecutor (or async-everything) for `hric_commands`
   and `person_in_map`; async MapAreas cache (timer + `call_async`, the 2026-07-02 fix);
   replace `wait_for_future`'s `while not future.done(): pass` with
   `future.add_done_callback` or a sleep-loop.
4. **Activity gating that actually works** — fix the `"asd"` constant, default detector
   active-flags off-when-idle, and have the TM's existing `pause_vision()` called in
   `navigate_to()` so InsightFace/YOLO don't burn CPU/GPU while driving (this was the
   nav-starvation root cause in June).
5. **Launch cleanups** (from the active-task audit): remove the duplicate image_orienter
   from restaurant_launch, trash_detection_node from gpsr/ppc launches,
   face_recognition_node from restaurant_launch, moondream_node from the --dlc profile;
   fix or delete the dangling `--carry` mapping; quarantine `is_person_inside.py`.
6. **Orin platform** — FP16 TRT everywhere already (`half=True`); standardize
   `imgsz=640`; keep the cpuset partition (nav 0-3 / rest 4-11); add a `jetson_clocks` /
   MAXN check to the robot bring-up; optional experiment: pin the always-on detector to
   DLA to free GPU for moondream.

## 3. Subtask manager v2 (`vision_tasks.py`)

1. **One `_call` helper** replaces 25 copies of the boilerplate:
   ```python
   def _call(self, client, request, timeout=TIMEOUT, name=""):
       future = client.call_async(request)
       rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
       result = future.result()
       if result is None:
           Logger.warn(self.node, f"{name} timed out after {timeout}s")
           return Status.TIMEOUT, None
       return None, result       # caller maps result → (Status, payload)
   ```
   Uniform timeout handling (no more `'NoneType' has no attribute success'`), uniform
   logging, ~200 fewer lines.
2. **Registry-driven service map** — a single table
   `{function: (client_attr, interface, topic_const, tasks: [...])}` generates both the
   per-task startup checks and the `@service_check` wiring. Kills the drift class of bug
   (HRIC using `track_person` without checking it; `beverage_location` checked but unused).
3. **Contract enforcement** — every function returns `(Status, payload)` with `Status`
   enum (never bare ints/bools); mock values must match the real shape. The
   `test_hric_vision.py` harness generalizes to `test_vision_contract.py` that runs all
   functions in `mock_data=True` and fails CI on shape mismatches.
4. Delete dead surface: `read_qr`, `detect_shelf` (no servers), `person_bounding_box` /
   `pan_to_person` pseudocode, `detect_guest` stub — or implement their servers.

## 4. Debug topics — compressed, lazy, namespaced

**Today (audited):** every vision debug publisher is raw `sensor_msgs/Image` (bgr8) —
zero `CompressedImage`, zero subscriber-gating anywhere in the repo. Rates: hric_commands,
tracker, gpsr_commands, customer_node publish on **10 Hz timers**; face_recognition 5 Hz;
**base_detector (object + zero-shot) publishes EVERY camera frame (30 Hz)** inside
`rgb_cb`; restaurant/dishwasher/yolo-service publish on-request (fine). The only
compressor in the stack is `web_video_server` in the display container, which JPEG-encodes
a raw topic on demand when the browser opens an MJPEG stream. `image_orienter` additionally
republishes every ZED frame (its RELIABLE re-publish is load-bearing: web_video_server
needs RELIABLE; the raw ZED topic is BEST_EFFORT — that QoS bridge is *why* the display
default feed is `/vision/camera/image_oriented`).

Cost: a 1280×720 bgr8 frame ≈ 2.7 MB. Each ungated debug publisher pays cv_bridge
conversion + copy + DDS write at 10–30 Hz *whether or not anything is watching* —
~27 MB/s per 10 Hz topic, ~83 MB/s for base_detector, all day.

Plan:
1. **`DebugImagePublisher` helper** (`vision_general/utils/debug_pub.py`): publishes
   `sensor_msgs/CompressedImage` (`cv2.imencode('.jpg', q≈70)` ≈ 100–200 KB/frame),
   **skips work when `get_subscription_count() == 0`**, rate-limits to ~5 Hz, RELIABLE
   depth 1. Idle cost becomes zero; active cost drops ~20×.
2. **Topic convention** `/vision/debug/<node>/compressed` in `vision_constants.py`
   (fix the junk constants while at it: `DETECTIONS_ACTIVE_TOPIC`/`DEBUG_IMAGE_TOPIC`
   = `"asd"`, the local-only trash topic, and the `img_person_detecion` misspelling).
3. **Display keeps working with less CPU**: web_video_server serves CompressedImage
   topics via `?type=ros_compressed` as **passthrough MJPEG** — the JPEG encoded once at
   5 Hz in the vision node replaces server-side re-encoding of a 30 Hz raw stream.
4. Migration order: helper → HRIC nodes → detector/base nodes (the 30 Hz one first) →
   the rest; keep old raw topic names alive behind the same gate for one transition
   window, then delete.

## 5. Display per subtask step

**Today (audited):** the display is a **Next.js app in Firefox kiosk** (per-task routes:
`/hric`, `/gpsr`, `/laundry`, `/ppc`, `/restaurant`) + `rosbridge_websocket` (:9090) +
`web_video_server` (:8080). Video = an `<img>` MJPEG stream pointed at whatever topic
name arrives on `/hri/display/change_video` (`hri.publish_display_topic`). The step bar
listens on `/hri/display/task_step` (`hri.publish_display_step`). So the per-task pages
and step tracking already exist — the gaps are: (a) TMs hand-publish topic/step
inconsistently, (b) several subtasks have no annotated stream to show, (c) the page shows
no live vision *status* (found/not-found/latency).

Plan (no new compositor node needed — the web app is the compositor):
1. **Stream contract**: every vision service call renders its annotated frame (bboxes,
   seat rectangle, wrist point, tracked person) to its `/vision/debug/<node>/compressed`
   topic (§4). Nodes already draw these overlays; they just need the gated publisher.
2. **`@display_step` decorator in vision_tasks.py** — publishes the step name on
   `/hri/display/task_step` and the node's debug topic on `/hri/display/change_video`
   on entry (plain String topics — no hri_tasks dependency needed), and pushes the
   resulting `Status` on exit:
   ```python
   @display_step("Finding a seat", stream=HRIC_DEBUG_TOPIC)
   def find_seat(self): ...
   ```
   The FSMs stop hand-publishing display topics; the display follows the task
   automatically.
3. **Subtask status ticker**: new topic `/hri/display/subtask_status`
   (String, JSON: `{function, status, ms}`) published by the decorator on exit; the
   Next.js pages render it as a footer/ticker (DOM overlay — crisper than burning text
   into frames, and it doubles as a live log of vision results for demos/judges).
4. Page polish per task: the step bars already map `task_step` strings — align the
   decorator's step names with each page's `TASK_STEPS` keys so the bar advances without
   extra wiring.

## 6. Phased rollout (competition-safe ordering)

**Phase 0 — quick wins, no behavior risk (≈ 1 day)**
- Launch cleanups (§2.5). Re-apply the three hric_commands fixes from the Orin tree.
- `fetch_models.py` + MANIFEST + `--warmup` target (§1.1-2).
- `_call` helper + timeout statuses in vision_tasks (§3.1) — pure refactor, mock-tested.
- DebugImagePublisher + adopt in the HRIC nodes (hric_commands, face_rec, tracker).

**Phase 1 — structural (2–4 days, test per task)**
- Centralized MODELS_DIR wiring in trt_utils + registry (§1.3-4).
- Pose-model consolidation (§2.1) and moondream yolov8n drop (§2.2).
- Executor/busy-wait fixes (§2.3), activity gating (§2.4).
- Registry-driven service map + contract CI (§3.2-3).

**Phase 2 — polish**
- @display_step decorator + subtask status ticker in the Next.js pages (§5).
- Full debug-namespace migration, remove raw debug topics (§4).
- Dead-surface deletion (§3.4), Dockerfile cleanup (yolov5, insightface pin).

Success criteria: vision container VRAM −1.5 GB and idle CPU −30 % on the Orin (measure
with jtop before/after); zero first-run model downloads/exports on competition day;
`test_vision_contract` green in CI; display shows step-labeled annotated frames for every
vision-touching FSM state.

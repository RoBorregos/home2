# Migration Ubuntu 22.04/Humble/JetPack 6 → Ubuntu 24.04/Jazzy/JetPack 7

## Image names

The images keep the same names as on `main` (`l4t_base`, `vision-l4t`,
`navigation-l4t`, `manipulation-l4t`, `roudi-l4t`, `zed-l4t`, `hri-l4t`,
`hri-stt-l4t`, `hri-tts-l4t`, `integration-cpu`, etc.) — with no `jazzy_`
prefix. This Orin is dedicated exclusively to Jazzy, so there is no need to
tell them apart by name; `docker/l4t.yaml`, `docker/cpu.yaml` and
`docker/cuda.yaml` replace the old base recipes (Humble/`dustynv`) outright
instead of living alongside them.

## Rationale

The team is migrating to the Jetson AGX Thor (JetPack 7, L4T r38+, Ubuntu
24.04, no ROS 2 Humble support). It was validated first on an Orin AGX devkit
already running JetPack 7.2 / L4T R39.2 / Ubuntu 24.04 (noble) / CUDA 13.2 /
cuDNN 9 / TensorRT 10.16, Ampere GPU `sm_87`.

## Base image (`docker/Dockerfile.ROS`, `docker/Dockerfile.ROS-l4t`)

- `ROS_DISTRO` is parameterized (`ARG`/`ENV`), default `humble` left untouched,
  but the new compose files (`docker/jazzy_cpu.yaml`, `docker/jazzy_cuda.yaml`,
  `docker/jazzy_l4t.yaml`) set it to `jazzy`.
- Ubuntu 24.04 ships a stock `ubuntu` user/group at UID/GID 1000; it is deleted
  before creating the `ros` user (it collides with the host's typical UID/GID
  1000).
- `PIP_BREAK_SYSTEM_PACKAGES=1` — Ubuntu 24.04 enforces PEP 668.
- `PIP_INDEX_JETSON` changed from `jp6/cu126` (JetPack 6) to `sbsa/cu130`
  (aarch64 + CUDA 13, cp312) — jetson-ai-lab has no dedicated `jp7` index. This
  was the root cause of everything still resolving JetPack 6 packages even
  though the base image was already JetPack 7.
- `Dockerfile.ROS-l4t` now bakes in Jetson's own APT repo
  (`repo.download.nvidia.com/jetson/{common,som,ffmpeg}`, via
  `docker/jetson-apt/`) so CUDA/cuDNN are available at build time (not only at
  runtime through CDI/CSV mounts), which is required to compile dlib (vision)
  and CTranslate2 (hri-stt).
- `ENV NVIDIA_VISIBLE_DEVICES=all` / `NVIDIA_DRIVER_CAPABILITIES=all` added to
  the l4t base. The old `dustynv/l4t-pytorch` had them baked in (standard in
  NVIDIA's Jetson images); ours (`ubuntu:24.04` + ROS) does not, so
  `runtime: nvidia` in the compose files alone was not enough — any process
  that only checked `runtime: nvidia` without the env var (e.g. CTranslate2 in
  hri-stt) did not see the GPU and silently fell back to CPU/int8.

## Host configuration (Orin), outside the repo

- The kernel's `net.core.rmem_max`/`wmem_max` were at Ubuntu's default
  (~208 KB), far below the 10 MB CycloneDDS asks for on its socket — without
  this, **no** ROS 2 node could create its DDS domain
  (`rmw_create_node: failed to create domain, error Error`), in any container.
  Raised to 2 GB via `/etc/sysctl.d/60-cyclonedds.conf` on the Orin (outside the
  repo — it is host config, not image config).

## By area

- **hri**: `dockerfiles/Dockerfile.ROS` with no fundamental changes (audio deps
  + pip). `Dockerfile.stt-l4t`/`Dockerfile.tts-l4t` migrated from
  `dustynv/l4t-pytorch:r36.4.0` (no JetPack 7 tag) to `jazzy_l4t_base`.
  `hri-ros.yaml`/`hri/run.sh` had real bugs that kept the build on the old base
  despite the migration (`BASE_IMAGE` without the `jazzy_` prefix,
  `TTS_BASE_IMAGE` not set for l4t). Requirements (`nlp.txt`, `speech.txt`)
  updated: pydantic 1→2, spacy/thinc, onnxruntime, scipy, torchaudio,
  openwakeword (installed `--no-deps`; its only hard dependency without an
  aarch64/cp312 wheel is tflite-runtime, unused by this code — only the ONNX
  path is), piper (no real use, removed), deepfilterlib (needs `cargo`/`rustc`
  to compile). PyAV (cloned without a branch pin, same as on `main`) started
  failing with `make: uv: No such file or directory` — its upstream build script
  now requires `uv`; `pip install uv` was added before that step.
- **vision**: `ros-humble-*` → `ros-${ROS_DISTRO}-*`. dlib compiled with
  `DLIB_USE_CUDA_COMPUTE_CAPABILITIES=87` (previously rejected by CUDA 13).
  Unlike the old `dustynv/l4t-pytorch`, `jazzy_l4t_base` does not ship OpenCV
  with CUDA preinstalled — the Dockerfile now compiles OpenCV 4.14.0 with CUDA
  from source (same script/patches as navigation). `numpy>=2` is installed
  before compiling OpenCV so its Python bindings stay ABI-compatible with
  torch/onnxruntime-gpu (which require NumPy ≥2) — compiling against NumPy 1.x,
  as `navigation` does, breaks onnxruntime-gpu here because vision needs both in
  the same process. `ultralytics` and `insightface` are installed with
  `--no-deps` (both depend on `opencv-python`, which would clobber the CUDA
  build). `cv_bridge` rebuilt from source against the active NumPy.
- **navigation** (`Dockerfile.l4t` only; `cpu`/`.cuda` get the
  `ros-humble-*` → `ros-${ROS_DISTRO}-*` + `ubuntu:24.04` rename, untested):
  - Base `dustynv/l4t-pytorch` → `l4t_base`. Since it no longer ships
    OpenCV-CUDA or PyTorch: OpenCV **4.14.0** with CUDA from source (4.10.0 does
    not compile with CUDA 13; `cudacodec` off), explicit torch
    (`torch==2.9.0 torchvision`) and `unzip` added to apt.
  - rtabmap with `-DCMAKE_CXX_STANDARD=20`. Nav2/BehaviorTree.CPP/STVL on
    `jazzy` branches (`BTCPP_TAG=4.6.2`).
  - `--skip-keys`: `+pcl +eigen3`, `-dashgo_driver`.
  - CycloneDDS/iceoryx: the selective `dpkg -r` is no longer enough (two
    `libiceoryx_posh.so` in one process → SIGSEGV in RouDi); the whole DDS apt
    set is purged and `rmw_cyclonedds_cpp` is rebuilt from source. `libacl1-dev`
    via `apt-get download` + `dpkg -i` (apt-get install refuses with the broken
    deps left by the Nav2 removal). iceoryx with `-DINTROSPECTION=ON` (parity
    with `docker/roudi/Dockerfile`).
  - `numpy<2` at the end: `pip install torch` drops NumPy 2.x into `~/.local`,
    which breaks `import cv2`.
  - `nav2_omni.yaml` / `nav2_omni_limp.yaml`: the STVL plugin **stays** as
    `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer` (with the slash). Do
    NOT migrate it to `::`: the `costmap_plugins.xml` on the `jazzy` branch (the
    one the Dockerfiles clone, `STVL_BRANCH=jazzy`) declares it as
    `name="spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"`, and pluginlib
    resolves by `name`. With `::` the costmap does not load the layer.
- **manipulation**: `ros-humble-*` → `ros-${ROS_DISTRO}-*`; base changed from
  `dustynv/l4t-pytorch:r36.4.0` to `jazzy_l4t_base`; `libvtk-qt` added to
  skip-keys (ROS's rosdep base.yaml still points at `libvtk7-qt-dev`, which does
  not exist on noble; only used by the rtabmap_viz GUI, not in headless
  operation).
- **roudi**: iceoryx recompiled with prefix `/opt/ros/${ROS_DISTRO}`.
- **display**, **integration**, **simulation**: `ros-humble-*` →
  `ros-${ROS_DISTRO}-*` package rename, same iceoryx/cyclonedds pattern.
- **zed**: Stereolabs SDK updated from `zedsdk/5.0/l4t36.4` to
  `zedsdk/5.4/l4t38.4` (Stereolabs did publish a build for L4T r38);
  `zed-ros2-wrapper` from branch `humble-v5.0.0` to `v5.4.1`. The `ros` user was
  not in the `zed` group the SDK installer creates (permissions 770 on
  `/usr/local/zed/lib`) — it is now added via `usermod -aG zed ros` in the
  Dockerfile, and `docker-compose.yaml` is fixed to pass `group_add: zed` by
  name instead of a stale hardcoded GID (`1001`, which no longer matches the
  group's real GID after the rebuild).
- **frida_interfaces_cache**: the l4t compose rebuilt the image from
  `ubuntu:22.04` with `Dockerfile.ROS`, silently overwriting the real
  `jazzy_l4t_base` tag with a broken image — fixed to reference the
  already-built image, with no rebuild of its own.

## Verified (build + GPU smoke test) on the real Orin AGX

- `jazzy_l4t_base`, `hri-l4t`, `hri-stt-l4t`, `hri-tts-l4t` — CUDA/torch
  confirmed working inside the container.
- `jazzy_vision-l4t` — cv2 4.14.0 (CUDA, 1 device detected), torch,
  onnxruntime-gpu, dlib, ultralytics and insightface, all importing and working
  together in the same process (shared NumPy 2.x).
- `jazzy_navigation-l4t` — numpy, cv2 with CUDA, torch, all verified.
- `jazzy_manipulation-l4t`, `jazzy_roudi-l4t` — rebuilt against the fixed base;
  manipulation with torch CUDA confirmed.
- `jazzy_integration-cpu` — build and container startup verified.
- `jazzy_zed-l4t` — successful build, SDK and group permissions correct.

## End-to-end hri test (`./run.sh --hric l4t`)

With the `NVIDIA_VISIBLE_DEVICES` and CycloneDDS sysctl fixes, the full `hric`
stack (hri-ros, stt, tts, postgres, llamacpp) came up:
- `hri-stt`: went from `Using device: cpu with compute type: int8` (failing with
  `ValueError: Requested int8 compute type...`) to
  `Using device: cuda with compute type: float16` — working.
- `hri-ros`: `llm_utils` and `extract_data` initialize and run correctly against
  real CycloneDDS/DDS (not just isolated imports).
- `extract_data` needed the spacy model `en_core_web_md`, which no Dockerfile
  downloaded (not even on `main`) — a pre-existing setup gap, not a migration
  one. The runtime fallback (`spacy.cli.download` + `spacy.load` inside the same
  process) failed because the container runs with a UID/GID that has no entry in
  `/etc/passwd` (`user: ${LOCAL_USER_ID}`), so `pip install --user` does not land
  in a `site-packages` the process can see. `RUN python3 -m spacy download
  en_core_web_md` was added to `Dockerfile.ROS` (next to `nlp.txt`, running as
  root at build time) so this is not needed at runtime — but `extract_data.py`
  still called `spacy.cli.download()` unconditionally on every startup (it never
  tried loading the already-installed package first), so it hit the same UID
  problem anyway. The `try/except` in
  `hri/packages/nlp/scripts/extract_data.py` was reordered to try
  `spacy.load(spacy_model)` (the already-baked package) before falling back to
  `spacy.cli.download()`.
- `nlp.txt`/`speech.txt` had several stale pins that stopped resolving on Python
  3.12/aarch64 on a clean build of `hri-ros` (the cached image we had been using
  predated these requirements and never exposed it): `pydantic==1.10.11` in
  `nlp.txt` forced pip to resolve `thinc==9.1.1` (the only thinc version
  compatible with pydantic 1.x), which has no aarch64/cp312 wheel and fails
  compiling Cython from source — the pin was dropped (nothing in `nlp/` uses the
  pydantic 1.x API). In `speech.txt`: `onnxruntime==1.16.3` and `scipy==1.10.1`
  no longer have a Python 3.12 wheel (bumped to `1.17.3`/`1.11.4`);
  `openwakeword==0.6.0` was duplicated (the Dockerfile already installs it
  separately with `--no-deps`, precisely because its `tflite-runtime` dependency
  has no wheel here) — removed from the requirements; `piper-tts`/`piper` are
  imported by no script and `piper-tts` requires `piper-phonemize`, which has no
  wheel available — both removed; `torchaudio<=2.5.0` (a stale pin, no longer
  needed thanks to the `df/io.py` patch below) forced a torchaudio version
  incompatible with the torch that `nlp.txt`/`postgres.txt` install
  (`torch 2.14.0`), the same kind of CUDA ABI breakage documented below — pinned
  to `torchaudio==2.11.0` (the version that actually resolves alongside the
  rest).
- With `nlp.txt`/`speech.txt`/`postgres.txt` installing cleanly, a second, more
  subtle problem appeared: `voice_detection.py`, `noise_cancellation.py` and
  `llm_utils.py` died with `ValueError: numpy.dtype size changed, may indicate
  binary incompatibility. Expected 96 from C header, got 88 from PyObject` when
  importing `scipy.spatial.transform`. `pip show`/`import numpy` confirmed
  `numpy 2.5.2` and `scipy` were correct — the problem was not which version
  ended up installed, but that **`scipy==1.18.1` (the latest at the time) has a
  real ABI bug against `numpy 2.5.2` in this environment**. Confirmed by
  reinstalling several scipy versions live inside the running container and
  testing `from scipy.spatial.transform import Rotation`: `1.16.2`, `1.15.3`,
  `1.14.1` and `1.13.1` work, `1.18.1` does not. `scipy==1.16.2` (with
  `numpy==2.5.2`) was pinned consistently across **all three** requirements
  (`nlp.txt`, `speech.txt`, `postgres.txt`) — each one is a separate pip
  invocation in the Dockerfile, so a single file without the pin (e.g.
  `postgres.txt`, which pulls scipy transitively via
  `scikit-learn`←`sentence_transformers`) reintroduces the broken version. Same
  reason for pinning `pydantic==2.13.5` in `nlp.txt`+`postgres.txt` and
  `sentence_transformers==2.6.1` in `postgres.txt` (unpinned, it pulled a newer
  version that raised `transformers` above what `nlp.txt` pins, silently,
  between separate pip invocations). `deepfilternet` complicates this further:
  its metadata demands `numpy<2.0` even though its compiled part
  (`deepfilterlib`) is Rust/PyO3, not Cython, and does not actually depend on
  the numpy ABI — pinning it together with `scipy==1.18.1` (numpy≥2.0) in the
  same file gave an outright `ResolutionImpossible`. It is installed separately
  with `pip install --no-deps deepfilternet==0.5.6 deepfilterlib==0.5.6` in
  `Dockerfile.ROS` (same pattern as `openwakeword`), with its real dependencies
  (`appdirs`, `loguru`) added explicitly to `speech.txt`.
- `noise_cancellation.py` (uses `deepfilternet==0.5.6`, the latest published
  version) failed with `ModuleNotFoundError: No module named
  'torchaudio.backend'`. Torchaudio 2.11+ removed its old I/O API entirely
  (`torchaudio.info()`, `torchaudio.backend.common.AudioMetaData`) in favor of
  `torchaudio.io`; there is no torchaudio version that is both ABI-compatible
  with torch 2.13.0/CUDA 13 and still has that old API (pinning
  `torchaudio<=2.5.0` breaks torch's CUDA binding). deepfilternet only uses
  `AudioMetaData`/`torchaudio.info()` to read a file's sample rate before
  loading it — something `soundfile` (already a dependency) does just as well.
  `df/io.py` is patched at build time (`Dockerfile.ROS`, after installing
  `speech.txt`) to replace that single call with
  `soundfile.info(file).samplerate`, without touching the package itself.
  Verified: `NoiseCancellation node ready` + DeepFilterNet initializes and loads
  the full model without errors.
- `hri-tts`: initially failed on ALSA audio (`Couldn't open audio device`). Real
  cause: the Orin uses PipeWire-Pulse (PulseAudio's replacement in Ubuntu
  24.04), running but with its actual socket at `/run/user/<uid>/pulse/native` —
  not at `~/.config/pulse/pulseaudio.socket`, which is what the compose mounts
  and `PULSE_SERVER` points at. On top of that, without `SDL_AUDIODRIVER=pulse`,
  SDL/pygame tried raw ALSA first (with no `/dev/snd` mounted) before even
  trying pulse. Fixed in `docker/hri/compose/tts.yaml`: mount
  `/run/user/${LOCAL_USER_ID}/pulse` directly (not `~/.config/pulse`, which only
  holds the cookie) and add `SDL_AUDIODRIVER: pulse`. Verified: the Kokoro
  server starts cleanly against the Orin's real audio sink.
- `edge-impulse` (door/kws): AWS containers specific to Jetson Orin 6.0, not
  thoroughly tested — low priority, already flagged in earlier phases as
  potentially tied to JetPack 6.
- `hri-ros` (`docker/hri/compose/hri-ros.yaml`) had the same audio problem as
  `tts.yaml` (classic PulseAudio socket instead of the PipeWire-Pulse one) — it
  made `audio_capturer.py` fail with `PyAudio: Invalid input device`. The same
  fix was applied: mount `/run/user/${LOCAL_USER_ID}/pulse` directly and add
  `SDL_AUDIODRIVER: pulse` to the shared `x-speech-devices`.
- `frida_interfaces_cache` (builds `frida_interfaces`/`frida_constants`/
  `xarm_msgs` before `hri`) can be left in an incomplete build (e.g. interrupted
  by a reboot) with nothing detecting it: `lib.sh` only rebuilds the cache if the
  `build/` folder does not exist, not if the build inside it failed halfway.
  When that happens, `docker/frida_interfaces_cache/{build,install,log}` must be
  deleted by hand (may need `sudo rm -rf` if the container ran as root because
  `UID`/`GID` were not exported) and `./run.sh hri --build` rerun. Detection of
  an incomplete build was not automated in this iteration.

## zed — verified with a real camera (ZED2 over USB)

With the camera connected, 4 chained problems were found and fixed (each one
masking the next):
1. `docker/zed/.env` on the Orin still had `BASE_IMAGE`/`IMAGE_NAME` pointing at
   `jazzy_l4t_base`/`jazzy_zed-l4t` — it was not updated in the image rename.
   Fixed.
2. `zed-l4t` had been built before the `NVIDIA_VISIBLE_DEVICES` fix in the base
   — without it, `libcuda.so.1` was not found and the `zed_camera_component`
   failed to load. Rebuilt.
3. The host udev rule for Stereolabs' vendor ID (`2b03`,
   `/etc/udev/rules.d/99-slabs.rules`) was missing — without it, the camera's
   MCU/sensors gave `Permissions denied`. This is host config (normally created
   by the SDK installer when run natively, not inside a container), so it never
   existed here. Created.
4. `/usr/local/zed/settings` and `/usr/local/zed/resources` on the host (mounted
   into the container) were `root:root` with no write permission — the SDK needs
   to write the camera's calibration file there (downloaded by serial) and the
   TensorRT-optimized neural depth model (~26 MB, compiled on first run).
   `chown 2002:2002` (the container's UID) on both.

With all 4 fixes: `=== zed started ===`, positional tracking active, publishing
real RGB/depth/IMU/point cloud — `rgb/color/rect/image` confirmed at ~30 Hz via
`ros2 topic hz`.

## Single `l4t_base` base image (size optimization)

The Jazzy l4t images were much heavier than on `main` (`l4t_base` 26 GB,
`hri-l4t` 53 GB, `manipulation-l4t` 50 GB). `docker history` showed the cause
was not PYTHONPATH but three things:

1. **The base carried the full `cuda-toolkit-13-2` plus the `-dev` packages: a
   14.3 GB layer.** It included `libnvinfer_static.a` (3.3 GB), Nsight
   Systems/Compute (1.7 GB) and CUDA's static libraries (~4 GB). Every area
   inherited it.
2. **torch was installed from PyPI (2.14.0+cu130).** The Jetson index only goes
   up to 2.11, so pip picked PyPI's because it was newer. That wheel brings its
   own CUDA/cuDNN in `nvidia-*` packages (3.3 GB) plus `triton`, duplicating
   what was already on the system. On top of that, `torchaudio` did not match
   torch.
3. **`PIP_IGNORE_INSTALLED=1` made every `pip install` reinstall its whole
   dependency tree.** In hri-l4t, torch ended up twice (9.66 GB and 6 GB
   layers). Added to that were OpenCV compiled in vision and in navigation
   without deleting the build tree, and iceoryx/CycloneDDS compiled in 5
   Dockerfiles.

The old `dustynv/l4t-pytorch` was light because it already shipped torch/OpenCV
compiled against the system CUDA in a single shared layer. `docker/Dockerfile.ROS-l4t`
now plays that same role, and every l4t area inherits from it as on `main`:

- **CUDA 13.2 / cuDNN 9 / TensorRT 10.** Compiler, headers and shared libraries
  are installed, without the `cuda-toolkit` meta-package (Nsight) and without
  `*_static*.a` libraries. TensorRT's builder resources for dGPUs
  (`sm90`/`sm100`) are also deleted; `sm110` (Thor) is kept.
- **Shared Python stack:** `torch 2.11.0`, `torchvision 0.26.0` and
  `torchaudio 2.11.0` (matching versions) come from **PyPI**.
  - The `jetson-ai-lab sbsa/cu130` wheels **do not work on Orin**: they only
    carry kernels for `sm_110` (Thor) and `sm_121` (Spark). On the Orin
    (`sm_87`) they fail with `no kernel image is available for execution on the
    device`. PyPI's cu130 build carries `sm_80`, which runs on the Orin, and
    `sm_110`, which covers Thor.
  - torch's metadata has the `cuda-toolkit`, `cuda-bindings`,
    `nvidia-cudnn-cu13` and `triton` dependencies stripped, so it uses the
    system's CUDA 13.2 and cuDNN 9.20. Only NCCL, NVSHMEM and cuSPARSELt, which
    JetPack does not ship, are installed as wheels. Without triton,
    `torch.compile` is unavailable.
  - On the Orin, torch prints a compute-capability warning (8.7 vs 8.0), but
    matmul, conv (cuDNN) and `torchvision.ops.nms` work on the GPU.
  - `onnxruntime-gpu 1.24.0` still comes from the Jetson index (TensorRT EP).
  - Also: `numpy 2.5.2`, `scipy 1.16.2`, OpenCV 4.14 + contrib with CUDA
    (`docker/scripts/build_opencv.sh`, build tree removed) and
    `cv_bridge`/`image_geometry` compiled against that OpenCV.
- **GPU-less builds:** `docker build` has no NVIDIA runtime. The steps that
  import torch/cv2 use `with-cuda-stub <cmd>`, which adds the `libcuda.so.1`
  stub for that command only.
- **Placeholder `.deb` packages:** iceoryx and cv_bridge, compiled from source,
  are registered as empty apt packages (version 99, on hold) with
  `install-placeholder-deb`. This keeps apt from ending up with broken
  dependencies (they used to be removed with `dpkg --force-depends`) and stops
  rosdep reinstalling the apt ones.
- **iceoryx 2.0.6 (expanded limits) + CycloneDDS 0.10 with SHM** are compiled
  once. Shared memory stays off by default; each area that uses it sets
  `ENV CYCLONE_SHM=1` and reruns `cyclonedds_setup.sh`.
- **No `PIP_BREAK_SYSTEM_PACKAGES` and no `PIP_IGNORE_INSTALLED`:**
  - `/usr/lib/python3.12/EXTERNALLY-MANAGED` is deleted.
  - The Python packages apt installs for ROS and that the areas upgrade (numpy,
    pillow, pyyaml, requests…) are installed once into `/usr/local` with
    `--ignore-installed`. From then on pip sees them first and can upgrade them
    normally.
- **`/etc/pip.conf`** sets the Jetson index, `no-cache-dir` and
  `constraint = /etc/pip/constraints.txt` (`docker/scripts/constraints-l4t.txt`).
  Because it lives in `pip.conf` and not in `ENV`, it also applies under
  `sudo pip`. No area can reinstall or change the version of
  torch/numpy/scipy/OpenCV/onnxruntime: if a requirement clashes, the build
  fails instead of silently bloating the image.
- **Placeholder distributions** (`opencv-python*`, `onnxruntime`): empty
  `dist-info` directories carrying the base's version. This way `ultralytics`,
  `insightface`, `faster-whisper`, etc. consider the dependency satisfied and do
  not install the CPU wheels from PyPI over it. `ros-jazzy-cv-bridge` and
  `ros-jazzy-image-geometry` are placeholder packages too, so apt/rosdep do not
  install the apt ones on top.
- **`pip-install-reqs a.txt b.txt`** is a base helper for the requirements files
  shared with cpu/cuda. It drops lines for packages the base already provides
  (for example `numpy<2` in `tts.txt` or `onnxruntime==1.17.3` in `speech.txt`)
  and does a single `pip install`. The shared Dockerfiles
  (`hri/dockerfiles/Dockerfile.ROS`, `integration`) only use it if it exists.

Changes by area:

- **vision:** no longer compiles OpenCV, does not pip-install
  torch/numpy/onnxruntime, does not rebuild cv_bridge and does not compile
  iceoryx/cyclone. dlib is compiled in a single step.
- **navigation:** no longer compiles OpenCV nor runs `pip install torch` as the
  `ros` user into `~/.local`. That is why the `PYTHONPATH` and `LD_LIBRARY_PATH`
  entries pointing at `~/.local` are also removed, along with the
  `pip install --force-reinstall "numpy<2"` (now numpy 2, like everything else).
  rtabmap and rtabmap_ros are pinned by commit. OpenVDB is compiled in a single
  layer and its sources deleted.
- **manipulation:** uses the base's torch/numpy. No longer clones
  `RoBorregos/home2` (it pulled the default branch); `rosdep` reads this
  checkout's `package.xml` files through a BuildKit bind mount.
- **hri-ros:** `nlp.txt`, `speech.txt` and `postgres.txt` are installed in a
  single resolver run.
- **stt:** CTranslate2 and PyAV are pinned by commit, compiled and cleaned up in
  a single step, and installed before the requirements (so the final
  `--force-reinstall` is not needed).
- **tts:** uses the base directly.
- **roudi, zed, display:** use the base's iceoryx/cyclone. roudi and display
  still compile it only if their base does not provide it (cpu/cuda).

- **moondream-server:** a single `pip install`; on l4t it uses the base's
  numpy/OpenCV/torch.
- **stt:** the FFmpeg libraries PyAV compiles are copied to `/usr/local/lib`, so
  the `command` in `stt-l4t.yaml` no longer runs
  `source /tmp/PyAV/scripts/activate.sh` (that directory no longer exists).
- **navigation:** `LD_LIBRARY_PATH` includes the base's `torch/lib`, because
  rtabmap links against libtorch.

Measured result on the Orin (2026-09-15). The "Before" column is `docker images`
for the baseline. Under "Now", the total is `docker images` and the own size
comes from `docker system df -v`: what the image adds on top of `l4t_base`,
which is stored on disk only once.

| Image | Before | Now (total) | Now (own) |
|---|---|---|---|
| l4t_base | 26.3 GB | 16.2 GB | — |
| hri-l4t | 53.6 GB | 17.9 GB | 1.7 GB |
| manipulation-l4t | 50.3 GB | 22.2 GB | 6.0 GB (MoveIt + the repo's rosdep) |
| moondream-server | 42.6 GB | 16.5 GB | 0.3 GB |
| vision-l4t | 39.9 GB | 16.9 GB | 0.7 GB |
| navigation-l4t | 38.7 GB | 23.1 GB | 7.0 GB (rtabmap, Nav2, OpenVDB) |
| hri-tts-l4t | 36.4 GB | 17.0 GB | 0.9 GB |
| hri-stt-l4t | 30.2 GB | 16.6 GB | 0.5 GB |
| zed-l4t | 28.2 GB | 17.2 GB | 1.1 GB |
| display-l4t | 27.8 GB | 17.0 GB | 0.8 GB |
| roudi-l4t | 26.3 GB | 16.2 GB | 0 |
| integration-l4t | 21.0 GB | 16.6 GB | 0.5 GB |

On disk, the 12 l4t images take ~16 GB shared plus ~20 GB of their own (~36 GB
in total). They used to be ~190 GB, because each one carried its own torch/CUDA.
Every image passed a smoke test on a real GPU:
- **All:** torch matmul/conv on CUDA.
- **vision:** cv2.cuda, TensorRT EP, dlib CUDA, YOLO, cv_bridge.
- **hri:** spaCy, DeepFilterNet, openwakeword, sentence-transformers.
- **stt:** CTranslate2 with float16 on CUDA, PyAV, faster-whisper.
- **navigation:** SuperPoint TorchScript; rtabmap linked against OpenCV 4.14 and
  torch.
- **tts:** kokoro.
- **manipulation:** ultralytics, CLIP.

To add or change a torch/numpy/OpenCV version: edit
`docker/scripts/constraints-l4t.txt` (and the wheel URL in `Dockerfile.ROS-l4t`
if it is torch) and rebuild the base.

## Pending / out of scope for this iteration

- The `cpu`/`cuda` flavors of each area: not prioritized in this iteration
  (exclusive focus on `l4t`, which is the robot's real hardware).

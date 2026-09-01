# Migración Ubuntu 22.04/Humble/JetPack 6 → Ubuntu 24.04/Jazzy/JetPack 7

## Nombres de imagen

Las imágenes usan los mismos nombres que en `main` (`l4t_base`, `vision-l4t`,
`navigation-l4t`, `manipulation-l4t`, `roudi-l4t`, `zed-l4t`, `hri-l4t`,
`hri-stt-l4t`, `hri-tts-l4t`, `integration-cpu`, etc.) — sin prefijo
`jazzy_`. Esta Orin queda dedicada exclusivamente a Jazzy, así que no hace
falta distinguir por nombre; `docker/l4t.yaml`, `docker/cpu.yaml` y
`docker/cuda.yaml` reemplazan directamente las recetas de base viejas
(Humble/`dustynv`) en vez de vivir en paralelo.

## Motivo

El equipo migra hacia Jetson AGX Thor (JetPack 7, L4T r38+, Ubuntu 24.04, sin
soporte para ROS 2 Humble). Se validó primero en una Orin AGX devkit que ya
corre JetPack 7.2 / L4T R39.2 / Ubuntu 24.04 (noble) / CUDA 13.2 / cuDNN9 /
TensorRT 10.16, GPU Ampere `sm_87`.

## Imagen base (`docker/Dockerfile.ROS`, `docker/Dockerfile.ROS-l4t`)

- `ROS_DISTRO` parametrizado (`ARG`/`ENV`), default `humble` sin tocar, pero
  los compose nuevos (`docker/jazzy_cpu.yaml`, `docker/jazzy_cuda.yaml`,
  `docker/jazzy_l4t.yaml`) lo fijan a `jazzy`.
- Ubuntu 24.04 trae un usuario/grupo `ubuntu` en UID/GID 1000 de fábrica; se
  borra antes de crear el usuario `ros` (colisiona con el UID/GID 1000 típico
  del host).
- `PIP_BREAK_SYSTEM_PACKAGES=1` — Ubuntu 24.04 aplica PEP 668.
- `PIP_INDEX_JETSON` cambiado de `jp6/cu126` (JetPack 6) a `sbsa/cu130`
  (aarch64 + CUDA 13, cp312) — no existe índice `jp7` dedicado en
  jetson-ai-lab. Esta era la causa raíz de que todo siguiera resolviendo
  paquetes de JetPack 6 pese a que la imagen base ya era JetPack 7.
- `Dockerfile.ROS-l4t` ahora hornea el repo APT propio de Jetson
  (`repo.download.nvidia.com/jetson/{common,som,ffmpeg}`, vía
  `docker/jetson-apt/`) para tener CUDA/cuDNN disponibles en build time (no
  solo en runtime vía CDI/CSV mounts), necesario para compilar dlib
  (vision) y CTranslate2 (hri-stt).
- `ENV NVIDIA_VISIBLE_DEVICES=all` / `NVIDIA_DRIVER_CAPABILITIES=all`
  agregados a la base l4t. La vieja `dustynv/l4t-pytorch` los traía horneados
  (estándar en imágenes Jetson de NVIDIA); nuestra base (`ubuntu:24.04` +
  ROS) no, así que `runtime: nvidia` solo en los compose no bastaba —
  cualquier proceso que solo revisara `runtime: nvidia` sin el env var
  (p. ej. CTranslate2 en hri-stt) no veía la GPU y caía a CPU/int8 en
  silencio.

## Configuración del host (Orin), fuera del repo

- `net.core.rmem_max`/`wmem_max` del kernel venían en el default de Ubuntu
  (~208KB), muy por debajo de los 10MB que CycloneDDS pide para su socket —
  sin esto, **ningún** nodo ROS 2 podía crear su dominio DDS
  (`rmw_create_node: failed to create domain, error Error`), en cualquier
  contenedor. Se subió a 2GB vía `/etc/sysctl.d/60-cyclonedds.conf` en la
  Orin (fuera del repo, es config de host, no de imagen).

## Por área

- **hri**: `dockerfiles/Dockerfile.ROS` sin cambios de fondo (deps de audio +
  pip). `Dockerfile.stt-l4t`/`Dockerfile.tts-l4t` migrados de
  `dustynv/l4t-pytorch:r36.4.0` (sin tag JetPack 7) a `jazzy_l4t_base`.
  `hri-ros.yaml`/`hri/run.sh` tenían bugs reales que hacían que el build
  siguiera usando la base vieja pese a la migración (`BASE_IMAGE` sin
  prefijo `jazzy_`, `TTS_BASE_IMAGE` no seteado para l4t). Requirements
  (`nlp.txt`, `speech.txt`) actualizados: pydantic 1→2, spacy/thinc,
  onnxruntime, scipy, torchaudio, openwakeword (instalado `--no-deps`,
  su única dependencia dura sin wheel aarch64/cp312 es tflite-runtime,
  no usado en este código — solo el path ONNX), piper (sin uso real,
  eliminado), deepfilterlib (necesita `cargo`/`rustc` para compilar).
  PyAV (clonado sin pin de rama, igual que en main) empezó a fallar con
  `make: uv: No such file or directory` — su script de build upstream
  ahora requiere `uv`; se agregó `pip install uv` antes de ese paso.
- **vision**: `ros-humble-*` → `ros-${ROS_DISTRO}-*`. dlib compilado con
  `DLIB_USE_CUDA_COMPUTE_CAPABILITIES=87` (antes rechazado por CUDA 13).
  A diferencia de la vieja `dustynv/l4t-pytorch`, `jazzy_l4t_base` no trae
  OpenCV con CUDA preinstalado — el Dockerfile ahora compila OpenCV 4.14.0
  con CUDA desde fuente (mismo script/patches que navigation). `numpy>=2`
  se instala antes de compilar OpenCV para que sus bindings de Python queden
  ABI-compatibles con torch/onnxruntime-gpu (que requieren NumPy ≥2) —
  compilar contra NumPy 1.x, como hace `navigation`, rompe onnxruntime-gpu
  aquí porque vision necesita ambos en el mismo proceso. `ultralytics` e
  `insightface` se instalan con `--no-deps` (ambos dependen de
  `opencv-python`, que pisaría la build CUDA). `cv_bridge` reconstruido
  desde fuente contra el NumPy activo.
- **navigation**: script de instalación de OpenCV apuntado a 4.14.0 (el
  4.10.0 original no compila contra el CUDA 13/C++17 de este hardware);
  Nav2/BehaviorTree.CPP/STVL apuntados a ramas `jazzy`; 2 rosdep keys
  (`pcl`, `eigen3`) agregadas a `--skip-keys` (paquetes de terceros con
  dependencias no resolubles en el rosdep DB de noble, no bloquean el
  colcon build real que ocurre después desde el volumen montado).
- **manipulation**: `ros-humble-*` → `ros-${ROS_DISTRO}-*`; base cambiada de
  `dustynv/l4t-pytorch:r36.4.0` a `jazzy_l4t_base`; `libvtk-qt` agregado a
  skip-keys (rosdep base.yaml de ROS sigue apuntando a `libvtk7-qt-dev`,
  inexistente en noble; solo usado por la GUI de rtabmap_viz, no en
  operación headless).
- **roudi**: iceoryx recompilado con prefix `/opt/ros/${ROS_DISTRO}`.
- **display**, **integration**, **simulation**: renombrado de paquetes
  `ros-humble-*` → `ros-${ROS_DISTRO}-*`, mismo patrón de iceoryx/cyclonedds.
- **zed**: SDK de Stereolabs actualizado de `zedsdk/5.0/l4t36.4` a
  `zedsdk/5.4/l4t38.4` (Stereolabs sí publicó build para L4T r38);
  `zed-ros2-wrapper` de la rama `humble-v5.0.0` a `v5.4.1`. El usuario `ros`
  no quedaba en el grupo `zed` que crea el instalador del SDK (permisos 770
  en `/usr/local/zed/lib`) — se agrega vía `usermod -aG zed ros` en el
  Dockerfile, y el `docker-compose.yaml` se corrige para pasar `group_add:
  zed` por nombre en vez de un GID viejo hardcodeado (`1001`, que ya no
  coincide con el GID real del grupo tras el rebuild).
- **frida_interfaces_cache**: el compose l4t reconstruía la imagen desde
  `ubuntu:22.04` con `Dockerfile.ROS`, lo que sobrescribía silenciosamente
  el tag `jazzy_l4t_base` real con una imagen rota — corregido para
  referenciar la imagen ya construida, sin rebuild propio.

## Verificado (build + GPU smoke test) en la Orin AGX real

- `jazzy_l4t_base`, `hri-l4t`, `hri-stt-l4t`, `hri-tts-l4t` — CUDA/torch
  confirmados funcionando dentro del contenedor.
- `jazzy_vision-l4t` — cv2 4.14.0 (CUDA, 1 dispositivo detectado), torch,
  onnxruntime-gpu, dlib, ultralytics e insightface, todos importando y
  funcionando juntos en el mismo proceso (NumPy 2.x compartido).
- `jazzy_navigation-l4t` — numpy, cv2 con CUDA, torch, todos verificados.
- `jazzy_manipulation-l4t`, `jazzy_roudi-l4t` — reconstruidos contra la base
  ya corregida; manipulation con torch CUDA confirmado.
- `jazzy_integration-cpu` — build y arranque de contenedor verificados.
- `jazzy_zed-l4t` — build exitoso, SDK y permisos de grupo correctos.

## Prueba end-to-end de hri (`./run.sh --hric l4t`)

Con los fixes de `NVIDIA_VISIBLE_DEVICES` y del sysctl de CycloneDDS, se
levantó el stack completo de `hric` (hri-ros, stt, tts, postgres, llamacpp):
- `hri-stt`: pasó de `Using device: cpu with compute type: int8` (fallando
  con `ValueError: Requested int8 compute type...`) a
  `Using device: cuda with compute type: float16` — funcionando.
- `hri-ros`: `llm_utils` y `extract_data` inicializan y corren
  correctamente contra CycloneDDS/DDS real (no solo imports aislados).
- `extract_data` necesitó el modelo spacy `en_core_web_md`, que no se
  descarga en ningún Dockerfile (ni en `main`) — gap de setup preexistente,
  no de la migración; se instaló manualmente para la prueba.
- `noise_cancellation.py` (usa `deepfilternet==0.5.6`, la última versión
  publicada) fallaba con `ModuleNotFoundError: No module named
  'torchaudio.backend'`. Torchaudio 2.11+ eliminó por completo su antiguo
  API de I/O (`torchaudio.info()`, `torchaudio.backend.common.AudioMetaData`)
  a favor de `torchaudio.io`; no existe versión de torchaudio que sea a la
  vez ABI-compatible con torch 2.13.0/CUDA13 y todavía tenga esa API vieja
  (fijar `torchaudio<=2.5.0` rompe el binding CUDA de torch). deepfilternet
  solo usa `AudioMetaData`/`torchaudio.info()` para leer el sample rate de
  un archivo antes de cargarlo — algo que `soundfile` (ya es dependencia)
  hace igual de bien. Se parcha `df/io.py` en build time (`Dockerfile.ROS`,
  después de instalar `speech.txt`) para reemplazar esa única llamada por
  `soundfile.info(file).samplerate`, sin tocar el paquete en sí. Verificado:
  `NoiseCancellation node ready` + DeepFilterNet inicializa y carga el
  modelo completo sin errores.
- `hri-tts`: falla inicialmente por audio ALSA (`Couldn't open audio device`).
  Causa real: la Orin usa PipeWire-Pulse (reemplazo de PulseAudio en Ubuntu
  24.04), corriendo pero con su socket real en `/run/user/<uid>/pulse/native`
  — no en `~/.config/pulse/pulseaudio.socket`, que es donde el compose
  monta y `PULSE_SERVER` apunta. Además, sin `SDL_AUDIODRIVER=pulse`, SDL/
  pygame intentaba ALSA directo primero (sin `/dev/snd` montado) antes de
  siquiera probar pulse. Arreglado en `docker/hri/compose/tts.yaml`: monta
  `/run/user/${LOCAL_USER_ID}/pulse` directo (no `~/.config/pulse`, que solo
  tiene el cookie) y agrega `SDL_AUDIODRIVER: pulse`. Verificado: el server
  Kokoro arranca limpio contra el sink real de audio de la Orin.
- `edge-impulse` (door/kws): contenedores AWS específicos de Jetson Orin
  6.0, no probados a fondo — bajo prioridad, ya señalados en fases previas
  como potencialmente atados a JetPack 6.

## Pendiente / fuera de alcance de esta iteración

- `zed`: no hay cámara física conectada a este devkit, así que no se pudo
  probar streaming real. Al levantar el contenedor (sin cámara) se encontró
  un problema real, no relacionado a la falta de hardware: `iox-roudi`
  (compilado desde fuente, v2.0.6) usa `memfd_create` para su memoria
  compartida y no expone segmentos nombrados en `/dev/shm`, mientras que el
  componente interno del ZED SDK 5.4 (`zed_components`, con su propio
  iceoryx embebido) intenta abrir un segmento por nombre vía `shm_open` y
  falla (`Unable to create shared memory ... Shared Memory does not exist`).
  Es un conflicto de versión/mecanismo de iceoryx entre nuestro build y el
  SDK de Stereolabs, no un problema de la migración a JetPack7/Jazzy en sí
  — pendiente de investigar si Stereolabs publica una versión de iceoryx
  compatible o si hay que ajustar el build de iceoryx para usar shm nombrada.
- Sabores `cpu`/`cuda` de cada área: no priorizados en esta iteración (foco
  exclusivo en `l4t`, que es el hardware real del robot).

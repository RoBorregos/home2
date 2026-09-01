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

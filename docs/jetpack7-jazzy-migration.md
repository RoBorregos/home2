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
- `extract_data` necesitaba el modelo spacy `en_core_web_md`, que no se
  descargaba en ningún Dockerfile (ni en `main`) — gap de setup preexistente,
  no de la migración. El fallback en runtime (`spacy.cli.download` +
  `spacy.load` dentro del mismo proceso) fallaba porque el contenedor corre
  con un UID/GID sin entrada en `/etc/passwd` (`user: ${LOCAL_USER_ID}`), así
  que `pip install --user` no cae en un `site-packages` que el proceso pueda
  ver. Se agregó `RUN python3 -m spacy download en_core_web_md` en
  `Dockerfile.ROS` (junto a `nlp.txt`, corre como root en build time) para
  no depender de esto en runtime — pero `extract_data.py` seguía llamando a
  `spacy.cli.download()` incondicionalmente en cada arranque (nunca
  intentaba cargar el paquete ya instalado primero), así que igual pegaba
  contra el mismo problema de UID cada vez. Se reordenó el `try/except` en
  `hri/packages/nlp/scripts/extract_data.py` para intentar
  `spacy.load(spacy_model)` (el paquete ya horneado) antes de caer a
  `spacy.cli.download()`.
- `nlp.txt`/`speech.txt` tenían varios pines viejos que dejaron de resolver
  en Python 3.12/aarch64 al hacer un build limpio de `hri-ros` (la imagen
  cacheada que veníamos usando predataba estos requirements y nunca lo
  expuso): `pydantic==1.10.11` en `nlp.txt` forzaba a pip a resolver
  `thinc==9.1.1` (única versión de thinc compatible con pydantic 1.x), que
  no tiene wheel para aarch64/cp312 y falla al compilar Cython desde fuente
  — se quitó el pin (nada en `nlp/` usa la API de pydantic 1.x). En
  `speech.txt`: `onnxruntime==1.16.3` y `scipy==1.10.1` ya no tienen wheel
  para Python 3.12 (bump a `1.17.3`/`1.11.4`); `openwakeword==0.6.0` estaba
  duplicado (el Dockerfile ya lo instala aparte con `--no-deps`, precisamente
  porque su dependencia `tflite-runtime` no tiene wheel aquí) — se quitó del
  requirements; `piper-tts`/`piper` no los importa ningún script y
  `piper-tts` requiere `piper-phonemize`, sin wheel disponible — se
  quitaron; `torchaudio<=2.5.0` (pin viejo, ya no hace falta gracias al
  parche de `df/io.py` de abajo) forzaba una versión de torchaudio
  incompatible con el torch que instalan `nlp.txt`/`postgres.txt`
  (`torch 2.14.0`), el mismo tipo de rotura de ABI CUDA documentada abajo —
  se fijó a `torchaudio==2.11.0` (la versión que efectivamente resuelve
  junto al resto).
- Con `nlp.txt`/`speech.txt`/`postgres.txt` ya instalando limpio, apareció
  un segundo problema, más sutil: `voice_detection.py`, `noise_cancellation.py`
  y `llm_utils.py` morían con `ValueError: numpy.dtype size changed, may
  indicate binary incompatibility. Expected 96 from C header, got 88 from
  PyObject` al importar `scipy.spatial.transform`. `pip show`/`import numpy`
  confirmaban `numpy 2.5.2` y `scipy` correctos — no era un problema de qué
  versión quedaba instalada, sino que **`scipy==1.18.1` (la última en ese
  momento) tiene un bug real de ABI contra `numpy 2.5.2` en este entorno**.
  Se confirmó reinstalando varias versiones de scipy en vivo dentro del
  contenedor corriendo y probando `from scipy.spatial.transform import
  Rotation`: `1.16.2`, `1.15.3`, `1.14.1` y `1.13.1` funcionan, `1.18.1` no.
  Se fijó `scipy==1.16.2` (con `numpy==2.5.2`) de forma consistente en los
  **tres** requirements (`nlp.txt`, `speech.txt`, `postgres.txt`) — cada uno
  es una invocación de pip separada en el Dockerfile, así que un solo
  archivo sin el pin (p. ej. `postgres.txt`, que jala scipy transitivamente
  vía `scikit-learn`←`sentence_transformers`) reintroduce la versión rota.
  Mismo motivo para fijar `pydantic==2.13.5` en `nlp.txt`+`postgres.txt` y
  `sentence_transformers==2.6.1` en `postgres.txt` (sin pin, jalaba una
  versión más nueva que subía `transformers` por encima de lo que fija
  `nlp.txt`, silenciosamente, entre invocaciones de pip separadas).
  `deepfilternet` complica esto más: su metadata exige `numpy<2.0` aunque su
  parte compilada (`deepfilterlib`) es Rust/PyO3, no Cython, y no depende
  realmente del ABI de numpy — pinnearlo junto con `scipy==1.18.1`
  (numpy≥2.0) en el mismo archivo daba `ResolutionImpossible` directo. Se
  instala aparte con `pip install --no-deps deepfilternet==0.5.6
  deepfilterlib==0.5.6` en `Dockerfile.ROS` (mismo patrón que
  `openwakeword`), con sus dependencias reales (`appdirs`, `loguru`)
  agregadas explícitamente a `speech.txt`.
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
- `hri-ros` (`docker/hri/compose/hri-ros.yaml`) tenía el mismo problema de
  audio que `tts.yaml` (socket de PulseAudio clásico en vez del de
  PipeWire-Pulse) — hacía fallar `audio_capturer.py` con
  `PyAudio: Invalid input device`. Se aplicó el mismo fix: montar
  `/run/user/${LOCAL_USER_ID}/pulse` directo y agregar `SDL_AUDIODRIVER:
  pulse` al `x-speech-devices` compartido.
- `frida_interfaces_cache` (compila `frida_interfaces`/`frida_constants`/
  `xarm_msgs` antes de `hri`) puede quedar en un build incompleto (p. ej.
  interrumpido por un reboot) sin que nada lo detecte: `lib.sh` solo
  reconstruye la caché si la carpeta `build/` no existe, no si el build dentro
  de ella falló a medias. Cuando pase, hay que borrar
  `docker/frida_interfaces_cache/{build,install,log}` a mano (puede necesitar
  `sudo rm -rf` si el contenedor corrió como root por `UID`/`GID` sin
  exportar) y volver a correr `./run.sh hri --build`. No se automatizó una
  detección de build incompleto en esta iteración.

## zed — verificado con cámara real (ZED2 por USB)

Con la cámara conectada, se encontraron y arreglaron 4 problemas en cadena
(cada uno tapaba al siguiente):
1. `docker/zed/.env` en la Orin seguía con `BASE_IMAGE`/`IMAGE_NAME`
   apuntando a `jazzy_l4t_base`/`jazzy_zed-l4t` — no se actualizó en el
   rename de imágenes. Corregido.
2. `zed-l4t` estaba construida antes del fix de `NVIDIA_VISIBLE_DEVICES`
   en la base — sin eso, `libcuda.so.1` no se encontraba y el componente
   `zed_camera_component` fallaba al cargar. Reconstruida.
3. Faltaba la regla udev del host para el vendor ID de Stereolabs (`2b03`,
   `/etc/udev/rules.d/99-slabs.rules`) — sin ella, el MCU/sensores de la
   cámara daban `Permissions denied`. Esto es config de host (normalmente
   la crea el instalador del SDK cuando se corre nativo, no dentro de un
   container), así que nunca existió aquí. Creada.
4. `/usr/local/zed/settings` y `/usr/local/zed/resources` en el host
   (montados al container) eran `root:root` sin permiso de escritura — el
   SDK necesita escribir ahí el archivo de calibración de la cámara
   (descargado por serial) y el modelo neural de profundidad optimizado
   con TensorRT (~26MB, se compila la primera vez). `chown 2002:2002`
   (el UID del container) en ambos.

Con los 4 fixes: `=== zed started ===`, positional tracking activo,
publicando RGB/depth/IMU/point cloud reales — confirmado `rgb/color/rect/image`
a ~30Hz vía `ros2 topic hz`.

## Imagen base única `l4t_base` (optimización de tamaño)

Las imágenes l4t de Jazzy pesaban mucho más que en `main` (`l4t_base` 26 GB,
`hri-l4t` 53 GB, `manipulation-l4t` 50 GB). Con `docker history` se vio que
la causa no era el PYTHONPATH, sino tres cosas:

1. **La base traía `cuda-toolkit-13-2` completo más los `-dev`: una capa de
   14.3 GB.** Incluía `libnvinfer_static.a` (3.3 GB), Nsight Systems/Compute
   (1.7 GB) y las librerías estáticas de CUDA (~4 GB). Todas las áreas la
   heredaban.
2. **torch se instalaba desde PyPI (2.14.0+cu130).** El índice de Jetson solo
   llega a 2.11, así que pip elegía el de PyPI por ser más nuevo. Ese wheel trae
   su propio CUDA/cuDNN en paquetes `nvidia-*` (3.3 GB) más `triton`, y duplica
   lo que ya estaba en el sistema. Además, `torchaudio` no coincidía con torch.
3. **`PIP_IGNORE_INSTALLED=1` hacía que cada `pip install` reinstalara todo su
   árbol de dependencias.** En hri-l4t, torch quedaba dos veces (capas de
   9.66 GB y 6 GB). A eso se sumaban OpenCV compilado en vision y en navigation
   sin borrar el árbol de build, e iceoryx/CycloneDDS compilados en 5
   Dockerfiles.

La vieja `dustynv/l4t-pytorch` era ligera porque ya traía torch/OpenCV
compilados contra el CUDA del sistema en una sola capa compartida. Ahora
`docker/Dockerfile.ROS-l4t` hace ese mismo papel, y todas las áreas l4t
heredan de ella como en `main`:

- **CUDA 13.2 / cuDNN 9 / TensorRT 10.** Se instalan compilador, headers y
  librerías compartidas, sin el meta-paquete `cuda-toolkit` (Nsight) y sin
  librerías `*_static*.a`. También se borran los builder resources de TensorRT
  para GPUs dGPU (`sm90`/`sm100`); se conserva `sm110` (Thor).
- **Stack de Python compartido:** `torch 2.11.0`, `torchvision 0.26.0` y
  `torchaudio 2.11.0` (las versiones coinciden) vienen de **PyPI**.
  - Los wheels de `jetson-ai-lab sbsa/cu130` **no sirven en Orin**: solo traen
    kernels para `sm_110` (Thor) y `sm_121` (Spark). En la Orin (`sm_87`) fallan
    con `no kernel image is available for execution on the device`. El build
    cu130 de PyPI trae `sm_80`, que corre en la Orin, y `sm_110`, que sirve para
    Thor.
  - En la metadata de torch se quitan las dependencias `cuda-toolkit`,
    `cuda-bindings`, `nvidia-cudnn-cu13` y `triton`, así que usa CUDA 13.2 y
    cuDNN 9.20 del sistema. Solo se instalan como wheels NCCL, NVSHMEM y
    cuSPARSELt, que JetPack no trae. Sin triton, `torch.compile` no está
    disponible.
  - En la Orin torch muestra un aviso de compute capability (8.7 vs 8.0), pero
    matmul, conv (cuDNN) y `torchvision.ops.nms` funcionan en GPU.
  - `onnxruntime-gpu 1.24.0` sigue viniendo del índice de Jetson (TensorRT EP).
  - Además: `numpy 2.5.2`, `scipy 1.16.2`, OpenCV 4.14 + contrib con CUDA
    (`docker/scripts/build_opencv.sh`, sin árbol de build) y
    `cv_bridge`/`image_geometry` compilados contra ese OpenCV.
- **Builds sin GPU:** `docker build` no tiene el runtime de NVIDIA. Los pasos
  que importan torch/cv2 usan `with-cuda-stub <cmd>`, que agrega el stub
  `libcuda.so.1` solo para ese comando.
- **Paquetes `.deb` placeholder:** iceoryx y cv_bridge compilados desde fuente
  se registran como paquetes apt vacíos (versión 99, en hold) con
  `install-placeholder-deb`. Así apt no queda con dependencias rotas (antes se
  quitaban con `dpkg --force-depends`) y rosdep no vuelve a instalar los de
  apt.
- **iceoryx 2.0.6 (límites ampliados) + CycloneDDS 0.10 con SHM** se compilan
  una sola vez. La memoria compartida queda apagada por defecto; cada área que
  la usa pone `ENV CYCLONE_SHM=1` y vuelve a correr `cyclonedds_setup.sh`.
- **Sin `PIP_BREAK_SYSTEM_PACKAGES` ni `PIP_IGNORE_INSTALLED`:**
  - Se borra `/usr/lib/python3.12/EXTERNALLY-MANAGED`.
  - Los paquetes de Python que apt instala para ROS y que las áreas
    actualizan (numpy, pillow, pyyaml, requests…) se instalan una sola vez en
    `/usr/local` con `--ignore-installed`. A partir de ahí pip los ve primero y
    los puede actualizar normalmente.
- **`/etc/pip.conf`** fija el índice de Jetson, `no-cache-dir` y
  `constraint = /etc/pip/constraints.txt` (`docker/scripts/constraints-l4t.txt`).
  Como está en `pip.conf` y no en `ENV`, también aplica con `sudo pip`. Ningún
  área puede reinstalar ni cambiar la versión de torch/numpy/scipy/OpenCV/
  onnxruntime: si un requirement choca, el build falla en vez de engordar la
  imagen sin avisar.
- **Distribuciones placeholder** (`opencv-python*`, `onnxruntime`): son
  `dist-info` vacíos con la versión de la base. Así `ultralytics`,
  `insightface`, `faster-whisper`, etc. dan la dependencia por satisfecha y no
  instalan encima los wheels CPU de PyPI. `ros-jazzy-cv-bridge` e
  `ros-jazzy-image-geometry` también quedan como paquetes placeholder, para que
  apt/rosdep no instalen encima los de apt.
- **`pip-install-reqs a.txt b.txt`** es un helper de la base para los
  requirements que se comparten con cpu/cuda. Ignora las líneas de paquetes que
  ya da la base (por ejemplo `numpy<2` en `tts.txt` u `onnxruntime==1.17.3` en
  `speech.txt`) y hace un solo `pip install`. Los Dockerfiles compartidos
  (`hri/dockerfiles/Dockerfile.ROS`, `integration`) solo lo usan si existe.

Cambios por área:

- **vision:** ya no compila OpenCV, no instala torch/numpy/onnxruntime por
  pip, no reconstruye cv_bridge y no compila iceoryx/cyclone. dlib se compila
  en un solo paso.
- **navigation:** ya no compila OpenCV ni hace `pip install torch` como
  usuario `ros` en `~/.local`. Por eso se quitan también el `PYTHONPATH` y el
  `LD_LIBRARY_PATH` hacia `~/.local`, y el `pip install --force-reinstall
  "numpy<2"` (ahora numpy 2, igual que el resto). rtabmap y rtabmap_ros quedan
  fijados por commit. OpenVDB se compila en una sola capa y se borran sus
  fuentes.
- **manipulation:** usa torch/numpy de la base. Ya no clona
  `RoBorregos/home2` (traía la rama por defecto); `rosdep` lee los
  `package.xml` de este checkout con un bind mount de BuildKit.
- **hri-ros:** `nlp.txt`, `speech.txt` y `postgres.txt` se instalan en una
  sola corrida del resolver.
- **stt:** CTranslate2 y PyAV quedan fijados por commit, compilados y
  limpiados en un solo paso, e instalados antes de los requirements (así no
  hace falta el `--force-reinstall` final).
- **tts:** usa la base directamente.
- **roudi, zed, display:** usan el iceoryx/cyclone de la base. roudi y display
  siguen compilándolo solo si su base no lo trae (cpu/cuda).

- **moondream-server:** un solo `pip install`; en l4t usa numpy/OpenCV/torch de
  la base.
- **stt:** las librerías FFmpeg que compila PyAV se copian a `/usr/local/lib`,
  así que el `command` de `stt-l4t.yaml` ya no hace
  `source /tmp/PyAV/scripts/activate.sh` (ese directorio ya no existe).
- **navigation:** `LD_LIBRARY_PATH` incluye `torch/lib` de la base, porque
  rtabmap enlaza libtorch.

Resultado medido en la Orin (2026-09-15). La columna "Antes" es `docker images`
del baseline. En "Ahora", el total es `docker images` y el propio sale de
`docker system df -v`: lo que la imagen agrega sobre `l4t_base`, que se guarda
una sola vez en disco.

| Imagen | Antes | Ahora (total) | Ahora (propio) |
|---|---|---|---|
| l4t_base | 26.3 GB | 16.2 GB | — |
| hri-l4t | 53.6 GB | 17.9 GB | 1.7 GB |
| manipulation-l4t | 50.3 GB | 22.2 GB | 6.0 GB (MoveIt + rosdep del repo) |
| moondream-server | 42.6 GB | 16.5 GB | 0.3 GB |
| vision-l4t | 39.9 GB | 16.9 GB | 0.7 GB |
| navigation-l4t | 38.7 GB | 23.1 GB | 7.0 GB (rtabmap, Nav2, OpenVDB) |
| hri-tts-l4t | 36.4 GB | 17.0 GB | 0.9 GB |
| hri-stt-l4t | 30.2 GB | 16.6 GB | 0.5 GB |
| zed-l4t | 28.2 GB | 17.2 GB | 1.1 GB |
| display-l4t | 27.8 GB | 17.0 GB | 0.8 GB |
| roudi-l4t | 26.3 GB | 16.2 GB | 0 |
| integration-l4t | 21.0 GB | 16.6 GB | 0.5 GB |

En disco, las 12 imágenes l4t ocupan ~16 GB compartidos más ~20 GB propios
(~36 GB en total). Antes eran ~190 GB, porque cada una traía su propio torch/CUDA.
Cada imagen pasó un smoke test con GPU real:
- **Todas:** torch matmul/conv en CUDA.
- **vision:** cv2.cuda, TensorRT EP, dlib CUDA, YOLO, cv_bridge.
- **hri:** spaCy, DeepFilterNet, openwakeword, sentence-transformers.
- **stt:** CTranslate2 con float16 en CUDA, PyAV, faster-whisper.
- **navigation:** SuperPoint TorchScript; rtabmap enlazado a OpenCV 4.14 y torch.
- **tts:** kokoro.
- **manipulation:** ultralytics, CLIP.

Para agregar o cambiar una versión de torch/numpy/OpenCV: editar
`docker/scripts/constraints-l4t.txt` (y la URL del wheel en
`Dockerfile.ROS-l4t` si es torch) y reconstruir la base.

## Pendiente / fuera de alcance de esta iteración

- Sabores `cpu`/`cuda` de cada área: no priorizados en esta iteración (foco
  exclusivo en `l4t`, que es el hardware real del robot).

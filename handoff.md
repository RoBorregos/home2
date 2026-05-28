# Handoff — Pick & Place con VAMP en FRIDA xArm6

**Última actualización**: 2026-05-28
**Branch activa**: `tune/vamp-for-grasps` (último commit `cf9497be`)
**Estado**: ⚠️ Pausado. Pick funciona end-to-end con VAMP enabled, pero a través del **OMPL fallback** que MoveIt corre tras rechazar el plan de VAMP. El propio VAMP sigue siendo rechazado por la FCL post-validation — entender ese rechazo es la pieza que falta para que VAMP destape su velocidad real (~3-5× sobre OMPL puro). Pick + place ya operacionales para uso normal.

---

## TL;DR

- **Objetivo**: hacer que picks/places funcionen con el pipeline VAMP de MoveIt, **sin pre-grasp** (la cámara no es paralela al gripper, descarta el patrón `MoveIt PickAction` estándar).
- **Lo que aprendimos**: el problema de fondo es que en un grasp pose el gripper físicamente se traslapa con la representación de colisión del objeto, y MoveIt/FCL rechaza eso como colisión normal. Ningún tuneo de parámetros de VAMP solo lo arregla.
- **Lo que construimos**: pipeline de **filtrado object-aware del pointcloud** (Phases 1-3) + **ACM-allow scoped a las esferas de percepción del target** (Phase 4). Es el patrón estándar de la literatura moderna (MoveIt PickAction usa una variante, también PickNik `deep_grasp_task`, NVIDIA Isaac, etc.).
- **Resultado actual**: pick + place exitosos. Pick ~23 s (vs ~10-15 s con OMPL puro #1018). VAMP planea pero su FCL post-validation interno lo rechaza, OMPL fallback termina destapando — funciona, pero VAMP no está aportando lo que podría.
- **Próximo paso**: investigar por qué `VampPlannerManager::runOmplFallback`'s FCL post-validation no respeta la ACM que aplicamos. Si lo arreglamos, pick baja a ~5-8 s.

---

## 1. Contexto previo (qué heredamos al iniciar el trabajo)

### 1.1 Estado del repo cuando empezamos esta sesión
- `main` con **PR #1018 mergeado** (commit `ac89e35f`): `octomap_resolution: 0.05 → 0.025`, `default_planning_pipeline: vamp → ompl`, `start_vamp_server` default `true → false`. Pick + place funcionan con **OMPL puro** (~10-15 s/pick), VAMP está apagado.
- PR #1017 (también mergeado): fix de `vamp_server.py` que usaba `abspath` en lugar de `realpath` bajo symlink-install — desbloqueó el arranque del backend de VAMP.

### 1.2 La trampa del PR #977 (lo que rompió todo originalmente)
Hace ~5 días, **PR #977 (VAMP motion planning integration)** subió `octomap_resolution: 0.025 → 0.05` (probablemente para acelerar el CAPT pointcloud de VAMP). Con voxeles de 5 cm + padding `0.03/1.05`, cada voxel se infla a ~12-14 cm efectivos. Para objetos de ~5-7 cm (naranja, manzana) los voxeles del propio objeto cubren completamente el grasp pose → FCL rechaza → OMPL falla en samplear el goal → picks rotos para objetos pequeños.

El bisect lo localizó en el commit `6cd26cc3` exactamente, y PR #1018 lo destapó **para OMPL puro**.

### 1.3 Por qué queremos VAMP funcionando
- OMPL puro funciona pero es lento (~10-15 s/pick típico).
- VAMP es vectorizado (SIMD sphere-sphere), capaz de planear en ~20-100 ms cuando todo va bien.
- Lo confirmamos: en moves grandes (return-to-position), VAMP es **~10× más rápido** que OMPL puro.
- Si destapamos VAMP para grasps, picks bajarían a ~5-8 s.

### 1.4 El "deal-breaker" estructural: el gripper en el grasp ESTÁ tocando el objeto
- En cualquier grasp, el gripper físicamente se traslapa con el objeto (es lo que SIGNIFICA agarrar).
- MoveIt/FCL detectan eso como **colisión gripper↔objeto** y rechazan el goal.
- VAMP tiene un `self_filter_distance: 0.12 m` que borra los puntos cercanos al cuerpo del robot del modelo de colisión de VAMP → VAMP "planea bien" pero en realidad ignoró el objeto.
- Cuando MoveIt corre **FCL post-validation** contra el PlanningScene real (con el objeto), rechaza el plan de VAMP.
- Luego OMPL fallback intenta planear pero también ve el goal en colisión, falla con `Unable to sample any valid states for goal tree`.

### 1.5 Por qué descartamos `pre-grasp + cartesian descent`
- Es el patrón **estándar** de MoveIt PickAction.
- Pero la cámara de FRIDA está montada con un ángulo, no paralela al gripper. En un pre-grasp pose la cámara pierde visión del objeto (o el gripper bloquea la vista). El usuario explícitamente lo descartó.
- Esto nos obliga a buscar una variante "single-shot" del agarre.

### 1.6 El intento que falló (lección aprendida)
- Primer enfoque: **ACM-allow gripper vs `<octomap>`** (permitir que el gripper "colisione" con todo el octomap).
- Aparentemente funcionó (FCL ya no rechaza), pero **`<octomap>` es UNA entidad única en MoveIt** — el permiso cubre el octomap entero, incluyendo la mesa.
- En el primer test físico: el dedo del gripper cruzó por la posición de la mesa porque el planner ya no la consideraba obstáculo.
- **El usuario metió e-stop a tiempo, no hubo daño.** Lo revertimos inmediatamente (commit `97609ebf`).
- Lección: ACM-allow solo es seguro si el `object_id` permitido es **una entidad específica** (id concreto con extensión limitada al objeto), nunca `<octomap>`.

---

## 2. Arquitectura propuesta y construida (4 fases)

Patrón estándar de la literatura: **remover el objeto del pointcloud antes de que llegue al octomap monitor**, y representar la presencia de las esferas de percepción del objeto con un **ACM-allow scoped** (solo gripper vs spheres del target).

### 2.1 Fase 1 — Perception publica bbox del target
**Archivo**: `manipulation/packages/perception_3d/src/add_primitives.cpp`
**Commit**: `6d8f4585`

`add_primitives.cpp` ya hacía la segmentación PCL del cluster del target. Le agregamos:
- Publisher nuevo: `target_object_bbox_pub` (tipo `vision_msgs/Detection3DArray`).
- Topic: `/manipulation/target_object_bbox`.
- En `add_pick_primitives()`, **solo en la rama `is_object && !is_other_objects`** (el target real, NO los objetos circundantes), computa `getMinMax3D(*cloud)` y publica una `BoundingBox3D` axis-aligned en `base_link` con margen de 3 cm.
- Los "other_*" objects (clutter) deliberadamente NO publican bbox — su clutter debe quedar visible en el octomap para que el brazo los esquive.

Deps:
- `vision_msgs/Detection3DArray, Detection3D, BoundingBox3D` (de `ros-humble-vision-msgs`).
- Agregado a `CMakeLists.txt` (`find_package(vision_msgs REQUIRED)`, `ament_target_dependencies(pick_primitives ... vision_msgs)`) y `package.xml`.

### 2.2 Fase 2 — Filter node Python + Dockerfile
**Archivos**: `manipulation/packages/perception_3d/scripts/object_aware_pointcloud_filter.py` (nuevo), `manipulation/packages/perception_3d/launch/object_aware_pointcloud_filter.launch.py` (nuevo), `docker/manipulation/Dockerfile.{l4t,cuda,cpu}` (agregado `ros-humble-vision-msgs`).
**Commits**: `d2fb022e` (filter + Dockerfile), `121dca40` (respawn=True para el filter).

El nodo `object_aware_pointcloud_filter`:
- Suscribe `/point_cloud` (output de `downsample_pc`) y `/manipulation/target_object_bbox`.
- Publica `/point_cloud_object_filtered`.
- Latch del último bbox recibido. Si pasa más de `bbox_ttl_sec` (default 30 s) sin actualización, **pass-through** (no filtra). Esto significa: sin pick activo, octomap se construye exactamente igual a como lo hacía antes.
- Durante un pick: bbox latched → cada cloud que llega, transforma los puntos al frame del bbox (TF lookup una vez por cloud), checa AABB membership vectorizado con numpy, dropea los que caen adentro.
- Republica al output topic con QoS RELIABLE/KEEP_LAST(1) — match con `downsample_pc` y `occupancy_map_monitor` de move_group.
- `respawn=True, respawn_delay=2.0` en su launch — si las deps no están al momento de arrancar (caso vimos con `vision_msgs` faltante en contenedor recreado), se levanta solo cuando se instalan.

### 2.3 Fase 3 — Cableado en el launch
**Archivos**: `manipulation/packages/arm_pkg/launch/frida_moveit_config.launch.py`, `manipulation/packages/arm_pkg/config/sensors_3d.yaml`.
**Commit**: `61986e0f`.

- `frida_moveit_config.launch.py`: include del launch del filter, junto al de `downsample_pc`, y agregado al return list.
- `sensors_3d.yaml`: `point_cloud_topic: /point_cloud → /point_cloud_object_filtered`. Ahora `move_group`'s `occupancy_map_monitor` consume la versión filtrada del cloud.

Con esto la cadena queda: ZED → `downsample_pc` → `/point_cloud` → `object_aware_pointcloud_filter` → `/point_cloud_object_filtered` → occupancy_map_monitor → octomap (sin voxeles del objeto durante pick).

### 2.4 Fase 4 — ACM-allow scoped a esferas de percepción
**Archivo**: `manipulation/packages/pick_and_place/pick_and_place/pick_server.py`.
**Commit**: `cf9497be`.

El filter quita los voxeles del octomap, pero `add_primitives` también agrega **esferas de colisión explícitas** (`frida_pick_object_X.X Y.Y Z.Z`) que NO son octomap. Esas esferas siguen bloqueando el gripper en el grasp pose si no se permite la colisión.

Se introdujo:
- `set_acm_pairs(link_ids, object_ids, allowed)` — helper que modifica la Allowed Collision Matrix vía `/get_planning_scene` + `/apply_planning_scene` con un diff. **Surgical y simétrico**: solo flip de las cells `(link_id, object_id)` requeridas; agrega rows/cols con default `False` si no existen.
- En `pick()`, antes del loop de grasps, calcula `target_sphere_ids = [o.id for o in self.collision_objects if PICK_OBJECT_NAMESPACE in o.id]` (extraídos del snapshot que `save_collision_objects` toma justo arriba).
- `try/finally`: en el try, `set_acm_pairs(EEF_CONTACT_LINKS, target_sphere_ids, True)`. En el finally, restore con `False`. Garantiza que la ACM siempre se restaura, incluso en exception o early return.
- **NO incluye `<octomap>`** en `object_ids`. Esa fue la lección del incidente — `<octomap>` es una entidad única, permitirlo libera todo (incluida la mesa). Permitir solo los ids específicos de las esferas del target es seguro porque cada id existe físicamente solo en la zona del objeto.

`EEF_CONTACT_LINKS = ["link_eef", "link_6", "gripper", "left_finger", "right_finger"]` (constante de `frida_constants/manipulation_constants.py`).

---

## 3. Estado actual en la Orin (verificación en vivo)

Verificado en `192.168.31.10` (alias `orin10`, hostname `orin`):

### 3.1 Procesos vivos en `home2-manipulation`
```
move_group                                    (default planner pipeline = "vamp")
down_sample_pc                                (publica /point_cloud, ~10 Hz)
object_aware_pointcloud_filter.py             (publica /point_cloud_object_filtered, ~10 Hz)
pick_primitives                               (publica bbox cuando llega un pick)
vamp_server.py                                (con min_r_point=0.035, self_filter=0.07, range=0.025)
pick_server.py                                (con set_acm_pairs activo)
manipulation_core.py, place_server.py, ...    (resto del pipeline)
```

### 3.2 Topics cableados
```
/point_cloud                        ← downsample_pc (publica)
                                     ← object_aware_pointcloud_filter (suscribe)
/point_cloud_object_filtered        ← object_aware_pointcloud_filter (publica)
                                     ← move_group/occupancy_map_monitor (suscribe)
/manipulation/target_object_bbox    ← pick_primitives (publica esporádico)
                                     ← object_aware_pointcloud_filter (suscribe)
```

### 3.3 Métricas del último pick exitoso
- Filter dropea **~90 puntos / 35000+** (cluster del objeto).
- VAMP planea en 2-7 s (atraviesa internamente 2 attempts cuando el primer attempt encuentra colisión en su modelo de esferas).
- **VAMP plan failed FCL post-validation** persiste — VAMP planea pero la post-validation del `VampPlannerManager` lo rechaza.
- OMPL fallback (interno al `VampPlannerManager`) corre 4 attempts. El primer grasp pose alternative (j=0) suele fallar también; el segundo (j=1) suele encontrar plan.
- Total pick task: **~23 s** desde detección hasta `Pick Task completed successfully`.
- Resultado físico: **agarra el objeto correctamente**, sin colisiones con la mesa.

### 3.4 ⚠️ Cosas no fully resueltas
- **VAMP FCL post-validation sigue rechazando todo plan de VAMP**. Significa que la ACM-allow no está siendo respetada por esa validation. La hipótesis es que `VampPlannerManager` (C++) lee un snapshot del PlanningScene **antes** del apply de nuestra ACM, o no consulta la ACM en su check de path. Es lo más probable por el timing (apply async + plan dispatch en rápida sucesión).
- **OMPL fallback SÍ respeta la ACM** (por eso destapa el segundo grasp alternative). O sea: la ACM funciona, pero el FCL check del plugin de VAMP no la lee.
- **`<octomap>` en realidad NO contiene los voxeles del objeto durante un pick** (porque el filter los quita). Pero la "rechazada FCL" probablemente sigue ocurriendo por otra razón — quizás VAMP planea un path que pasa por donde estaban los voxeles del objeto antes (path-grazing del modelo de esferas vs malla FCL), no por la goal config en sí.
- Esto significa que el problema podría requerir investigación profunda en el C++ del plugin de VAMP (`vamp_planning_context.cpp::runOmplFallback`, su llamada a FCL `isPathValid`/`isStateValid`).

---

## 4. Lo que YA funciona (estado entregable)

- ✅ Pick para naranja/manzana exitoso end-to-end.
- ✅ Place también funciona (no requirió cambios — mismo principio de octomap intacto + ACM scoped via la attached object machinery).
- ✅ Mesa intacta en el octomap → brazo la respeta.
- ✅ Filter pass-through cuando no hay pick → octomap normal en standby.
- ✅ ACM se restaura siempre (try/finally probado).
- ✅ Cutlery branch (force-guarded descent) intacta — el try/finally lo envuelve pero el extra ACM allow es no-op para cutlery.
- ✅ Reproducible vía Dockerfile (vision_msgs en los 3 variants l4t/cuda/cpu).
- ✅ `respawn=True` en el filter — si las deps tardan en estar listas, se levanta solo.

---

## 5. Lo que falla / preguntas abiertas

### 5.1 VAMP-FCL post-validation rechaza cada plan
- **Síntoma**: cada attempt VAMP loguea "SUCCESS | X waypoints" pero move_group inmediatamente warnea "VAMP plan failed FCL post-validation against current PlanningScene".
- **Hipótesis A — Timing**: La ACM se aplica vía `/apply_planning_scene` (async). El plan dispatch a VAMP/move_group quizá ocurre antes que MoveIt internalice el diff. El `set_acm_pairs` espera el future de apply pero quizás MoveIt aplica el diff asincrónicamente en su monitor.
- **Hipótesis B — Path grazing**: VAMP planea un path que entre waypoints intermedios "atraviesa" geometría que la malla FCL detecta. No es problema del goal sino del path. Si esto, no es resuelto por ACM (que es state-level, no path-level).
- **Hipótesis C — Plugin no consulta ACM**: `VampPlannerManager` quizás invoca un check de colisión que ignora la AllowedCollisionMatrix (e.g., usa directamente FCL sin pasar por el `PlanningScene::isPathValid`).
- **Próximo experimento**: leer `vamp_planning_context.cpp::runOmplFallback` y la función que hace post-validation. Identificar exactamente qué check se usa. Si es algo como `planning_scene->isPathValid(...)`, debería respetar ACM — si no, hay un bug en el plugin.

### 5.2 ~23 s por pick (cuando podría ser ~5-8 s)
- Cada attempt VAMP gasta 2-7 s antes de fallar en FCL post-validation.
- Después corre OMPL fallback 4 retries × ~0.5 s c/u = ~2 s.
- Por grasp pose alternativa fallida son ~5-10 s perdidos.
- Si VAMP-FCL aceptara el plan a la primera, picks bajarían a ~5 s.

### 5.3 vision_msgs en contenedor recreado
- El Dockerfile lo tiene ya, pero si alguien usa una imagen pre-built (sin rebuild), el paquete no está.
- Tras un `docker compose up --build`, todo bien.
- Tras un container recreate desde imagen previa, falta y el `pick_primitives` C++ entra en respawn loop. Workaround: `docker exec --user root home2-manipulation apt install -y ros-humble-vision-msgs` y se recupera solo.
- Documentar esto en el README quizá vale la pena.

### 5.4 Filter dropping low % of points
- En logs vemos: `Filtered 90/35000 points` (~0.25%). El bbox del cluster del orange es chiquito (~10×10×8 cm con +3cm margen), y de los ~35k puntos del cloud, solo ~90 caen ahí — el objeto es pequeño relativo a la escena completa (mesa, paredes, etc.).
- **No es un bug** — el porcentaje es esperado para objetos pequeños. La masa del cloud son mesa+paredes+resto, que sigue presente en el octomap. El % bajo no indica falla.

### 5.5 Behavior con múltiples objetos en mesa
- Si hay varios objetos cercanos y perception clusteriza incorrectamente, el bbox podría no cubrir el target real. **Caso no probado en vivo todavía**.

---

## 6. Plan para la siguiente sesión

### 6.1 Prioridad 1 — Entender el rechazo FCL de VAMP (~1-2 h)
1. **Leer** `manipulation/packages/vamp_moveit_plugin/src/vamp_planning_context.cpp`. Focus en:
   - La función que llama al FCL post-validation tras recibir el plan de VAMP.
   - Identificar si llama `PlanningScene::isPathValid(...)` (respeta ACM) o algo más bajo nivel que no la respeta.
2. **Sub-experiment**: agregar logging al plugin para imprimir el ACM state que ve al momento del check (vía `scene->getAllowedCollisionMatrix()`). Confirmar si la ACM-allow está visible o no.
3. **Si la ACM no está**: timing issue → forzar un sleep tras el apply, o usar el future result para confirmar antes del plan dispatch.
4. **Si la ACM sí está pero FCL rechaza**: path-level grazing → la post-validation de VAMP está siendo muy estricta con el path intermedio (no solo el goal). Tres opciones:
   - Saltar la post-validation cuando la ACM tiene allows activos (riesgo: aceptar grazes reales).
   - Densificar el path de VAMP más finamente y forzar el FCL check con menos tolerancia.
   - Bajar VAMP `range` o subir `smoothing_passes` para que el path sea más suave (ya tuneado, no mejoró mucho).

### 6.2 Prioridad 2 — Validar consistencia (~30 min, sin código)
- Probar 5-10 picks consecutivos con la naranja en posiciones ligeramente distintas en la mesa.
- Medir success rate y tiempo promedio.
- Confirmar que el incident de la mesa no se repite.
- Probar también con otros objetos: manzana, cup, etc.

### 6.3 Prioridad 3 — Cleanup y PR (~30-60 min)
- Una vez confiables, abrir PR de `tune/vamp-for-grasps` → `main`.
- Tareas de cleanup:
  - Squash de commits incidentales (varios "iter" intermedios) en commits temáticos limpios:
    - "Add object-aware pointcloud filter pipeline (perception → filter → octomap)"
    - "pick_server: scoped ACM-allow for target perception spheres"
    - "VAMP retuning: min_r_point + self_filter + range" (si se mantiene)
    - "Dockerfile: add ros-humble-vision-msgs"
  - Eliminar code paths comentados.
  - README rápido en `perception_3d/` explicando el flow.
- Considerar: ¿este cambio activa VAMP como default en main? Sí — esto sería el reverso de #1018's `default_planning_pipeline=ompl`. Pero solo es sano una vez que VAMP funcione fast.

### 6.4 Prioridad 4 (opcional) — Robustez perception
- Validar con clutter: agregar varios objetos cerca del target y ver si la bbox del cluster correcto se publica.
- Bbox del cluster usa `getMinMax3D` axis-aligned — si el cluster está orientado (no axis-aligned), la bbox AABB sobreestima. Considerar usar la PCA-rotated bbox (ya disponible en el código para planes).
- Pero: el bbox que publicamos es para FILTRAR voxeles, no para representar el objeto. Sobreestimar levemente (con margen 3 cm) es seguro — solo borra un poco más de octomap. Probablemente no vale la pena.

---

## 7. Cómo retomar mañana

### 7.1 Setup básico
```bash
# Tu laptop: estar en la branch correcta
cd /home/dominguez/roborregos/manip/home2
git checkout tune/vamp-for-grasps
git pull origin tune/vamp-for-grasps   # por si hubo cambios

# Conectarse a la Orin correcta (LA del robot, no la otra)
ssh orin10
# o equivalente: ssh -i ~/.ssh/orin_key orin@192.168.31.10
# Verificar siempre con: hostname → debe decir "orin" (NO "ubuntu")
```

### 7.2 Verificar que el contenedor tenga el código
```bash
# En la Orin:
# Confirmar que vision_msgs está instalado:
docker exec home2-manipulation dpkg -l | grep ros-humble-vision-msgs
# Si NO está, reinstalar:
docker exec --user root home2-manipulation apt-get install -y ros-humble-vision-msgs

# Compilar perception_3d (si hubo cambios C++):
docker exec home2-manipulation bash -lc '
source /opt/ros/humble/setup.bash
source /workspace/frida_interfaces_cache/install/local_setup.bash
source /workspace/install/setup.bash
cd /workspace
colcon build --symlink-install --packages-select perception_3d
'
```

### 7.3 Lanzar el pipeline
```bash
# Dentro del contenedor (o vía run.sh):
cd /home/orin/home2/docker/manipulation && ./run.sh manipulation
# luego dentro:
ros2 launch manipulation_general ppc.launch.py
```

### 7.4 Sanity check post-launch
```bash
# Topics
ros2 topic hz /point_cloud                        # ~10 Hz
ros2 topic hz /point_cloud_object_filtered        # ~10 Hz (idéntico sin pick)
ros2 topic info /manipulation/target_object_bbox  # 1 publisher, 1 subscriber

# Nodos
ros2 node list | grep -E "object_aware|vamp_server|pick_server"
```

### 7.5 Disparar pick
```bash
ros2 run pick_and_place keyboard_input
# opción 2 (orange) o 1 (apple) etc.
```

### 7.6 Buscar logs clave en docker logs
```bash
docker logs home2-manipulation --tail 500 2>&1 | grep -E \
  "VAMP plan failed FCL|Unable to sample|Filtered.*points inside|Target bbox latched|Grasp pose reached|Pick Task completed"
```

### 7.7 ⚠️ Si algo va mal en seguridad
- **E-stop a la mano siempre**.
- Si el brazo se mueve hacia donde no debe → e-stop + matar el launch (Ctrl-C) + reportar.

---

## 8. Archivos modificados en esta branch (`tune/vamp-for-grasps` vs `main`)

Cambios funcionales (excluyendo iters revertidos):

| Archivo | Commit principal | Resumen |
|---|---|---|
| `manipulation/packages/perception_3d/src/add_primitives.cpp` | `6d8f4585` | +50 LOC. Publica `/manipulation/target_object_bbox` con la bbox del cluster del target objeto cuando se ejecuta `add_pick_primitives` para el target (no para los other_*). |
| `manipulation/packages/perception_3d/CMakeLists.txt` | `6d8f4585` | +2 LOC. `find_package(vision_msgs REQUIRED)` y agregado a `ament_target_dependencies` de `pick_primitives`. |
| `manipulation/packages/perception_3d/package.xml` | `6d8f4585` | +1 LOC. `<depend>vision_msgs</depend>`. |
| `manipulation/packages/perception_3d/scripts/object_aware_pointcloud_filter.py` | `d2fb022e` | +265 LOC nuevo. Nodo Python que filtra puntos del pointcloud dentro de la bbox publicada por percepción. Vectorized AABB test con numpy. Pass-through cuando bbox stale o ausente. |
| `manipulation/packages/perception_3d/launch/object_aware_pointcloud_filter.launch.py` | `d2fb022e`, `121dca40` | +54 LOC nuevo. Launch del filter con `respawn=True, respawn_delay=2.0`. |
| `manipulation/packages/perception_3d/CMakeLists.txt` | `d2fb022e` | +1 LOC. Agregado el script al `install(PROGRAMS ...)`. |
| `docker/manipulation/Dockerfile.{l4t,cuda,cpu}` | `d2fb022e` | +1 token c/u. `ros-humble-vision-msgs` en la línea de apt install. |
| `manipulation/packages/arm_pkg/launch/frida_moveit_config.launch.py` | `61986e0f` | +17 LOC. Include de `object_aware_pointcloud_filter.launch.py`, agregado al return list. |
| `manipulation/packages/arm_pkg/config/sensors_3d.yaml` | `61986e0f` | -1 +5 LOC. `point_cloud_topic: /point_cloud → /point_cloud_object_filtered`. |
| `manipulation/packages/pick_and_place/pick_and_place/pick_server.py` | `cf9497be` | +228 LOC, -126 LOC. Imports de `ApplyPlanningScene`, `GetPlanningScene`, `PlanningScene`, `AllowedCollisionEntry`. Clients en `__init__`. Nuevo helper `set_acm_pairs`. `pick()` envuelto en try/finally con ACM-allow scoped a `frida_pick_object_*` ids. |
| `manipulation/packages/vamp_moveit_plugin/launch/vamp_server.launch.py` | `1f1ce052`, `d3410563` | Retuning de VAMP params: `min_r_point: 0.09→0.035`, `self_filter_distance: 0.12→0.07`, `range: 0.05→0.025`, `validation_step_size: 0.05→0.025`, `smoothing_passes: 3→5`. |
| `manipulation/packages/arm_pkg/launch/frida_moveit_config.launch.py` | `1f1ce052` | `default_planning_pipeline: "ompl" → "vamp"`, `start_vamp_server` default `"false" → "true"`. |
| `manipulation/packages/arm_pkg/config/sensors_3d.yaml` | `1f1ce052` | `padding_offset: 0.03→0.0`, `padding_scale: 1.05→1.0`. |
| `manipulation/packages/pick_and_place/pick_and_place/managers/PickManager.py` | `1f1ce052` | Grasps ordenados por GPD score descendente antes de truncar a 5 (antes usaba `np.random.choice` ignorando scores). |

### 8.1 Historia de commits en la branch
```
cf9497be  pick_server: scoped ACM-allow for target perception spheres (safe variant)
121dca40  object_aware filter: respawn on crash
61986e0f  Activate object-aware pointcloud filter in the moveit pipeline
d2fb022e  perception_3d: object-aware pointcloud filter node + vision_msgs in Dockerfiles
6d8f4585  perception_3d: publish target object bbox on /manipulation/target_object_bbox
97609ebf  Revert "ACM-allow gripper-contact links during grasp..." (after table-finger incident)
[62e82672] ACM-allow gripper-contact links during grasp to fix VAMP+FCL/OMPL goal rejection ← REVERTIDO
d3410563  VAMP iter 2: finer RRT range and validation, revert grasp planning_time bump
1f1ce052  Tune VAMP for small-object grasps and improve pick selection/budget
ac89e35f  Restore OMPL pipeline as default and lower octomap_resolution... (#1018)
```

---

## 9. Riesgos y gotchas

### 9.1 Dos Orins en la red — usar SIEMPRE la correcta
- **Orin del robot**: `192.168.31.10`, alias `orin10`, hostname `orin`.
- **Otra Orin**: `100.74.137.47` (Tailscale), hostname `ubuntu`. **NO es la que usamos**.
- Verificar siempre: `ssh orin10 hostname` debe decir `orin`. Si dice `ubuntu`, parar y reconectar.
- Ya se documentó en `~/.claude/projects/.../memory/reference_orin_ssh.md`.

### 9.2 vision_msgs ausente en contenedor recreado
- Si `pick_primitives` o el filter Python no arrancan, primero check:
  ```bash
  docker exec home2-manipulation dpkg -l | grep ros-humble-vision-msgs
  ```
- Si no está, instalar como root:
  ```bash
  docker exec --user root home2-manipulation apt-get install -y ros-humble-vision-msgs
  ```
- Los nodos con respawn se recuperan solos cuando el paquete aparece.

### 9.3 Container recreado pierde estado de install
- `docker compose up --build` para rebuildar desde Dockerfile (incluye vision_msgs ahora).
- Si solo `docker compose up` (sin build), recrea del image previo — quizá sin vision_msgs si la imagen no se rebuildeó.

### 9.4 ACM diff async vs plan dispatch
- `set_acm_pairs` espera el `apply_planning_scene_future` antes de proceder. PERO MoveIt podría tener un PlanningSceneMonitor en otro thread que internaliza el diff con un pequeño delay.
- Si vemos timing issues, considerar dormir 100-200 ms después del apply, o suscribir al `/monitored_planning_scene` y esperar el snapshot que contenga nuestra ACM diff.

### 9.5 bbox publicado pero no consumido
- Si por alguna razón el filter no está corriendo, perception sigue publicando bbox. **Octomap sigue con voxeles del objeto**, picks fallan como antes.
- Indicador: `ros2 topic info /point_cloud_object_filtered` debe mostrar 1 publisher (el filter). Si dice 0 publishers, el filter no está vivo.

### 9.6 TTL del bbox = 30 s
- Si un pick tarda más de 30 s, el filter pasa a pass-through mid-pick — los voxeles del objeto se repueblan, podría fallar.
- Para picks largos, considerar republicar bbox periódicamente desde perception, o subir TTL.

### 9.7 Cutlery branch heredó el try/finally
- El `try/finally` con ACM-allow envuelve TANTO la rama normal como la cutlery.
- Para cutlery (force-guarded descent + clear_octomap interno), la ACM-allow extra es no-op (cutlery no depende de la ACM para su descenso).
- Pero asegurarse de testear cutlery (fork/knife/spoon) en futuras sesiones para confirmar que no introdujimos regresión.

### 9.8 demo_roborregosday.py + submódulo vamp
- Aparecen en `git status --short` como cambios sin commitear (`M task_manager/scripts/misc/demo_roborregosday.py`, `m manipulation/packages/vamp`).
- **Pre-existentes — NO son nuestros**. No incluir en commits/PRs.

---

## 10. Referencias técnicas

- **PR #977** (VAMP integration, el commit que rompió OMPL para grasps pequeños): https://github.com/RoBorregos/home2/pull/977
- **PR #1017** (realpath fix para vamp_server symlink-install): https://github.com/RoBorregos/home2/pull/1017
- **PR #1018** (OMPL default + octomap_resolution 0.025): https://github.com/RoBorregos/home2/pull/1018 (mergeado)
- **VAMP paper**: Thomason et al., "Motions in Microseconds via Vectorized Sampling-Based Planning". Rice University.
- **MoveIt2 PickAction**: https://moveit.picknik.ai/main/api/html/classmoveit__msgs_1_1action_1_1MoveGroupAction.html (referencia del pattern pre-grasp + Cartesian descent).
- **PickNik deep_grasp_task** (variante del pipeline): https://github.com/PickNikRobotics/deep_grasp_task

---

## 11. Quick reference de las decisiones clave

| Decisión | Por qué |
|---|---|
| **Filtrar pointcloud antes del octomap** (no usar ACM `<octomap>` global) | `<octomap>` es una entidad única; permitir collision con él libera la mesa. Filtrar antes de llegar al monitor preserva la mesa intacta. |
| **ACM scoped a `frida_pick_object_*` ids** | Cada id es una sphere específica solo en la ubicación del objeto. Permitir collision con ellas no afecta la mesa ni otros obstáculos. |
| **TTL de 30 s** en el filter | Largo suficiente para que un pick (~5-20 s) corra completo. Corto suficiente para que si un pick crashea sin cleanup, el filter vuelva a pass-through y el octomap se reconstruya con el objeto (recuperar safety). |
| **bbox margen +3 cm** | Permite leve sub-estimación del cluster (perception no es perfecta). Suficiente safety. |
| **respawn=True en el filter** | Sin esto, una dependencia faltante (e.g., vision_msgs no instalado) deja el filter dead y la cadena rota. Con respawn, se levanta solo cuando las deps aparecen. |
| **VAMP retuneado: min_r_point=0.035, self_filter=0.07** | min_r_point ~ octomap_resolution + safety margin. self_filter ~ gripper opening + margin. Antes (0.09 / 0.12) eran nav-scale, inflaban demasiado. |
| **Try/finally para ACM restore** | Garantiza restore en TODO exit path (success, fail, exception). Evita ACM contaminada entre picks. |
| **Pre-grasp + Cartesian descent DESCARTADO** | Cámara no paralela al gripper bloquea la visión durante el descenso. Diseño físico del robot, no del software. |

---

**Fin del handoff. Sesión pausada en este punto — pick funcionando, falta entender el rechazo FCL del VAMP plugin para destapar la velocidad real de VAMP.**

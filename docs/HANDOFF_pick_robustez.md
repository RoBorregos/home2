# HANDOFF — Robustez del pick: revert a main + Fase 0 (post-rotura de finger)

> Fecha: 2026-06-01. Autocontenido: para retomar exactamente donde quedó.
> Relacionado: `docs/pick_robustez_plan.md` (el plan), `docs/grasp_orientation_analysis.md`,
> `docs/HANDOFF_pick_zed_paralela.md` (handoff del experimento ZED-paralela, ahora pausado).

---

## 0. TL;DR

- En un pick de lata, el **descenso cartesiano bajó a ciegas y rompió el right finger**.
- Decisión: **revertir todo a `main`** (ZED de antes + pipeline viejo de plan-libre, que funcionaba la
  mayoría de las veces) y **hacerlo lo más robusto posible** (robustez > rapidez).
- **Fase 0 implementada** (detección de colisión del xArm en 2 capas + approach final lento),
  commiteada, pusheada y **build-verificada**. **NO validada en hardware** — bloqueada por el finger roto.
- El experimento ZED-paralela (#944) NO se pierde: respaldado en `origin`.

---

## 1. Qué pasó (incidente + causa raíz)

Pick de una Coca (cilindro ~17 cm). Secuencia en logs: pre-grasp OK → `Planning CARTESIAN path` →
`Execution completed successfully` → `success=1`. El software creyó que todo salió bien; físicamente el
gripper se clavó y el right finger reventó.

**Causa raíz (verificada):** el `object_aware_pointcloud_filter` quita el objeto target del octomap
(para que su grasp no se contamine). Efecto colateral: **MoveIt no tiene el objeto en colisión durante
el descenso**. Y el descenso normal **no tiene guarda de fuerza** (la guarda solo existe en cutlery, por
la Decisión 3 del plan v2). Resultado: un cartesiano en línea recta hacia un agarre lateral ligeramente
descentrado metió un finger contra la lata rígida (invisible-en-colisión), el brazo siguió empujando los
cm comandados, y rompió. Agravante: la **detección de colisión del propio xArm (`collision_sensitivity`)
estaba SIN setear** → no había red de seguridad de hardware.

(Contexto previo: antes del break ya habíamos visto S1 = grasps inclinados que se caen, y S2 = la muñeca
"voltea" en el descenso; análisis en `docs/grasp_orientation_analysis.md`. El break fue la escalada de S2
sin red.)

---

## 2. Decisión de diseño

Volver a `main` (estado viejo conocido-bueno) y **construir robustez en CAPAS**, porque el "bump" (el
brazo toca y mueve el objeto → el agarre falla) es el problema clásico del **grasping en lazo abierto**:
se percibe una vez y se ejecuta a ciegas, y todo el error (calibración + percepción + control) estalla al
llegar al objeto. No se puede eliminar el error → se construyen capas que toleran/absorben el residuo, con
una red de seguridad por debajo. Detalle y base en literatura: `docs/pick_robustez_plan.md`.

---

## 3. Estado de git (mapa completo)

Paths: **local** = `/home/dominguez/roborregos/home_ws/src`; **Orin** = `ssh orin@192.168.31.10`,
workspace `~/dev/manip` (montado como `/workspace/src` en el contenedor `home2-manipulation`).

**`main` ES el estado viejo pre-#944** (verificado: los commits `048b766e`/`a0320a7f`/`ab92c14f` NO están
en main). Volver a main revierte de verdad la ZED paralela.

| Branch | Dónde | Qué tiene |
|---|---|---|
| `main` | local + Orin (origin al día) | pipeline viejo, ZED de antes. Base del revert. |
| `feat/xarm-collision-safety` | local + Orin + **origin** (`30275fb3a`) | **main + Fase 0** (lo nuevo). El Orin está en esta rama, compilado. |
| `feat/zed-parallel-pick` | local (`96763085`) | F1/A2/B2 locales del experimento (preservado). |
| `feat/944-zed-parallel-pick` | Orin + **origin/home2** | object_aware filter, F1 cartesiano, A2, cherry-picks. Era única → respaldada. |
| `feat/collision-safety` (submódulo) | **origin/xarm_ros2** (`325c69b`) | el edit C++ de Fase 0 (ver §4). |
| `feat/944-zed-parallel-pick` (submódulo) | **origin/xarm_ros2** (`bf806b2`) | SRDF gripper paralelo de #944. Respaldado. |

Notas:
- El Orin **no tiene `user.name/email` de git** → ahí NO se commitea (autor saldría mal); se preserva con
  `git stash` y se commitea desde local. (En el revert, B2/A2 del Orin quedaron en un stash; el canónico
  está en el commit local `96763085`.)
- Submódulos en main: `xarm_ros2`→`05f4e43b` (estable), `vamp`→`2573e11`. Para Fase 0 el gitlink de
  `xarm_ros2` apunta a `325c69b` en la rama `feat/xarm-collision-safety`.

---

## 4. Fase 0 — qué se implementó (detalle)

Rama `feat/xarm-collision-safety` (desde main). Commit superproyecto `30275fb3a`, submódulo `325c69b`.
Autor JLDominguezM, sin atribución de modelo. **Build-OK** en Orin (22 paquetes, 0 fallos, `xarm_controller`
compila). **Sin validar en HW.**

**Fase 0.1 — detección de colisión del xArm, 2 capas (robustez máxima):**
1. **C++ (piso, cada activación):** `xarm_driver_.arm->set_collision_sensitivity(3)` en
   `manipulation/packages/xarm_ros2/xarm_controller/src/hardware/uf_robot_system_hardware.cpp`,
   `on_activate()` justo tras `motion_enable(true)` (~línea 247-253). Se aplica en CADA activación del
   hardware → red garantizada desde antes de que suban los nodos.
2. **Python (valor afinado, tuneable):** método `XArmServices.set_collision_sensitivity(level)`
   (`frida_motion_planning/.../utils/XArmServices.py`) llamado al arranque desde
   `motion_planning_server.py` (rama robot real, tras instanciar `XArmServices`) con
   `COLLISION_SENSITIVITY=3` (en `frida_constants/manipulation_constants.py`, tuneable SIN recompilar).
   Pensado para **re-aplicar también en recovery** (la secuencia clean_error→set_mode→set_state→
   motion_enable podría bajar la sensibilidad).
   - Servicio: `/xarm/set_collision_sensitivity` (`xarm_msgs/srv/SetInt16`, `request.data`), ya habilitado
     en `arm_pkg/config/xarm_params.yaml`.
   - Convención: `0`=off, `1-5`, **mayor = más sensible** (CONFIRMAR empíricamente — el doc del SDK y un
     agente dieron versiones opuestas; no confiar de palabra).

**Fase 0.2 — approach final lento:**
- `PICK_APPROACH_VELOCITY = 0.10` m/s (vs `PICK_VELOCITY = 0.5`) en `manipulation_constants.py`, aplicado al
  `move_to_pose(ee_link_pose, velocity=PICK_APPROACH_VELOCITY)` de la rama no-cutlery en `pick_server.py`.
  Razón: energía de impacto ~v² → da tiempo a la detección de frenar antes de dañar (ISO/TS 15066).

**Archivos tocados (6):** `manipulation_constants.py`, `XArmServices.py`, `motion_planning_server.py`,
`pick_server.py`, `uf_robot_system_hardware.cpp` (submódulo), `docs/pick_robustez_plan.md`.

---

## 5. Plan de robustez por capas (resumen; detalle en `pick_robustez_plan.md`)

Orden por ROI×independencia (cada capa testeable sola; atrapa el residuo de la siguiente):

- **L0 — Red de seguridad (Fase 0, HECHA):** el brazo PARA ante contacto inesperado (`collision_sensitivity`)
  + approach lento. Nunca más romper hardware por un error del planner.
- **L1 — Aproximación controlada (Fase 1):** pre-grasp standoff + tramo recto por el eje de approach +
  **ACM correcta** (objeto = obstáculo en tránsito, contacto permitido SOLO en el tramo final) — en vez de
  quitarlo de colisión. Con fallback a plan libre.
- **L2 — Absorber el residuo (Fase 2):** **fingertips compliant** (goma/espuma, acomodación pasiva Whitney
  1982 — perdona el error de todas las capas) + **guarded move** (mover hasta contacto y parar; generalizar
  el `force_guarded_descent` de cutlery).
- **L3 — Reducir el error de raíz (Fase 3):** verificar/rehacer calibración mano-ojo + de-sesgar la pose
  (ajustar primitiva/cilindro al cluster en vez del centroide crudo, que se sesga a la cara frontal).
- **L4 — Avanzado:** push-grasp (Dogar&Srinivasa 2011), ranking de grasp consciente de colisión.

Refs: Haddadin/De Luca T-RO 2017; De Luca IROS 2006; Mason 1981; Whitney 1982; Dogar&Srinivasa RSS 2011;
Dex-Net (Mahler 2017); Tsai-Lenz 1989; ISO/TS 15066.

---

## 6. Cómo validar Fase 0 cuando se repare el finger (protocolo §3.bis)

Probar la red SIN arriesgar el finger nuevo, por capas:
1. **Fingers fuera (o tapas blandas) primero** — validar que la detección frena antes de confiarle los fingers.
2. **Obstáculo blando y fijo** (espuma), NO el objeto rígido. Movimiento lento (vel. de 0.2) → confirmar parada.
3. **Barrido de sensibilidad** 2→3→4→5: en cada nivel (a) move normal sin contacto → ¿falsos disparos?; (b)
   contacto suave → ¿frena? Elegir el más alto sin falsos. (Confirmar aquí la dirección 0-5.)
4. **Medir distancia de frenado** del TCP tras contacto, en varias poses (la sensibilidad varía con la
   configuración/payload). Si sobrepasa → bajar velocidad o subir sensibilidad.
5. **Recovery:** tras un trip, `clean_error → set_mode → set_state → motion_enable` → vuelve a operar sin
   power-cycle, y la sensibilidad **sigue activa** (valida la capa Python del Frente 1).
6. **E-stop a la mano.** Orden: detección (0.1) → approach lento (0.2) → combinados → recién un pick real.

Tras validar 0: ajustar `COLLISION_SENSITIVITY` al valor elegido y mergear `feat/xarm-collision-safety`.

---

## 7. Pendientes / próximos pasos

1. **[HW, bloqueante] Reparar el right finger.** Bloquea TODA prueba.
2. **Validar Fase 0** con el protocolo §6.
3. **Fase 2.1 (fingertips compliant)** — mecánico, barato, alto ROI (se puede preparar/comprar sin el robot).
4. **Fase 1 (standoff + ACM) + Fase 2.2 (guarded descent generalizado)** — se pueden diseñar sin el robot.
5. **Fase 3 (calibración + de-sesgo de pose)** — si tras lo anterior el empujón persiste / es sistemático.
6. Reconciliar (algún día) el experimento #944 si se retoma la ZED paralela: branches respaldados en origin;
   el hueco a cerrar antes de confiar en ella = guarda de descenso (justo lo de Fase 0/1/2).

---

## 8. Gotchas / operación (aprendidos esta sesión)

- **Build "stale volume":** `./run.sh manipulation --clean` borra `build/log/install` que son **volúmenes**
  montados en el contenedor en ejecución → quedan mountpoints fantasma (link-count 0) y colcon falla con
  `FileNotFoundError: 'log/build_...'`. **Fix: `docker restart home2-manipulation`** (remonta limpio), luego
  `--build`. El `--build` incremental (sin --clean) no tiene este problema.
- **Build:** `cd ~/dev/manip && ./run.sh manipulation --build` (corre colcon en el contenedor,
  `--symlink-install` → los `.py` quedan en vivo, sin rebuild). Para detached + sobrevivir a desconexión:
  `nohup ./run.sh manipulation --build > /tmp/build.log 2>&1 </dev/null &`. Build limpio ~4 min; incremental ~20 s.
- **Orin sin git identity** → no commitear ahí; preservar con stash, commitear desde local.
- **ssh→docker `bash -lc`:** los **paréntesis `()`** rompen el quoting; `pkill` vía `bash -lc` falla silencioso
  → usar `docker exec home2-manipulation pkill -9 -f PATRON` directo.
- **Logs de nodos Python** block-buffered sin TTY → relanzar con `PYTHONUNBUFFERED=1`.
- **Stack:** se levantaba con `ros2 launch manipulation_general ppc.launch.py` (incluye move_group + perception
  + pick + motion_planning). **OJO:** `ppc.launch.py` es del branch #944; en `main` la estructura de launch
  puede diferir — verificar el bringup viejo antes de relanzar en main.

---

## 9. Punteros

- Plan: `docs/pick_robustez_plan.md` (capas, fases, validación §3.bis, refs).
- Análisis de grasps inclinados/descenso: `docs/grasp_orientation_analysis.md`.
- Experimento ZED-paralela (pausado): `docs/HANDOFF_pick_zed_paralela.md`.
- Memoria: `project_pick_robustez_revert.md` (índice en `MEMORY.md`).
- Política de commits: solo JLDominguezM, sin atribución a modelo/AI (ver memoria `feedback_commit_author`).

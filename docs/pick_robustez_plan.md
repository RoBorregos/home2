# Plan de robustez del pick (anti-"bump") — setup ZED viejo

> Objetivo: que el pick deje de **tocar y mover el objeto** (causando que el agarre falle), y que
> nunca vuelva a romperse hardware. Para el setup REVERTIDO (ZED de antes + pick de plan libre).
> Continúa el análisis de causas en este mismo doc §1 y en [[project_pick_generalization_plan]].
> Emitido 2026-06-01 tras romperse el right finger en un pick de lata (descenso a ciegas contra
> objeto filtrado de colisión).

## §1 Diagnóstico (por qué pasa)

Es el **problema clásico del grasping en lazo abierto**: se percibe el objeto **una vez**, se calcula
el grasp y se ejecuta **a ciegas**. Los errores se acumulan y se manifiestan en el peor momento —
cuando el gripper llega al objeto. Causas concretas (ver detalle en la conversación):

- **A — Modelo del mundo mal:** centroide de vista única sesgado hacia la cara frontal (A1), ruido de
  profundidad en objetos lisos/lata (A2), error de calibración mano-ojo = sesgo sistemático (A3),
  latencia/TF (A4).
- **B — Aproximación no controlada:** sin standoff + tramo recto, el plan libre llega de lado y clava
  un finger (B1); contradicción objeto=obstáculo Y objetivo mal resuelta (B2); flip de muñeca barre
  el gripper (B3); ancho finito del gripper se solapa con el objeto (B4).
- **C — Objeto frágil al contacto:** ligero/redondo se expulsa (C1), alto/CoM elevado se vuelca (C2),
  gripper rígido transmite toda la fuerza (C3).
- **D — Sin realimentación:** cámara ciega de cerca (min_depth 0.3) → no visual servoing (D1); sin
  táctil (D2); gripper binario sin control de fuerza (D3).

No se puede eliminar todo el error → la estrategia es **defensa en capas**: reducir el error, controlar
la aproximación, y absorber lo que quede; con una red de seguridad por debajo de todo.

## §2 Filosofía: defensa en capas

| Capa | Qué hace | Ataca |
|---|---|---|
| **L0 — Red de seguridad** | El brazo PARA ante contacto inesperado, pase lo que pase | C3, hardware |
| **L1 — Aproximación controlada y protegida** | Llegar recto por el eje, sin barrer el objeto | B1, B2, B3, B4 |
| **L2 — Absorber el error residual** | Compliance + guarded move: tolerar mm de error | C1, C2, A* residual |
| **L3 — Reducir el error de raíz** | Calibración + de-sesgo de pose | A1, A2, A3 |
| **L4 — Avanzado (si hace falta)** | Push-grasp, ranking consciente de colisión | casos difíciles |

Cada capa es **independiente y testeable**, y protege contra el residuo de la siguiente. Por eso el
orden NO es por "importancia teórica" sino por **ROI (impacto/esfuerzo) + independencia**.

---

## §3 Plan por fases

### Fase 0 — Red de seguridad (PRIMERO, siempre-activa, independiente)

**0.1 `collision_sensitivity` del xArm (2–3) al arranque.**
- *Qué:* el firmware del xArm monitorea corriente/par vs. su modelo dinámico y, ante una fuerza
  inesperada, **para y entra en estado de protección** (estrategia "Stop" de De Luca et al. IROS 2006,
  a nivel controlador, baja latencia).
- *Por qué primero:* es lo que **faltaba** cuando se rompió el finger (estaba sin setear → sin red).
  Protege en TODO movimiento, no solo el pick. Es barato y ortogonal a lo demás.
- *Cómo se cablea (DECISIÓN ROBUSTA, 2 capas — no la vía Python sola):*
  1. **C++ en `on_activate` del driver** (`xarm_ros2/xarm_controller/.../uf_robot_system_hardware.cpp`,
     tras `motion_enable`): `arm->set_collision_sensitivity(N)`. Se aplica en **cada activación** del
     hardware → no se puede olvidar, sobrevive a re-activaciones. (Edita el submódulo + rebuild C++.)
  2. **Re-aplicar en el recovery** (`XArmServices.py`): la secuencia `clean_error → set_mode → set_state
     → motion_enable` puede resetear la sensibilidad; añadir `set_collision_sensitivity(N)` al final del
     recovery garantiza que tras un trip-y-recuperación el brazo **no** se quede sin red.
  Servicio `/xarm/set_collision_sensitivity` (SetInt16) ya habilitado en `arm_pkg/config/xarm_params.yaml`.
- *Por qué las 2 y no solo Python:* robustez > esfuerzo (decisión del usuario). C++ cubre toda activación
  del hardware; el re-apply cubre el recovery runtime. Juntas no dejan ventana sin protección.
- *Valor:* convención UFACTORY `0`=off, `1–5` con **mayor = más sensible** (CONFIRMAR empíricamente, no de
  palabra). Arrancar en `3`; subir al máximo sin falsos disparos (favorecido por el approach lento 0.2).
- *Costo:* M (edit C++ submódulo + rebuild + Python recovery). *Validación:* protocolo de 6 pasos
  (ver §3.bis), fingers-fuera primero, obstáculo blando, medir distancia de frenado, probar recovery.

**0.2 Aproximación final lenta.**
- *Qué:* bajar la velocidad en los últimos cm del approach.
- *Por qué:* energía de impacto ∝ v² (ISO/TS 15066). Con menos velocidad, la detección (0.1) alcanza a
  frenar **antes** de dañar — la detección por corriente tiene umbral y distancia de frenado finita,
  así que la velocidad baja es lo que la hace efectiva en la práctica.
- *Cómo:* parámetro de velocidad del último tramo del approach en `pick_server`. *Costo:* S.
  *Riesgo:* picks más lentos. *Validación:* medir que para sin dañar a velocidad reducida.

### Fase 1 — Aproximación controlada y protegida (núcleo del fix B1/B2)

**1.1 Pre-grasp standoff + tramo recto por el eje de approach, gripper abierto.**
- *Qué:* en vez de plan libre directo al grasp, ir primero a un pre-grasp a N cm "atrás" del grasp a
  lo largo del **eje de approach del gripper**, y luego un tramo **recto** hasta el grasp con el
  gripper **totalmente abierto**; cerrar solo al llegar.
- *Por qué:* con plan libre directo, el planner llega desde una dirección arbitraria y el último tramo
  puede entrar de lado y barrer el objeto (B1, B4). Un tramo recto **colineal con la geometría de
  cierre** hace que los fingers **flanqueen** el objeto en vez de barrerlo. Es la idea correcta de F1
  — la lección es que debe ir **con guarda/protección**, no a ciegas (lo que rompió el finger).
- *Cómo:* re-introducir el pre-grasp+tramo recto en el path normal de `pick_server` (cutlery ya lo
  tiene; el normal no, en el setup viejo). Con **fallback** a plan libre si el pre-grasp es
  inalcanzable (zero-regression). El tramo recto debe correr **bajo la red L0 + guarded move L2**.
- *Costo:* M. *Riesgo:* el standoff puede caer cerca de límites/singularidad → por eso el fallback.
  *Validación:* pick de objeto fijo; confirmar que los fingers flanquean sin empujar.

**1.2 ACM correcta (objeto = obstáculo en tránsito, contacto permitido solo en el grasp).**
- *Qué:* mantener el objeto como **objeto de colisión** durante el tránsito (para no chocarlo), pero
  permitir contacto **gripper↔objeto** SOLO en el tramo final de aproximación/grasp.
- *Por qué:* resuelve la contradicción "objeto = obstáculo Y objetivo" (B2) **sin quitarlo** de la
  colisión (que es lo que el filtro hacía y lo que eliminó la protección → ram). En el setup viejo el
  objeto ya está en el octomap; falta la ACM que abra el contacto fino al final.
- *Cómo:* `motion_planning_server` ya maneja add/remove de objetos y attach con `touch_links`
  (EEF_CONTACT_LINKS). Extender: setear allowed-collision gripper↔objeto antes del tramo final y
  restaurar después. *Costo:* M. *Riesgo:* mal scope de la ACM → o se traba (muy estricto) o no
  protege (muy laxo). *Validación:* confirmar que en tránsito rodea el objeto y que el tramo final no
  se rechaza por colisión con él.

### Fase 2 — Absorber el error residual (compliance + guarded move)

**2.1 Fingertips compliant (goma/espuma).**
- *Qué:* almohadillas blandas en la punta de los fingers.
- *Por qué:* **acomodación mecánica pasiva** (Whitney 1982): la compliance convierte un error de
  posición en contacto en **deslizamiento/auto-centrado** en vez de empujar/expulsar. Es la palanca de
  **mayor ROI** para objetos ligeros/redondos porque perdona el error de TODAS las demás capas, sin
  software. Absorbe mm de desviación de calibración/percepción.
- *Cómo:* mod de hardware (cambiar/forrar las puntas; ajustar STL del gripper si aplica). *Costo:* S–M
  (hardware). *Riesgo:* cambia la geometría/apertura del gripper → re-verificar apertura efectiva y
  recalibrar el offset del TCP. *Validación:* pick de lata/pelota; confirmar que un roce desliza, no
  expulsa.

**2.2 Guarded descent generalizado (move-until-contact → stop → cerrar).**
- *Qué:* en el tramo final, mover **despacio por velocidad** monitoreando el contacto (effort/colisión)
  y **parar** en el primer toque, luego cerrar.
- *Por qué:* es el **"guarded move"** clásico (Mason 1981; Will & Grossman): no comandes un
  desplazamiento ciego de N cm — muévete hasta el contacto y detente. Convierte el "bump que expulsa"
  en "contacto que dispara el agarre". Es tu sustituto práctico del lazo cerrado sin sensores táctiles.
- *Cómo:* generalizar el `force_guarded_descent` de `pick_server` (hoy solo cutlery: xArm modo 5
  velocidad cartesiana + umbral de effort) al path normal, con umbral sano. Reabre la Decisión 3 del
  plan v2 — que a la luz del finger roto conviene reconsiderar. *Costo:* M. *Riesgo:* el effort es par
  articular (no FT real) → ruidoso → tunear umbral; mode-switch da spikes transitorios (ya hay grace
  period en cutlery). *Validación:* pick de objeto blando/ligero sin expulsarlo.

### Fase 3 — Reducir el error de raíz (percepción + calibración)

**3.1 Verificar/rehacer calibración mano-ojo (si el sesgo es sistemático = A3).**
- *Qué:* validar la transformada cámara→base (ZED extrínseca) del montaje viejo.
- *Por qué:* si el objeto se corre **siempre en la misma dirección**, no es ruido — es sesgo de
  calibración (Tsai-Lenz 1989). Atacarlo elimina el error en la fuente, no solo lo tolera.
- *Cómo:* procedimiento de calibración mano-ojo (similar a la "Calibración z gripper-touch" de #944,
  para el montaje viejo). *Costo:* M. *Riesgo:* fiddly. *Validación:* medir offset percibido vs. real
  del centro del objeto en varias poses.

**3.2 De-sesgar la pose del objeto (A1/A2).**
- *Qué:* en vez del **centroide crudo** del cluster (sesgado a la cara frontal), **ajustar una
  primitiva** (cilindro para lata, caja) y usar su **eje/centro**; o **fusionar multi-vista**.
- *Por qué:* una nube de vista única solo ve el frente → el centroide se jala hacia la cámara → el
  grasp queda corrido → roza el lado lejano. Una primitiva ajustada recupera el centro real aunque la
  parte de atrás no se vea.
- *Cómo:* en el clustering/PickManager (perception_3d), fit de primitiva o fusión de 2–3 vistas
  moviendo el brazo. *Costo:* M–L. *Riesgo:* añade complejidad de percepción. *Validación:* offset
  centro-estimado vs. real menor que con centroide crudo.

### Fase 4 — Avanzado (solo si L0–L3 no bastan)

- **4.1 Push-grasp** (Dogar & Srinivasa, RSS 2011): para objetos ligeros contra un respaldo, empujar el
  objeto **hacia** la mano → "embudona" la incertidumbre a un estado agarrado. Convierte el contacto
  en mecanismo, no en falla.
- **4.2 Ranking de grasp consciente de colisión** (Dex-Net, Mahler 2017): preferir grasps de **mayor
  margen** (apertura − ancho objeto) y **corredor de aproximación libre** para el ancho del finger;
  descartar los que clavarían. Más robustez a la incertidumbre de pose.
- **4.3 Lazo cerrado** (visual/tactile servoing) — bloqueado por hardware (cámara ciega de cerca,
  gripper binario). Diferido salvo cambio de sensores.

---

## §3.bis Protocolo de validación seguro (Fase 0)

Probar la red **sin** arriesgar el finger nuevo, por capas (post-incidente):
1. **Fingers fuera (o tapas blandas) primero** — validar que la detección frena el brazo antes de
   confiarle proteger los fingers. Si falla, que no haya finger que romper.
2. **Obstáculo blando y fijo** (espuma), **no** el objeto rígido. Movimiento lento (vel. de 0.2) → confirmar parada.
3. **Barrido de sensibilidad** 2→3→4→5, en cada nivel: (a) move normal sin contacto → ¿falsos?; (b) contacto suave → ¿frena? Elegir el más alto sin falsos.
4. **Medir distancia de frenado** del TCP tras contacto; si sobrepasa → bajar vel. o subir sensibilidad. Probar en varias poses (la sensibilidad varía con la configuración/payload).
5. **Recovery:** tras trip, `clean_error → set_mode → set_state → motion_enable` → vuelve a operar sin power-cycle, y la sensibilidad **sigue activa** (valida la capa 2 del Frente 1).
6. **E-stop a la mano.** Orden: detección (0.1) → approach lento (0.2) → combinados → recién un pick real.

## §4 Orden de implementación (robusto — hacerlo bien una vez)

Decisión del usuario (2026-06-01): **máxima robustez sobre rapidez**; hacer las fases completas, no parches.
ROI decreciente, cada fase testeable sola y como capa independiente:

1. **Fase 0** (0.1 collision_sensitivity 2 capas + 0.2 approach lento dedicado) — la red que faltaba. **Primero.**
2. **Fase 2.1** (fingertips compliant) — mecánico; perdona el error de todas las capas a la vez.
3. **Fase 1** (1.1 standoff+recto + 1.2 ACM) + **Fase 2.2** (guarded descent) — aproximación controlada y protegida.
4. **Fase 3** (calibración mano-ojo + de-sesgo de pose por primitiva) — fijar el error de raíz, no solo tolerarlo.

Se implementa en rama `feat/xarm-collision-safety` (desde main), verificando build en cada paso. Sin merge a
main hasta validar con hardware (finger reparado). Nada se prueba hasta reparar el finger.

**Primer paso concreto:** Fase 0.1 — `collision_sensitivity` en `on_activate` (C++) + re-apply en recovery
(Python) + Fase 0.2 (velocidad de approach lenta dedicada). Robot a salvo aunque el pick falle.

## §5 Referencias

- Haddadin, De Luca, Albu-Schäffer, *Robot Collisions: A Survey on Detection, Isolation, and
  Identification*, IEEE T-RO 2017 (pipeline detección→reacción).
- De Luca, Albu-Schäffer, Haddadin, Hirzinger, *Collision Detection and Safe Reaction with the DLR-III
  Lightweight Manipulator Arm*, IROS 2006 (estrategias de reacción).
- Mason, *Compliance and Force Control for Computer Controlled Manipulators*, IEEE T-SMC 1981; Will &
  Grossman 1975 (guarded moves).
- Whitney, *Quasi-Static Assembly of Compliantly Supported Rigid Parts*, 1982; Hogan, *Impedance
  Control*, 1985 (compliance / acomodación pasiva).
- Dogar & Srinivasa, *A Framework for Push-Grasping in Clutter*, RSS 2011 (manipulación bajo
  incertidumbre).
- Mahler et al., *Dex-Net 2.0*, RSS 2017 (robustez del grasp a incertidumbre de pose).
- Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration*,
  IEEE T-RA 1989.
- ISO/TS 15066:2016 (límites de fuerza/velocidad de contacto).

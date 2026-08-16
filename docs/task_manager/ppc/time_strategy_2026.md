# PPC 2026 — Estrategia de tiempo y maximización de puntos

> Plan escrito (NO implementado). Análisis 2026-06-30. Verificado contra el rulebook oficial
> "RoboCup@Home Rules / Final version for RoboCup 2026 (Revision 2026-06-19)", §5.2 (pág. 40-44).
> Task: `task_manager/scripts/pickandplace_task_manager.py`.

## 0. Resumen ejecutivo

- **Límite = 420 s (7:00). Muro duro pero los puntos ganados se quedan.** Hoy el run tarda ~14 min (happy path)
  → se queda sin tiempo **en breakfast**, justo donde están los puntos más densos.
- En 420 s solo caben **~3-5 acciones de manipulación**. **La selección de acciones lo es todo.**
- **El FSM nunca mira el reloj.** Corre cleanup-completo (`max_cleanup_objects=3`) y luego breakfast. Resultado:
  los **2 pours (400 pts)** casi nunca se ejecutan.
- **Decisión central: priorizar/proteger breakfast (sobre todo los pours) por encima del cleanup.**

### Capacidades confirmadas del robot (2026-06-30, definen el routing)
| Primitiva | ¿Confiable? | Consecuencia para la estrategia |
|---|---|---|
| Pour cereal/leche | ✅ (si agarra bien del cabinet) | Cluster de 400 pts. Proteger el shelf-pick de cereal/leche, es la ruta crítica. |
| `place_on_shelf` (cabinet) | ✅ (lento, 60-120s) | "Other" objects → cabinet (+60 c/u) valen la pena si hay reloj. |
| Colocar DENTRO del dishwasher | ❌ | **Fuera este año.** No perseguir +70/obj. cutlery/mug: banca solo el *pick*. |
| Flat-grasp del plato | ❌ | **Fuera este año.** Plato sigue skippeado (no apostar el pick de 150). |

## 0.bis Datos CONFIRMADOS de la sede (repo RoboCupAtHome/Incheon2026)

**Reality check de puntajes** (scores.md del repo, referencia de evento comparable con top teams):
**el mejor PP fue 170** (Tobi), luego 145/135/135/135/85/85/35/35/15/15, y muchos 0. Nadie se acerca a los 3515.
→ **~135-200 es puntaje GANADOR.** La estrategia óptima es **asegurar primero la base barata y confiable**
(navigate + recognize de TODOS los objetos + first-pick + 1-2 picks), y **luego** ir por breakfast/pours como
stretch. La base confiable sola ya es competitiva; un solo pour (+200) probablemente gana el reto.

**3 runs (1 por día), solo cuenta el mejor** → hacer un run SEGURO (banca ~200) y en otro día el run AGRESIVO
(ir por los pours). Bajo riesgo para intentar el stretch.

**Correcciones de config OBLIGATORIAS para Incheon:**
1. **TRASH = FRUITS** (mapa P&P: "Fruits will be the trash for this task"). El código tiene `trash_category="drink"`
   (`:131`). **Cambiar a `"fruit"`.** El item de trash en la mesa es una fruta → kitchen trash bin (loc 15).
   `trash_exceptions=["milk"]` ya no aplica (milk es drink, no fruit), pero milk NUNCA debe ir a trash.
2. **Labels REALES del modelo YOLO** (no los nombres físicos de Incheon). El modelo detecta:
   `apple, blue_cereal_box, brown_cereal_box, chocomilk_box, coca_cola, coca_cola_zero, green_elgae_pack,
   laundry_basket, lemmon, mangosteen, peach, pepsi_cola, pringles, red_bell_pepper, red_ramen_pack, redbull,
   rubik_cube, sponge, toothpaste_blue_box, trash_bin, yellow_bell_pepper`.
   - **APLICADO**: `cereal→blue_cereal_box` (¿o brown? confirmar), `milk→chocomilk_box` en `yolo_names`;
     `objects.json`/`objects.md` reescritos a estos labels (fruits→fruit para trash).
   - **Detección COMBINADA (multi-modelo)**: `object_detector_node` corre varios modelos a la vez
     (param `models` en `ModelRegistry`). Set combinado real (2 modelos):
     - Viejo: apple, baseball, blue_cereal_box, bowl, coca_cola, cup, flan_box, grapes, knife, laundry_basket,
       lime, mango, mundred_can, orange, pear, pringles, red_plate, soup_can, spam_can, spoon, sprite_can,
       tennis_ball, trash_bin, valero, video_game_control, yogurt.
     - Nuevo (RCWUP2026): apple, blue_cereal_box, brown_cereal_box, chocomilk_box, coca_cola, coca_cola_zero,
       green_elgae_pack, laundry_basket, lemmon, mangosteen, peach, pepsi_cola, pringles, red_bell_pepper,
       red_ramen_pack, redbull, rubik_cube, sponge, toothpaste_blue_box, trash_bin, yellow_bell_pepper.
     `objects.json`/`objects.md` reescritos con la UNIÓN (39 labels categorizados; `laundry_basket`/`trash_bin`
     fuera por ser contenedor/ubicación). Dishes vienen del modelo viejo → breakfast bowl/spoon y cleanup
     cutlery/plate/cup son perceptibles.
   - **2 caveats del set combinado**:
     - El plato se detecta como **`red_plate`** (no `plate`). El FSM ya lo maneja (`skip_names`, tableware).
     - **NO existe label `fork`** en ningún modelo. Si el cubierto de la mesa es un tenedor, NO se detecta
       (cuchillo/cuchara sí). El spoon del breakfast sí se detecta.
   - Pendiente: confirmar el default del param `models` (el default `["yolo_v26_finetuned"]` no existe en
     MODEL_CONFIGS → se sobreescribe al lanzar) y cuál cereal (blue vs brown).
3. **No existe "side_table" en la arena.** Ubicaciones de cocina (LocationsNames): cabinet(9, pared izq),
   dinner table(16, centro), counter(11), sink(12), cooking table(13), dishwasher(14), kitchen trash bin(15).
   - Cutlery/plate/cup → **dishwasher(14)**. Aunque no entren al rack (+70 ❌), colocar ENCIMA probablemente
     banca la base 40 (designated furniture). **Confirmar con TC.** Routear a dishwasher(14), no a side_table.
   - "other" objects + milk + cereal → **cabinet(9)** (place_on_shelf funciona ✅).
   - trash (fruta) → **kitchen trash bin(15)**.
4. **Fuentes de breakfast** (mapa P&P): **dishwasher tab** (amarillo) y **bowl+spoon** (verde) en lugares
   DISTINTOS de la cocina (zona cooking table/counter, pared inferior). milk+cereal **dentro del cabinet(9)**.
   Mapear waypoints a estas ubicaciones en el slot de Arena Mapping (Miér, 10 min/arena).
5. **3 arenas casi iguales** (mesa/fridge ligeramente movidos); mapear cada día. Dining table ±50 cm.
6. **Red: LAN only, sin WiFi** en día de competencia; internet "siempre roto". `categorize_objects` (HRI/embeddings)
   DEBE correr offline (ya hay commit "Fix no internet hric #1092" — verificar que cubre PPC).
7. **Start: detectar puerta abierta autónomamente.** Usar el botón de start alternativo cuesta **−100**
   (scoresheet, "Using alternative start signal"). Verificar `check_door()`.
8. **dishwasher_tab** está en la task (cleaning_supply) pero no lo manipulamos → **skip** (0 pts, 0 penalización).

**Calendario:** Arena Mapping Miér; **PP corre Jue/Vie 11:00-12:30 y Sáb 13:30-15:00** (3 slots).

## 1. Scoring oficial (verificado) y lo realmente alcanzable

Total posible = **3515 pts**. Mesa = **6 objetos**: 1 cutlery, 1 plate, 1 mug/cup, 1 trash (categoría
anunciada en setup-days), 2 other. Extra surface = 2 common (−20 pick / −20 place c/u). Breakfast: bowl+spoon
(superficie de cocina), milk+cereal (DENTRO del cabinet; **milk cerrada, cereal abierto**).

| Acción | Pts | Alcanzable por nosotros |
|---|---|---|
| Navigate to table | 15 (×1) | ✅ gratis |
| Recognize object | 10 (×12, máx 120) | ✅ casi gratis al percibir/anunciar |
| Perceive shelf + indicar repisa | 30 (×2, máx 60) | ✅ en SCAN_CABINET_SHELVES |
| Pick base | 50 (×12) | ✅ |
| **First-pick bonus** | +100 (1 vez) | ✅ **garantizar en un pick confiable** |
| Cutlery pick | +50 (×2) | ✅ (pick en mesa) |
| Plate pick | +100 | ❌ (flat-grasp no confiable) |
| Dishwasher-tab pick / slot | +100 / +160 | ❌ (no se manipula el tab) |
| From floor | +30 | ⚠️ opcional |
| Place base | 40 (×12) | ✅ donde haya destino designado |
| Dishwasher place | +70 (×3) | ❌ (no se coloca dentro) |
| **Cabinet next-to-similar** | +20 (×2) | ✅ (place_on_shelf funciona) |
| **Pour cereal / milk** | +200 / +200 | ✅ **máxima prioridad** (spill = −100 c/u) |
| Open milk / dishwasher door / rack | 400 / 400 / 200 | ❌ → **pedir ayuda (0 pts, 0 penalización)** |

**Puntos por objeto, con NUESTRAS capacidades:**
- Breakfast: bowl ~90 · cereal ~290 (50+200+40) · milk ~290 · spoon ~90-140 → **~760 pts**.
- Cleanup: first-pick +100 · "other"→cabinet 110 c/u · trash 90 · cutlery pick 100 (place≈0) · mug pick 50 (place≈0).

→ **Breakfast (~760) supera con creces a limpiar la mesa.** El plan optimiza para que breakfast SÍ ocurra.

## 2. A dónde se va el tiempo (time sinks rankeados)

| Sink | Ubicación | Costo | Nota |
|---|---|---|---|
| `place_on_shelf` (cleanup→cabinet) | `manipulation_tasks.py:571`; FSM `:1177`+`:1208-1244` | 60-120s c/u, ×3 intentos ×3 repisas | Cap 180s. ⚠️ riesgo inactividad-30s. |
| Navegación agregada | `nav_tasks` move_to_location/dock | ~465s (~15 tramos×31s) | Sola excede 420s. |
| Cabinet pick cereal/leche (`_pick_from_shelf`) | FSM `:477-534` | ~45s c/u | Re-escanea TODOS los niveles aunque SCAN_CABINET_SHELVES ya lo hizo. |
| Tramo a KITCHEN al inicio | FSM `:664` | ~23s | Inútil (PERCEIVE va a DINING justo después). |
| `timeout(3.0)` octomap-settle | FSM `:413,463,491,527` | ~3s c/u, ~30-50s total | Hard-coded. |
| Re-anuncio de TODAS las detecciones por breakfast pick | FSM `:1315` | ~12s/pick | No suma recognize (ya contado). |
| Doble percepción en `close_to` place | `PlaceManager.py:214-256` y `:316-366` | ~2-4s/place | Bug: corre perception+heatmap 2 veces. |

**Causa raíz estructural:** no hay chequeo de reloj. `total_start_time` (`:238`) solo se usa en el reporte
final (`:1439`). Además está fijado en `__init__`, ANTES de door-open (`:641-658`) → cualquier deadline basado
en él cuenta de más la espera de puerta.

## 3. Plan de cambios (por prioridad)

### TIER 0 — Cambio estructural (mayor impacto, bajo riesgo)

**0.1 Re-anclar el reloj en door-open** y añadir presupuesto de tiempo.

```python
# Near top, junto a ATTEMPT_LIMIT (~l.37)
CLEANUP_DEADLINE_S = 180.0   # tras esto, abandona cleanup y va a breakfast
BREAKFAST_TAIL_S   = 360.0   # tras esto, recorta cola de breakfast (spoon, 2º pour)
HARD_DEADLINE_S    = 405.0   # tras esto, ve a END (deja el brazo en nav_pose)
```

```python
# En WAIT_FOR_BUTTON, JUSTO después de "Door open..." y antes de pasar a START (~l.657)
self.total_start_time = datetime.now()   # re-ancla el cronómetro al inicio real del test
```

```python
# Helper nuevo
def _elapsed(self) -> float:
    return (datetime.now() - self.total_start_time).total_seconds()
```

**0.2 Guard en CLEANUP_LOOP** (`~l.812`, dentro del bloque CLEANUP_LOOP, antes de elegir objeto):

```python
if self._elapsed() > CLEANUP_DEADLINE_S:
    CLog.fsm(self, "STATE", f"Cleanup deadline ({CLEANUP_DEADLINE_S}s) hit at {self._elapsed():.0f}s "
                            "→ jumping to breakfast.", level="warn")
    self.current_state = PickAndPlaceTM.TaskStates.START_BREAKFAST_PREP
    return
```

**0.3 Guard de cola en GET_BREAKFAST_ITEMS** (`~l.1256`): cuando `_elapsed() > BREAKFAST_TAIL_S`, saltar
ítems de bajo valor que aún no se han hecho (spoon; y el 2º pour si milk va a requerir ayuda lenta), priorizando
que al menos cereal-pour quede hecho.

```python
near_wall = self._elapsed() > BREAKFAST_TAIL_S
for item in self.breakfast_items:
    if item["picked"]:
        continue
    if near_wall and item["name"] in ("spoon",):   # cola de bajo valor
        item["picked"] = True   # marca como hecho para saltarlo
        continue
    self.current_breakfast_item = item
    break
```

**0.4 Guard duro global** (al inicio de `run()`, o en cada estado de navegación): si `_elapsed() > HARD_DEADLINE_S`
y no estamos ya terminando, ir a END para dejar el brazo seguro en `nav_pose`.

> Nota: estos guards SOLO saltan trabajo; no agregan riesgo. Son la corrección de mayor impacto.

### TIER 1 — Tiempo gratis (0 pts en riesgo)

**1.1 Borrar el tramo a KITCHEN** en START (`:664`). Ir directo a la mesa; hacer la petición de "remover sillas"
en el approach a `dining_table`. Ahorro ~23s.

**1.2 Recortar `timeout(5.0)`→`2.0`** en START (`:668`). El `say(wait=True)` ya bloquea. Ahorro ~3s.

**1.3 En PICK_BREAKFAST_ITEM (`:1314-1316`)**: anunciar SOLO el ítem objetivo, no todas las detecciones de la
superficie. Ahorro ~12s/pick, 0 pts perdidos (recognize ya se contó).

**1.4 Dedup en `PlaceManager` `close_to`** (`PlaceManager.py:214-256` vs `:316-366`): reutilizar `result_pose`
del primer bloque y saltar la 2ª percepción+heatmap. Ahorro ~2-4s por place de breakfast (todos usan close_to).
⚠️ verificar que `result_pose.header.frame_id` quede seteado tras el 1er bloque.

**1.5 Usar el layout cacheado en `_pick_from_shelf`** (`:487-534`): si SCAN_CABINET_SHELVES ya encontró el nivel
del cereal/leche, ir directo a ese nivel (1 detección de confirmación) en vez de re-escanear los 3 niveles.
Ahorro ~6-15s por shelf-pick. ⚠️ mantener 1 detect de confirmación por si el objeto se movió.

### TIER 2 — Routing correcto al scoresheet (con nuestras capacidades)

**2.1 NO routear al dishwasher interior** (no es confiable). Para cutlery/mug: el *pick* se banca en la mesa
(cutlery = 100, mug = 50); el place de esos a `side_table` da **0 pts** → no gastar un ciclo nav+place caro en
ellos salvo que sobre reloj y solo para liberar el gripper sin tirar (−40 si se cae).
- ⚠️ **Confirmar con el TC en setup-days**: si colocar ENCIMA del dishwasher cuenta como "designated location"
  (base 40), entonces vale la pena dejar cutlery/mug encima del dishwasher (40 c/u) aunque no entren al rack.

**2.2 `determine_placement_location`** (`:567-576`): hoy manda cutlery/tableware a `side_table`. Cambiar a la
política decidida en 2.1 (probablemente "encima del dishwasher si el TC lo acepta, si no banca solo el pick").

**2.3 Plato: sigue skippeado** (`:761`). No tocar (flat-grasp no confiable). No apostar el pick de 150.

**2.4 `trash_category`** (`:131`): NO hardcodear "drink". Convertir en parámetro a fijar en setup-days
(ver checklist §5). Mal-clasificar = 0 pts de place para ese objeto.

### TIER 3 — Tuning de primitivas (NO días antes de competir)

PICK_VELOCITY 0.5→0.8 · `planning_attempts` 5→2 · `timeout(3.0)`→1.5 · VAMP en picks · `place_on_shelf`
cap a 1 intento. EV total ~20-40s pero cada uno puede regresar un grasp y quemar un ciclo completo, o causar
colisión (que puede ANULAR el test). **Solo con un pase de regresión en el robot.** Lo único de aquí que sí
recomiendo pronto: **capar el fallback de `place_on_shelf` de cleanup** (3×3) a 1 intento × 1 repisa fallback
(`:1208-1244`) para defusar el costo de 60-180s y el **riesgo de inactividad-30s** (que SACA al robot).

### NO TOCAR
- `use_vision_confirmation=True`: es la única red anti air-grab (`use_grasp_detector=False`). Quitarla deja que
  un grab vacío gaste un ciclo nav+place entero por 0 pts. Si acaso, evaluar activar el grasp detector.

## 4. "Race plan" recomendado para 420 s

Orden greedy por densidad de puntos, capability-aware:

1. **Nav mesa + percibe + anuncia** → +15 navigate, +~60 recognize. (~75s)
2. **1 pick confiable de mesa ("other"/box) → +100 first-pick + 50.** Place en cabinet (+60, place_on_shelf
   funciona). Banca el bonus en lo más seguro. (~120s)  *(Alternativa si confían en el grasp del bowl: hacer el
   bowl el primer pick para que el +100 cabalgue en breakfast y ahorrar un ciclo de cleanup.)*
3. **Breakfast (prioridad máxima):** bowl→place (40) · cereal: shelf-pick + **pour 200** + place 40 · milk:
   shelf-pick + (pedir ayuda para abrir, 0/0) + **pour 200** + place 40. (~210s)
4. **Si sobra reloj:** 2º "other"→cabinet, o trash→bin.

Objetivo: **~700-850 pts alcanzables** vs ~370-490 del flujo actual que se queda sin tiempo en cleanup.

> El deadline guard (Tier 0) es lo que GARANTIZA que el paso 3 ocurra. Sin él, el robot se atora en cleanup.

## 5. Checklist de Setup-Days (cosas a fijar/confirmar)
- [ ] **Categoría de trash anunciada** → fijar `trash_category` (y `trash_exceptions` si aplica). Confirmar que
      "milk" no caiga en trash.
- [ ] **¿Colocar ENCIMA del dishwasher cuenta como designated (base 40)?** Pregunta al TC. Decide routing 2.1.
- [ ] Alturas reales de las repisas del cabinet → recalibrar `shelf_level_heights` (`:147`).
- [ ] Validar grasp de **bowl** (¿puede ser el first-pick?) y de **cereal/leche en el cabinet** (gate de los pours).
- [ ] Confirmar que el **pour vierte cantidad significativa sin spill** en cereal y en leche (spill = −100).
- [ ] Layout de breakfast: spoon junto a bowl, cereal junto a milk, **≥5 cm de holgura** (si no, −50 y −30 c/u).
- [ ] Posición real del dining table (puede moverse ±50 cm) y del extra surface (±5 cm).
- [ ] Decidir `CLEANUP_DEADLINE_S` / `BREAKFAST_TAIL_S` con cronometrajes reales de los logs (END timing report).

## 6.bis Mapa de roles PPC → waypoints raw de la cocina (LocationsNames)

**Plan acordado:** Nav graba los muebles de la cocina con sus **nombres literales de `LocationsNames.png`**;
NOSOTROS mapeamos los roles PPC a esos nombres en `nav_locations` (ya **APLICADO** en el código).

| Rol PPC (`Location`) | Waypoint raw (areas.json) | Mueble | Dock | Uso |
|---|---|---|---|---|
| `DINING_TABLE` | `dinner_table` | dinner table (16) | dock_table | limpiar + poner breakfast |
| `EXTRA_SURFACE` | `counter` | counter (11) | dock_table | 2 common objects (−20/obj) |
| `DISHWASHER_TAB` | `cooking_table` | cooking table (13) | dock_table | fuente del tab |
| `CABINET` | `cabinet` | cabinet (9) | offset 0.30 | place "other"; fuente milk/cereal |
| `TRASH_BIN` | `trash` | kitchen trash bin (15) | place is_trash | fruta (trash) |
| `DISHWASHER` | `dishwasher` | dishwasher (14) | dock_table | **place cutlery/plate/cup ADENTRO** |
| `BREAKFAST_ITEMS` | `dishwasher` | dishwasher (14), ARRIBA | dock_table | **bowl+spoon pick por ARRIBA** |
| `KITCHEN` | `safe_place` | pose segura | — | nav |

**Confirmado (diseño final):**
- **bowl + spoon → pick desde ARRIBA del dishwasher** (pick normal, sin cambiar lógica: navegar → approach → pick).
- **cutlery / plate / cup → place DENTRO del dishwasher** (TODO: abrir puerta [ayuda = 0 pts/0 penalización] +
  bajar al rack; gateado por `use_dishwasher`).
  - **TEMP para pruebas (`place_dishwasher_at_tab=True`)**: como aún no hay primitiva de "place adentro",
    estos objetos se mandan a la zona del **dishwasher tab (`cooking_table`)** con place normal por arriba.
    Apagar el flag cuando exista el place dentro del dishwasher.
- ⚠️ Confirmar con Nav los **strings exactos** de los waypoints (snake_case de LocationsNames).
- refrigerator (10) y sink (12) NO se usan en PPC.

## 6.ter Lo que Nav debe grabar (Arena Mapping, Miér, 10 min/arena × 3)
Pedir a Nav: bajo room `kitchen`, grabar estos waypoints con los nombres de LocationsNames:
`dinner_table`, `counter`, `cooking_table`, `cabinet`, `dishwasher`, `trash`, `safe_place`
(YA mapeados en `nav_locations`; confirmados contra el mapa de Nav 2026-06-30).
(La mesa puede moverse ±50 cm → re-grabar cada día. `dishwasher` sirve DOBLE: pick arriba + place adentro.)

## 6. Riesgos / pendientes
- **Inactividad 30s**: `place_on_shelf` puede dejar el brazo quieto >30s planeando → te SACAN. Capar intentos
  y/o pre-armar la extensión de inactividad (pedible 1 vez, ≤3 min).
- Pedir ayuda para abrir puerta de dishwasher / leche es **0 pts pero 0 penalización** → pedir sin miedo.
- Los guards de tiempo necesitan cronometrajes reales para calibrar los umbrales (corre el END timing report).
```
```

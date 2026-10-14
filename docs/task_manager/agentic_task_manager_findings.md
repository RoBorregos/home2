# Agentic task manager: hallazgos del prototipo y A/B offline

> Fecha: 2026-10-10 · Estado: prototipo, **no probado en el robot**.
> Código: `task_manager/task_manager/agentic/`, `task_manager/scripts/agentic_task_manager.py`,
> specs en `task_manager/config/agentic/`, tests y benchmark en `task_manager/scripts/test/`.

## 1. Pregunta

¿Conviene reemplazar las FSM escritas a mano (una por prueba) por un único task manager
"inteligente" en el que un LLM (el Qwen que ya corre en la Orin de HRI) decide qué hacer,
reintenta o se salta pasos según el tiempo, y habla con más contexto y personalidad?

## 2. Qué se construyó

Arquitectura **híbrida**: la estructura de la prueba es determinista y el LLM solo
interviene cuando algo falla.

```
spec YAML (pasos del rulebook, puntos, tiempo, fallbacks)
        │
TaskExecutor ── llama skills ──► ros_skills (SubtaskManager)  |  sim_skills (offline)
        │  si una skill falla
        ▼
Supervisor:  rules  | rules+ | llm (+ guardrails, si falla vuelve a rules)
        └─► {action: retry|fallback|skip|abort, say, reason}
```

- **Un solo nodo** (`agentic_task_manager.py`, params `spec`, `policy`, `base_url`, `model`)
  sirve para cualquier prueba: agregar una prueba es escribir un YAML y, si hace falta, una skill.
- **Políticas** (intercambiables con un parámetro → esto es lo que permite el A/B):
  - `rules`: igual que las FSM de hoy (reintentar hasta `max_attempts` → deus ex machina → seguir).
  - `rules+`: `rules`, pero va directo al fallback si el error coincide con un patrón
    permanente declarado en el spec (`permanent_errors`).
  - `llm`: pregunta al servidor OpenAI-compatible (llama.cpp/ollama) con un contexto compacto
    (paso, error, intentos, tiempo restante, puntos en juego, últimos eventos, reglas). Salida
    forzada con JSON schema → gramática en llama.cpp.
- **Guardrails del LLM** (cualquier violación → decide `rules` y se cuenta como inválida):
  acción fuera del enum, fallback sin fallbacks, *retry* más allá de `max_attempts`
  (el LLM solo puede **recortar** reintentos), skip/abort en pasos `skippable: false`,
  timeout o error de red.
- La latencia del LLM **cuenta como tiempo de la prueba** (en el sim se suma al reloj).
- Cada corrida en el robot se guarda como JSON (`results_dir`) para comparar A/B reales.

## 3. Cómo se probó (offline, desktop sin Docker ni GPU)

- llama.cpp `b11540` build **CPU-only** (no toca la GPU), mismos flags que producción
  (`--jinja --reasoning off`, ctx 2048) — ver `docker/hri/scripts/llama-cpp-entrypoint.sh`.
- Modelos: **Qwen3.5-4B UD-Q4_K_XL** (el de producción, alias `frida-llm`) y Qwen3-1.7B Q4_K_M.
- Spec `restaurant_lite.yaml` (tomar orden → bar → por cada item: pick → entregar → place),
  420 s de límite. Puntos aproximados, no los oficiales.
- `sim_skills.py`: cada episodio (seed) sortea qué items son fáciles, resbalosos
  (falla transitoria), inalcanzables o ausentes (fallas permanentes), además de fallas de
  navegación/STT/place. **Common random numbers**: el intento *k* de cada skill tiene el mismo
  resultado en todas las políticas, así que solo difieren por sus decisiones.
- 20 episodios por política, mismas seeds. Latencia cobrada al reloj simulado fija en **3 s por
  decisión** (estimado para la Orin, no medido) para comparar decisiones y no hardware; la
  latencia real en CPU se reporta aparte.

## 4. Resultados

### 4.1 Puntaje (restaurant_lite, mismas seeds)

| Política | Seeds | Puntos medios | % del máximo | Tiempo medio | Timeouts | Decisiones inválidas* |
|---|---|---|---|---|---|---|
| `rules` (FSM actual) | 0–19 | **507.5** | 89.7 % | 352 s | 3 | – |
| `rules+` (reglas + patrones de error) | 0–19 | **517.5** | 91.3 % | 325 s | 1 | – |
| `llm` Qwen3-1.7B | 0–19 | 500.0 | 88.4 % | 366 s | 5 | 18 / 95 |
| `llm` Qwen3.5-4B, prompt v1 | 0–19 | 448.8 | 79.5 % | 327 s | 1 | 24 / 69 |
| `llm` Qwen3.5-4B, prompt v2 | 0–9 | 460.0 | 82.8 % | 343 s | 2 | 16 / 42 |

Sobre las seeds 0–9 (las de la v2): rules 490 · rules+ 505 · 1.7B 480 · 4B v1 427.5 · 4B v2 460.

\* Respuestas que violaron un guardrail; en esos casos decidió `rules`. Sin guardrails los
LLM habrían salido peor. En el 1.7B casi todas fueron *retry* más allá de `max_attempts`
(14 de 18). En el 4B **todas** fueron intentos de saltarse una navegación (`go_bar`,
`back_to_bar`, `deliver`).

**El 4B (el modelo de producción) no superó a `rules` en ninguna de las 20 seeds** (empató en 4).
`rules+` no perdió en ninguna.

### 4.2 Qué decide cada uno (decisiones por tipo de error)

| Error | rules | rules+ | 1.7B | 4B v1 | 4B v2 |
|---|---|---|---|---|---|
| permanente / pick (fuera de alcance, no detectado) | 37 retry, 18 fallback | 19 fallback | 29 retry, 16 fallback, 1 skip | 13 fallback, **6 skip** | **11 fallback** |
| transitorio / navegación | 27 retry, 1 skip | igual | 24 retry | 23 retry, **5 skip** | 15 retry, 4 skip |
| transitorio / pick (resbaló) | 8 retry, 1 fallback | igual | 8 retry, 1 fallback | **6 fallback**, 1 skip | 4 retry |
| transitorio / place | 11 retry | 13 retry | 9 retry, 2 fallback | 7 fallback, 3 skip | 4 fallback, 1 retry |
| transitorio / take_order | 5 retry | 5 retry | 5 retry | **4 fallback** | 3 fallback |

Lectura:
- **Qwen3-1.7B** casi no razona: reintenta errores permanentes aunque la regla del spec dice
  que no se arreglan, y tiene más timeouts.
- **Qwen3.5-4B** entiende muy bien los errores permanentes (con el prompt v2, 11 de 11 bien),
  pero es **impaciente**: trata errores transitorios como permanentes ("Permanent slip error,
  skip to save time") y salta o pide ayuda humana aunque sobre tiempo. Cada fallback cuesta la
  mitad de los puntos y cada skip los pierde todos. Intentó saltarse la navegación de vuelta al
  bar llevando un objeto (lo bloqueó `skippable: false`).
- El prompt importa mucho (v1 → v2: +32 pts en seeds 0–9), pero ni con conocimiento explícito
  de qué errores son transitorios alcanzó a `rules`.
- **Casi todo lo que el LLM hace bien lo consigue `rules+` con 2 líneas de YAML**
  (`permanent_errors`), sin latencia y de forma determinista.

### 4.3 Diálogo y personalidad (donde el LLM sí aporta)

- 4B: frases naturales, con contexto y humor ligero; **0 de 71 frases mencionaron nombres
  internos**. Ejemplos: *"Looks like that item is hiding behind its friends; I'll ask for a
  little help to get it."*, *"Looks like someone is blocking the path, let's try a different route!"*
- 1.7B: repetitivo ("Hmm, … Let me try again!") y filtra nombres internos
  (*"let's try deus_pick"*) en 4 de 77 frases.
- Hablar cuesta tiempo de la prueba: el 1.7B habló en casi cada falla, y eso suma a sus timeouts.
  Conviene limitar las frases del LLM a momentos con valor (pedir ayuda, explicar una demora).

### 4.4 Latencia y contexto

| Modelo (CPU desktop, sin GPU) | p50 | p95 |
|---|---|---|
| Qwen3-1.7B (8 hilos, compartidos) | 5.0 s | 6.8 s |
| Qwen3.5-4B (7 hilos, compartidos) | 11.2 s | 14.7 s |
| Qwen3.5-4B (12 hilos) | 7.7 s | 13.0 s |

- Prompt de ~330–650 tokens por decisión (cabe en el ctx 2048 de producción; llama.cpp reusa
  el prefijo del system prompt entre llamadas). Salida ~40–50 tokens.
- En CPU: prefill ~53 tok/s y generación ~5.4 tok/s. **En la Orin no se midió**; el benchmark
  cobró 3 s por decisión como supuesto. Hay que medirlo con `docker/hri/compose/bench-l4t.yaml`
  y con STT + command interpreter cargados al mismo tiempo.
- La salida con JSON schema (gramática) funcionó: **0 respuestas no parseables** en todas las corridas.

## 5. Conclusiones y recomendación

1. **No reemplazar las FSM por un LLM que decida el flujo.** Con los modelos que caben en la
   Orin, el LLM decide peor que las reglas actuales en recuperación de fallas y además agrega
   latencia y no-determinismo. Un "agente que hace todo" sería peor todavía, porque aquí el LLM
   solo decidía en las fallas.
2. **Sí vale la pena unificar los task managers en "spec + executor + skills"**, aunque sea sin LLM:
   - un solo nodo y un YAML por prueba (pasos, puntos, tiempo y fallbacks salen del rulebook);
   - la política `rules+` ya mejora a las FSM de hoy (+10 pts, −27 s, menos timeouts) solo
     declarando qué errores son permanentes;
   - log estructurado de cada corrida → permite A/B reales en el robot.
   Requisito: que las skills devuelvan errores descriptivos (hoy muchas devuelven solo `Status`).
3. **Usar el LLM donde sí gana: el diálogo.** Frases contextuales cuando el robot pide ayuda,
   explica una demora o se recupera de un error, con personalidad, sin decidir el flujo. Es barato,
   no bloquea (puede ir en paralelo a la acción) y mejora la interacción con jueces y operadores.
4. Si se quiere seguir investigando el LLM como supervisor: modo **"asesor"** (solo puede proponer
   `fallback` temprano ante errores no vistos en `permanent_errors`, todo lo demás lo deciden las
   reglas), few-shot con ejemplos de transitorio/permanente, y medir en la Orin.

## 6. ¿Vale la pena el A/B en el robot?

Sí, pero con estas comparaciones:
- **`rules` vs `rules+`** en una prueba piloto (restaurant o pick & place): es el cambio con más
  evidencia a favor y riesgo casi nulo.
- **LLM solo para diálogo** vs frases fijas (encuesta rápida a quien haga de operador o juez).
- El LLM como supervisor, solo si se cambia el enfoque (punto 4) y después de que gane en el sim.

Cómo: `ros2 run task_manager agentic_task_manager.py --ros-args -p spec:=<prueba> -p policy:=<rules|rules+|llm>`;
cada corrida se guarda en `results_dir` (JSON). Al menos 5 corridas por política, mismo
escenario y alternando políticas.

## 7. Limitaciones

- El simulador es un modelo burdo: probabilidades, tiempos y puntos inventados (no del score
  sheet oficial). Las diferencias pequeñas (±10 pts) están dentro del ruido. Solo la de 4B contra
  rules es consistente seed a seed.
- 20 episodios (10 en la v2), un solo spec (`restaurant_lite`).
- Latencias medidas en CPU con carga compartida; no representan la Orin.
- `ros_skills.py` y `agentic_task_manager.py` **no se ejecutaron** (no hay ROS 2 en el desktop).
  El fallback `ask_to_type_order` en ROS es provisional (todavía no hay entrada de texto en el display).
- `restaurant_lite` simplifica la prueba real (no tiene búsqueda del cliente ni escaneo de
  mesas). Para el robot, `safety.yaml` es el spec más directo de probar.

## 8. Archivos

| Archivo | Qué es |
|---|---|
| `task_manager/task_manager/agentic/spec.py` | Spec YAML (pasos, loops `foreach`, puntos, fallbacks, `skippable`, `permanent_errors`) |
| `task_manager/task_manager/agentic/executor.py` | Ejecuta el spec, presupuesto de tiempo, consulta al supervisor |
| `task_manager/task_manager/agentic/supervisor.py` | `rules`, `rules+`, `llm` (+ guardrails, prompt) |
| `task_manager/task_manager/agentic/sim_skills.py` | Mundo simulado con fallas (common random numbers) |
| `task_manager/task_manager/agentic/ros_skills.py` | Skills sobre `SubtaskManager` |
| `task_manager/scripts/agentic_task_manager.py` | Nodo ROS genérico |
| `task_manager/config/agentic/*.yaml` | Specs `restaurant_lite`, `safety` |
| `task_manager/scripts/test/test_agentic_executor.py` | 19 tests (pytest, sin ROS ni LLM) |
| `task_manager/scripts/test/agentic_ab_benchmark.py` | A/B offline (`--policies rules,rules+,llm`) |

Reproducir: `python task_manager/scripts/test/agentic_ab_benchmark.py --episodes 20 --policies rules,rules+,llm --base-url http://<host>:11434/v1 --model frida-llm --latency-charge 3`.

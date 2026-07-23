"""Self-diagnosis and supervised self-healing for the task manager.

Modules:
    health_gate   — Fase 5: guards each subtask on the health of its critical nodes.
    log_context   — Fase 2 bridge: pulls status.log_collector/log_parser if available.
    knowledge_base— Fase 3: local RAG over docs/ai/.
    oracle        — Fase 3: standalone Ollama diagnosis → {razon_falla, accion_sugerida}.
    actions       — Fase 4: closed catalog of corrective commands.
    orchestrator  — Fase 4: supervised diagnose→propose→(confirm)→execute→re-verify.

All modules degrade gracefully: if the status/ engine, rclpy, or Ollama are not
reachable, the gate becomes a no-op rather than breaking the FSM."""

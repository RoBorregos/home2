"""Fase 3: standalone diagnosis oracle.

Given a parsed error (Fase 2) and a health result (Fase 1), the oracle produces:
    {"razon_falla": <str>, "accion_sugerida": <closed-catalog action>}

It calls a *local* Ollama model with its own OpenAI-compatible client — it does
NOT go through the HRI nlp service, because HRI may be exactly what failed. If
Ollama is unreachable or returns malformed output, a deterministic rule-based
fallback maps the error kind to a sensible corrective action, so the oracle
always returns something actionable."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass

from task_manager.diagnosis import actions
from task_manager.diagnosis.knowledge_base import KnowledgeBase

DEFAULT_BASE_URL = os.getenv("DIAGNOSIS_OLLAMA_URL", "http://localhost:11434/v1")
DEFAULT_MODEL = os.getenv("DIAGNOSIS_MODEL", "qwen3")

# error kind → default corrective action (used by the rule-based fallback)
_FALLBACK_BY_KIND = {
    "segfault": "restart_container",
    "missing_node": "restart_container",
    "dds": "restart_dds",
    "tf2": "restart_container",
    "build": "rebuild_package",
}


@dataclass
class Diagnosis:
    razon_falla: str
    accion_sugerida: str
    source: str = "llm"  # "llm" | "fallback"
    context_used: str = ""


def _build_prompt(error, health, context: str) -> str:
    missing = ", ".join(getattr(health, "missing_nodes", []) or []) or "(none)"
    kind = getattr(error, "kind", "unknown")
    area = getattr(error, "area", "") or "(unknown)"
    line = getattr(error, "raw_line", "")
    actions_list = ", ".join(actions.VALID_ACTIONS)
    return f"""You are FRIDA's fault-diagnosis oracle for a ROS 2 robot.

FAULT:
  kind: {kind}
  area: {area}
  missing_nodes: {missing}
  log_line: {line}

RELEVANT ARCHITECTURE CONTEXT:
{context or "(none available)"}

Choose exactly one corrective action from this closed set:
  {actions_list}
Actions that need an argument use "kind:arg", e.g. "restart_container:navigation"
or "rebuild_package:task_manager". Prefer the action whose area matches the fault.

Respond with ONLY a JSON object, no prose, with exactly these keys:
  "razon_falla": short explanation in Spanish of the root cause
  "accion_sugerida": one action string from the closed set
"""


def _extract_json(text: str) -> dict | None:
    # strip <think>…</think> that some local models (qwen3) emit
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
    match = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except ValueError:
        return None


class DiagnosisOracle:
    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        kb: KnowledgeBase | None = None,
    ):
        self.base_url = base_url
        self.model = model
        self.kb = kb

    def _client(self):
        from openai import OpenAI

        return OpenAI(api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=self.base_url)

    def _retrieve_context(self, error) -> str:
        if self.kb is None:
            return ""
        try:
            query = f"{getattr(error, 'kind', '')} {getattr(error, 'raw_line', '')}"
            chunks = self.kb.retrieve(query, k=2)
            return "\n---\n".join(f"[{c.source}] {c.text}" for c in chunks)
        except Exception:
            return ""

    def _fallback(self, error, health) -> Diagnosis:
        kind = getattr(error, "kind", "unknown")
        area = getattr(error, "area", "") or ""
        action_kind = _FALLBACK_BY_KIND.get(kind, "none")
        if action_kind == "restart_container":
            action = f"restart_container:{area}" if area else "none"
        elif action_kind == "rebuild_package":
            # best-effort: rebuild the area's package if we can name it, else none
            action = f"rebuild_package:{area}" if area else "none"
        else:
            action = action_kind
        reason = (
            f"Fallo tipo '{kind}'"
            + (f" en área '{area}'" if area else "")
            + (
                f"; nodos ausentes: {', '.join(health.missing_nodes)}"
                if getattr(health, "missing_nodes", None)
                else ""
            )
        )
        return Diagnosis(razon_falla=reason, accion_sugerida=action, source="fallback")

    def diagnose(self, error, health) -> Diagnosis:
        context = self._retrieve_context(error)
        prompt = _build_prompt(error, health, context)
        try:
            resp = self._client().chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2,
            )
            content = resp.choices[0].message.content or ""
            data = _extract_json(content)
        except Exception:
            data = None

        if not data or "accion_sugerida" not in data:
            return self._fallback(error, health)

        accion = str(data.get("accion_sugerida", "")).strip()
        # Validate against the closed catalog; unknown → fallback.
        if actions.resolve(accion) is None and actions.parse_action(accion)[0] != "none":
            return self._fallback(error, health)

        return Diagnosis(
            razon_falla=str(data.get("razon_falla", "")).strip() or "(sin razón)",
            accion_sugerida=accion or "none",
            source="llm",
            context_used=context,
        )

"""ROS 2 adapters: the same ports, answered by services on the robot.

The only file that imports rclpy. Calls block the calling thread until the server
answers or the timeout passes, and other threads complete them, so the node must
spin in a MultiThreadedExecutor and never call these from its only free thread.
"""

from __future__ import annotations

import json
import time
from typing import Any
from weakref import WeakKeyDictionary

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from frida_interfaces.srv import EmbedText, LLMWrapper, MapAreas
from ports import LLMUnavailable

SERVICE_TIMEOUT_S = 5.0
LLM_TIMEOUT_S = 30.0
POLL_S = 0.01

# Clients are created once per node and service: each one holds DDS resources.
_clients: WeakKeyDictionary = WeakKeyDictionary()


def _call(
    node: Node,
    srv_type: Any,
    service: str,
    request: Any,
    timeout_s: float = SERVICE_TIMEOUT_S,
) -> Any:
    """Call a service, raising TimeoutError if it is missing or does not answer in time."""
    by_service = _clients.setdefault(node, {})
    if service not in by_service:
        by_service[service] = node.create_client(
            srv_type, service, callback_group=ReentrantCallbackGroup()
        )
    client = by_service[service]
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
        raise TimeoutError(f"{service} is not available")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            client.remove_pending_request(future)
            raise TimeoutError(f"{service} did not answer in {timeout_s} s")
        time.sleep(POLL_S)
    return future.result()


class RosClipEncoder:
    """CLIP text embeddings from the vision service."""

    def __init__(self, node: Node, service: str = "/vision/embed_text") -> None:
        self.node = node
        self.service = service
        self._dim = 0

    @property
    def dim(self) -> int:
        if not self._dim:
            self._dim = len(self.encode_text("object"))
        return self._dim

    def encode_text(self, text: str) -> np.ndarray:
        """Encode a string, L2-normalized."""
        response = _call(
            self.node, EmbedText, self.service, EmbedText.Request(texts=[text])
        )
        if not response.success:
            raise RuntimeError(f"{self.service} failed to embed {text!r}")
        vector = np.asarray(response.embeddings, np.float32)
        return vector / np.linalg.norm(vector)


class RosLLM:
    """Structured completion through the HRI LLM wrapper, which only returns text."""

    def __init__(
        self,
        node: Node,
        service: str = "/hri/nlp/llm",
        timeout_s: float = LLM_TIMEOUT_S,
    ) -> None:
        self.node = node
        self.service = service
        self.timeout_s = timeout_s

    def complete_json(self, prompt: str, schema: dict[str, Any]) -> dict[str, Any]:
        """The model's answer as a dict, or LLMUnavailable if there is none to use."""
        # An example object, not the schema: small models copy a schema's shape back.
        example = {key: "" for key in schema.get("properties", {})}
        question = (
            f"{prompt}\n\nAnswer with only one JSON object with exactly these keys, "
            f"filled in, and nothing else. Example: {json.dumps(example)}"
        )
        try:
            answer = _call(
                self.node,
                LLMWrapper,
                self.service,
                LLMWrapper.Request(question=question),
                self.timeout_s,
            ).answer
            parsed = json.loads(answer[answer.index("{") : answer.rindex("}") + 1])
        except (TimeoutError, ValueError) as error:
            raise LLMUnavailable(str(error)) from error
        if not isinstance(parsed, dict):
            raise LLMUnavailable(f"expected a JSON object, got {answer!r}")
        return parsed


class RosClock:
    """The node clock, so simulated time is respected."""

    def __init__(self, node: Node) -> None:
        self.node = node

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9


def load_areas(node: Node, service: str = "/navigation/areas_json") -> dict[str, Any]:
    """The active map's areas.json, to pass to SemanticGraph.load_layout."""
    return json.loads(_call(node, MapAreas, service, MapAreas.Request()).areas)

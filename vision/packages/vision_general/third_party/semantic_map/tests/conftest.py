from pathlib import Path

import pytest

from adapters.clock import ManualClock
from adapters.files import load_snapshot
from core.config import SemanticMapConfig
from core.graph import SemanticGraph

# Not imported from main.py: it pulls in CLIP and torch, and these tests need neither.
MOCK_SNAPSHOT = Path(__file__).parent.parent / "data" / "mock_snapshot.json"


@pytest.fixture
def config():
    return SemanticMapConfig()


@pytest.fixture
def snapshot(config):
    return load_snapshot(MOCK_SNAPSHOT, config)


@pytest.fixture
def clock(snapshot):
    return ManualClock(snapshot.t_capture)


@pytest.fixture
def graph(snapshot, config, clock):
    """The mock graph with no encoder and no LLM."""
    return SemanticGraph.from_snapshot(snapshot, config=config, clock=clock)

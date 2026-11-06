from dataclasses import asdict

import numpy as np
import pytest

from adapters.files import EMBEDDINGS_NAME, load_snapshot, save_snapshot
from core.graph import SemanticGraph
from core.serialize import snapshot_from_dict, snapshot_to_dict
from core.types import Observation, Point


def _objects(snapshot):
    """Objects as comparable dicts; crops are resolved from disk on load, so left out."""
    out = {}
    for object_id, obj in snapshot.objects.items():
        fields = asdict(obj)
        embedding = fields.pop("embedding")
        fields.pop("crop")
        fields["embedding"] = None if embedding is None else embedding.tolist()
        out[object_id] = fields
    return out


def _assert_same(a, b):
    assert a.rooms == b.rooms
    assert _objects(a) == _objects(b)
    assert (a.t_capture, a.map_name, a.embedding_dim) == (
        b.t_capture,
        b.map_name,
        b.embedding_dim,
    )


@pytest.fixture
def embedded(snapshot):
    """The mock snapshot with a known unit vector on every object."""
    rng = np.random.default_rng(0)
    for obj in snapshot.objects.values():
        vector = rng.standard_normal(snapshot.embedding_dim).astype(np.float32)
        obj.embedding = vector / np.linalg.norm(vector)
    return snapshot


def test_dict_round_trip_is_lossless(embedded, config):
    _assert_same(snapshot_from_dict(snapshot_to_dict(embedded), config), embedded)


def test_every_surface_keeps_its_type_without_guessing(snapshot, config):
    data = snapshot_to_dict(snapshot)
    surfaces = sum(len(room.surfaces) for room in snapshot.rooms.values())
    assert len(data["furniture_meta"]) == surfaces


def test_file_round_trip_keeps_embeddings_out_of_the_json(embedded, config, tmp_path):
    path = tmp_path / "snapshot.json"
    save_snapshot(embedded, path)
    assert '"embedding": null' in path.read_text()
    _assert_same(load_snapshot(path, config), embedded)


def test_saving_without_embeddings_clears_old_ones(embedded, config, tmp_path):
    path = tmp_path / "snapshot.json"
    save_snapshot(embedded, path)
    for obj in embedded.objects.values():
        obj.embedding = None
    save_snapshot(embedded, path)
    with np.load(tmp_path / EMBEDDINGS_NAME) as vectors:
        assert not vectors.files
    assert all(
        o.embedding is None for o in load_snapshot(path, config).objects.values()
    )


def test_graph_state_survives_a_save_and_reload(graph, clock, config, tmp_path):
    clock.advance(30.0)
    new_id = graph.observe(
        Observation("sponge", Point(0.0, -13.0, 0.9), clock.now(), 0.9)
    )
    graph.mark_scanned("kitchen", "sink")
    path = tmp_path / "snapshot.json"
    save_snapshot(graph.snapshot(), path)

    reloaded = SemanticGraph.from_snapshot(load_snapshot(path, config), config=config)
    assert reloaded.t_capture == clock.now()
    assert reloaded.objects[new_id].label == "sponge"
    assert reloaded.rooms["kitchen"].surfaces["sink"].last_scanned == clock.now()

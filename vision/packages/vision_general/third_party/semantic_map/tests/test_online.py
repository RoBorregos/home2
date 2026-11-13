"""Building the map online: what a real detector stream throws at it. No models needed."""

import math

import numpy as np
import pytest

from adapters.clock import ManualClock
from adapters.files import load_snapshot, save_snapshot
from core.config import SemanticMapConfig
from core.graph import SemanticGraph
from core.types import Observation, ParsedQuery, Point
from core.visibility import should_be_visible
from ports import LLMUnavailable

CROP = np.array([0.0, 1.0], np.float32)
OTHER_CROP = np.array([1.0, 0.0], np.float32)


class FakeEncoder:
    """Text vectors nearly orthogonal to CROP, as CLIP's text and image scores are."""

    dim = 2

    def __init__(self):
        self.fail = False

    def encode_text(self, text):
        if self.fail:
            raise TimeoutError("/vision/embed_text is not available")
        return np.array([1.0, 0.0], np.float32)


def seen(label="cup", x=0.0, y=0.0, z=0.75, stamp=1.0, confidence=0.9, **fields):
    return Observation(label, Point(x, y, z), stamp, confidence, **fields)


def online_graph(**config):
    return SemanticGraph(config=SemanticMapConfig(**config))


# --- fusion ---


def test_failed_crop_embedding_does_not_duplicate_the_object():
    graph = SemanticGraph(encoder=FakeEncoder())
    first = graph.observe(seen())  # EmbedCrops failed: only the label's text.
    second = graph.observe(seen(embedding=CROP))
    assert first == second and len(graph.objects) == 1
    obj = graph.objects[first]
    assert not obj.embedding_from_text and obj.embedding @ CROP == pytest.approx(1.0)


def test_text_vector_never_replaces_a_crop_embedding():
    graph = SemanticGraph(encoder=FakeEncoder())
    obj_id = graph.observe(seen(embedding=CROP))
    graph.observe(seen())
    assert graph.objects[obj_id].embedding @ CROP == pytest.approx(1.0)
    assert not graph.objects[obj_id].embedding_from_text


def test_different_crops_at_the_same_spot_are_two_objects():
    graph = online_graph()
    graph.observe(seen("cup", embedding=CROP))
    graph.observe(seen("mug", embedding=OTHER_CROP))
    assert len(graph.objects) == 2


def test_crop_embedding_is_the_normalized_mean():
    graph = online_graph(merge_sim_min=0.5)
    obj_id = graph.observe(seen(embedding=CROP))
    near = np.array([0.6, 0.8], np.float32)
    graph.observe(seen(embedding=near))
    vector = graph.objects[obj_id].embedding
    assert np.linalg.norm(vector) == pytest.approx(1.0)
    assert vector @ CROP > near @ CROP


def test_position_is_a_confidence_weighted_mean():
    graph = online_graph()
    obj_id = graph.observe(seen(x=0.0, confidence=0.9))
    graph.observe(seen(x=0.2, confidence=0.45))
    assert graph.objects[obj_id].position.x == pytest.approx(0.2 * 0.45 / 1.35)


def test_detection_joins_the_nearest_instance():
    graph = online_graph(merge_dist_m=0.5)
    far = graph.observe(seen(x=0.0))
    graph.objects[far].position = Point(0.4, 0.0, 0.75)
    near = graph.objects[far]
    other = graph.objects[graph.observe(seen(x=-0.3))]
    assert other is not near
    assert graph.observe(seen(x=-0.2)) == other.id


def test_weak_detection_keeps_the_confidence_and_the_best_label():
    graph = online_graph()
    obj_id = graph.observe(seen("cup", confidence=0.9, embedding=CROP, category="dish"))
    graph.observe(seen("bowl", confidence=0.5, embedding=CROP, category="other"))
    obj = graph.objects[obj_id]
    assert (obj.label, obj.category, obj.confidence) == ("cup", "dish", 0.9)
    graph.observe(seen("mug", confidence=0.95, embedding=CROP, category="dish"))
    assert (obj.label, obj.confidence) == ("mug", 0.95)


def test_encoder_failure_leaves_the_graph_untouched():
    encoder = FakeEncoder()
    graph = SemanticGraph(encoder=encoder)
    encoder.fail = True
    with pytest.raises(TimeoutError):
        graph.observe(seen())
    assert graph.objects == {}


def test_area_follows_the_fused_position(graph, clock):
    table = graph.rooms["kitchen"].surfaces["dinner_table"].pose
    obs = Observation("remote", Point(table.x, table.y, 0.75), clock.now(), 0.9)
    obj = graph.objects[graph.observe(obs)]
    assert (obj.area, obj.sublocation) == ("kitchen", "dinner_table")


# --- confirmation ---


def test_objects_answer_queries_only_after_enough_detections():
    graph = online_graph(confirm_min_observations=3)
    graph.observe(seen())
    graph.observe(seen())
    assert graph.find("cup") == [] and graph.known_objects() == []
    graph.observe(seen())
    assert [m.obj.label for m in graph.find("cup")] == ["cup"]


def test_unconfirmed_objects_are_not_reported_misplaced():
    graph = online_graph(confirm_min_observations=2)
    graph.config.category_to_furniture = {"dish": ["kitchen", "sink"]}
    graph.load_layout(
        {
            "kitchen": {
                "polygon": [[-5, -5], [5, -5], [5, 5], [-5, 5]],
                "sink": [9.0, 9.0, 0, 0, 0, 0, 1.0],
                "table": [0.0, 0.0, 0, 0, 0, 0, 1.0],
            }
        }
    )
    graph.observe(seen(category="dish"))
    assert graph.misplaced() == []
    graph.observe(seen(category="dish"))
    assert [d.label for d in graph.misplaced()] == ["cup"]


# --- forgetting ---


def test_object_is_forgotten_after_miss_limit_looks():
    graph = online_graph(miss_limit=3)
    obj_id = graph.observe(seen())
    assert graph.report_unseen([obj_id]) == []
    assert graph.report_unseen([obj_id]) == []
    assert graph.report_unseen([obj_id]) == [obj_id]
    assert graph.objects == {}


def test_a_detection_resets_the_misses():
    graph = online_graph(miss_limit=2)
    obj_id = graph.observe(seen())
    graph.report_unseen([obj_id])
    graph.observe(seen())
    assert graph.report_unseen([obj_id]) == []
    assert graph.objects[obj_id].misses == 1


def test_unknown_ids_are_ignored():
    assert online_graph().report_unseen(["obj_404"]) == []


def test_forgotten_ids_are_not_reused():
    graph = online_graph(miss_limit=1)
    first = graph.observe(seen(x=0.0))
    second = graph.observe(seen(x=5.0))
    graph.report_unseen([first])
    third = graph.observe(seen(x=9.0))
    assert len({first, second, third}) == 3


def test_reset_keeps_the_map(graph):
    rooms = dict(graph.rooms)
    graph.reset_objects()
    assert graph.objects == {} and graph.rooms == rooms


# --- commands ---


def test_uncalibrated_command_for_something_absent_is_not_found():
    graph = SemanticGraph(encoder=FakeEncoder(), config=SemanticMapConfig(sim_min=0.5))
    graph.observe(seen(embedding=CROP))
    assert graph.answer("elephant").status == "not_found"


def test_commands_with_no_objects_yet_find_nothing():
    graph = SemanticGraph(encoder=FakeEncoder())
    assert graph.find("cup") == []
    assert graph.answer("cup").status == "not_found"


class FlakyLLM:
    def __init__(self):
        self.down = True

    def complete_json(self, prompt, schema):
        if self.down:
            raise LLMUnavailable("timeout")
        return {"target": "cup", "anchor": "", "relation": "", "room": ""}


def test_llm_failure_degrades_and_is_not_cached():
    llm = FlakyLLM()
    graph = SemanticGraph(llm=llm)
    assert graph.parse("bring me the cup").target == "bring me the cup"
    llm.down = False
    assert graph.parse("bring me the cup").target == "cup"


# --- the map as tagged on the robot ---

ROOM = [[-5, -5], [5, -5], [5, 5], [-5, 5]]
FACING_EAST = [0.0, 0.0, 0, 0, 0, 0, 1.0]
FACING_WEST = [1.0, 0.0, 0, 0, 0, 1.0, 0.0]


def test_surface_is_where_the_furniture_is_not_where_the_robot_stands():
    layout = {
        "kitchen": {
            "polygon": ROOM,
            "east_table": FACING_EAST,
            "west_table": FACING_WEST,
        }
    }
    graph = online_graph(surface_standoff_m=0.8)
    graph.load_layout(layout)
    # Right on east_table (x=0.8), yet nearer the pose tagged for west_table (x=1).
    assert graph.area_for_point(0.75, 0.0, 0.75).sublocation == "east_table"


def test_furniture_just_outside_its_room_still_belongs_to_it():
    # The room is drawn up to the wall; the cabinet stands against it, 0.5 m past.
    layout = {"kitchen": {"polygon": ROOM, "cabinet": [4.6, 0.0, 0, 0, 0, 0, 1.0]}}
    graph = online_graph(surface_standoff_m=0.8, room_margin_m=0.6)
    graph.load_layout(layout)
    where = graph.area_for_point(5.4, 0.0, 0.9)
    assert (where.area, where.sublocation) == ("kitchen", "cabinet")
    assert graph.area_for_point(5.8, 0.0).area == ""  # past the margin: unknown


def test_room_margin_is_off_by_default():
    graph = online_graph()
    graph.load_layout({"kitchen": {"polygon": ROOM}})
    assert graph.area_for_point(5.1, 0.0).area == ""


def test_layout_skips_what_is_not_a_surface():
    graph = online_graph()
    graph.load_layout(
        {
            "kitchen": {
                "polygon": ROOM,
                "table": FACING_EAST,
                "house_entry": FACING_EAST,
                "broken": [1.0, 2.0],
                "nan": [None] * 7,
            },
            "notes": "not an area",
        }
    )
    assert list(graph.rooms) == ["kitchen"]
    assert list(graph.rooms["kitchen"].surfaces) == ["table"]


def test_misplaced_ignores_surfaces_this_map_does_not_have():
    graph = online_graph()
    graph.load_layout({"kitchen": {"polygon": ROOM, "table": FACING_EAST}})
    graph.config.category_to_furniture = {"drink": ["kitchen", "refrigerator"]}
    graph.observe(seen(category="drink"))
    assert graph.misplaced() == []


def test_misplaced_ignores_objects_with_no_known_surface():
    graph = online_graph()
    graph.load_layout(
        {"kitchen": {"polygon": ROOM, "table": [40.0, 40.0, 0, 0, 0, 0, 1.0]}}
    )
    graph.config.category_to_furniture = {"drink": ["kitchen", "table"]}
    graph.observe(seen(category="drink"))
    assert graph.misplaced() == []


# --- persistence ---


def test_online_state_survives_a_snapshot(tmp_path, config):
    graph = SemanticGraph(config=config, encoder=FakeEncoder(), clock=ManualClock(5.0))
    graph.load_layout({"kitchen": {"polygon": ROOM, "table": FACING_EAST}})
    text_only = graph.observe(seen(x=0.0))
    with_crop = graph.observe(seen(x=3.0, embedding=CROP))
    path = tmp_path / "snapshot.json"
    save_snapshot(graph.snapshot(), path)

    loaded = load_snapshot(path, config).objects
    assert loaded[text_only].embedding_from_text
    assert not loaded[with_crop].embedding_from_text
    assert loaded[with_crop].position_weight == pytest.approx(0.9)

    restored = SemanticGraph.from_snapshot(load_snapshot(path, config), config=config)
    assert restored.observe(seen(x=9.0)) not in (text_only, with_crop)


# --- visibility ---

CAMERA = dict(
    fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, max_range_m=2.0
)


def depth_at(meters):
    return np.full((480, 640), meters, np.float32)


def test_point_in_clear_view_is_visible():
    assert should_be_visible((0.0, 0.0, 1.0), depth=depth_at(1.0), **CAMERA)


def test_empty_spot_with_the_background_behind_is_visible():
    assert should_be_visible((0.0, 0.0, 1.0), depth=depth_at(3.0), **CAMERA)


def test_point_hidden_behind_something_nearer_is_not_visible():
    assert not should_be_visible((0.0, 0.0, 1.5), depth=depth_at(0.5), **CAMERA)


@pytest.mark.parametrize(
    "point",
    [
        (0.0, 0.0, -1.0),
        (0.0, 0.0, 2.5),
        (5.0, 0.0, 1.0),
        (0.0, 5.0, 1.0),
        (0.62, 0.0, 1.0),
    ],
    ids=["behind", "past range", "off to the side", "off the bottom", "on the border"],
)
def test_point_out_of_frame_or_range_is_not_visible(point):
    assert not should_be_visible(point, depth=depth_at(1.0), **CAMERA)


@pytest.mark.parametrize("bad", [0.0, math.nan, math.inf])
def test_without_valid_depth_it_does_not_count(bad):
    assert not should_be_visible((0.0, 0.0, 1.0), depth=depth_at(bad), **CAMERA)
    assert not should_be_visible((0.0, 0.0, 1.0), depth=None, **CAMERA)


# --- what the robot's LLM and detector actually returned on lab_7_nov ---


class SchemaEchoLLM:
    """qwen3.5-4b on the robot: the values nested under "properties"."""

    def complete_json(self, prompt, schema):
        target = "cup" if "cup" in prompt.split("Command:")[-1] else "banana"
        return {
            "type": "object",
            "properties": {"target": target, "anchor": "", "relation": "", "room": ""},
            "required": ["target", "anchor", "relation", "room"],
        }


def test_values_echoed_under_properties_are_read():
    graph = SemanticGraph(llm=SchemaEchoLLM())
    assert graph.parse("I am looking for a cup.").target == "cup"


def test_a_plain_answer_is_still_read():
    class Plain:
        def complete_json(self, prompt, schema):
            return {"target": "trash bin", "anchor": "", "relation": "", "room": ""}

    assert SemanticGraph(llm=Plain()).parse("Trash bin.").target == "trash bin"


class OrthogonalEncoder:
    """Text and crops far apart, as when a detector crop looks nothing like CLIP's idea of it."""

    dim = 3
    vectors = {}

    def encode_text(self, text):
        key = text.replace("a photo of a ", "")
        if key not in self.vectors:
            v = np.zeros(3, np.float32)
            v[len(self.vectors) % 3] = 1.0
            self.vectors[key] = v
        return self.vectors[key]


def test_an_object_named_outright_wins_even_with_a_poor_crop():
    graph = SemanticGraph(
        encoder=OrthogonalEncoder(), config=SemanticMapConfig(sim_min=0.24)
    )
    cup_crop = np.array([0.0, 0.6, 0.8], np.float32)
    bin_crop = np.array([0.0, 0.0, 1.0], np.float32)
    graph.observe(seen("cup", x=0.0, embedding=cup_crop))
    graph.observe(seen("trash_bin", x=5.0, embedding=bin_crop))
    answer = graph.answer("trash bin", ParsedQuery(target="trash bin"))
    assert answer.status == "found"
    assert answer.matches[0].obj.label == "trash_bin"


@pytest.mark.parametrize(
    "text,label,expected",
    [
        ("Trash bin.", "trash_bin", True),
        ("where are the trash bins", "trash_bin", True),
        ("I'm looking for a cup.", "cup", True),
        ("cupboard", "cup", False),
        ("bin", "trash_bin", False),
    ],
)
def test_naming_a_label(text, label, expected):
    from core.graph import _names_label

    assert _names_label(text, label) is expected

"""Geometry, placement rules, staleness and fusion. No models needed."""

import pytest

from core.types import Observation, Point


def test_point_resolves_to_room_and_surface(graph):
    table = graph.rooms["kitchen"].surfaces["dinner_table"].pose
    where = graph.area_for_point(table.x, table.y, 0.75)
    assert (where.area, where.sublocation) == ("kitchen", "dinner_table")


def test_point_on_the_floor_is_not_on_furniture(graph):
    table = graph.rooms["kitchen"].surfaces["dinner_table"].pose
    where = graph.area_for_point(table.x, table.y, 0.03)
    assert where.area == "kitchen" and where.sublocation != "dinner_table"


def test_point_outside_every_polygon_is_unknown(graph):
    where = graph.area_for_point(100.0, 100.0)
    assert where.area == "" and not where.in_house


def test_referee_waypoints_are_not_rooms(graph):
    entrance = graph.rooms["entrance"].safe_place
    assert graph.area_for_point(entrance.x, entrance.y).area != "entrance"


def test_misplaced_follows_category_rules(graph):
    misplaced = {d.label for d in graph.misplaced()}
    assert {"cereal", "cup", "banana", "orange_juice"} <= misplaced
    # Where it belongs, without a rule, or without a category.
    assert not {"coca_cola", "apple", "tshirt", "milk"} & misplaced


def test_stale_lists_never_seen_first(graph):
    stale = graph.stale()
    assert [(s.name, s.last_scanned) for s in stale[:2]] == [("trash", None)] * 2
    assert all(s.age_s > graph.config.stale_after_s for s in stale)
    assert "refrigerator" not in {s.name for s in stale}


def test_mark_scanned_clears_staleness(graph, clock):
    graph.mark_scanned("kitchen", "dinner_table")
    assert "dinner_table" not in {s.name for s in graph.stale()}
    clock.advance(graph.config.stale_after_s + 1)
    assert "dinner_table" in {s.name for s in graph.stale()}


def test_observe_fuses_a_repeated_detection(graph, clock):
    coke = graph.objects["obj_001"]
    seen = coke.observation_count
    obs = Observation("coca_cola", coke.position, clock.now(), confidence=0.9)
    assert graph.observe(obs) == "obj_001"
    assert coke.observation_count == seen + 1 and coke.last_seen == clock.now()


def test_observe_creates_and_locates_a_new_object(graph, clock):
    sofa = graph.rooms["living_room"].surfaces["sofa"].pose
    obs = Observation(
        "remote", Point(sofa.x, sofa.y, 0.45), clock.now(), confidence=0.9
    )
    new = graph.objects[graph.observe(obs)]
    assert (new.area, new.sublocation) == ("living_room", "sofa")


def test_observe_drops_low_confidence(graph, clock):
    obs = Observation("remote", Point(0.0, 0.0), clock.now(), confidence=0.1)
    assert graph.observe(obs) is None


class FakeVLM:
    """Answers a fixed image number and remembers which crops it was shown."""

    def __init__(self, image):
        self.image, self.shown = image, []

    def complete_json(self, prompt, schema, images=()):
        self.shown = list(images)
        return {"image": self.image}


@pytest.fixture
def cascade(graph):
    """Keeps every candidate scoring at least 1 by the score threshold."""
    graph.config.set_method, graph.config.conformal_cutoff = "threshold", -1.0
    return graph


def test_answer_without_calibration_trusts_the_top_match(graph):
    answer = graph.answer("coca_cola")
    assert answer.status == "found" and answer.matches[0].obj.label == "coca_cola"


def test_answer_with_a_single_candidate_skips_the_vlm(cascade):
    cascade.vlm = FakeVLM("1")
    answer = cascade.answer("coca_cola")
    assert answer.status == "found" and not answer.vlm_used


def test_answer_finds_nothing(cascade):
    assert cascade.answer("elephant").status == "not_found"


def test_answer_lets_the_vlm_break_a_tie(cascade):
    cascade.vlm = FakeVLM("2")
    answer = cascade.answer("cutlery")
    assert answer.status == "found" and answer.vlm_used
    assert answer.matches[0].obj.crop == cascade.vlm.shown[1]


def test_answer_asks_when_the_vlm_picks_none(cascade):
    cascade.vlm = FakeVLM("")
    answer = cascade.answer("cutlery")
    assert answer.status == "ask" and len(answer.matches) == 3


def test_answer_trusts_clip_while_the_set_is_small(cascade):
    cascade.vlm = FakeVLM("2")
    cascade.config.escalate_above = 3
    answer = cascade.answer("cutlery")
    assert answer.status == "found" and not answer.vlm_used


def test_answer_shows_the_vlm_at_most_max_candidates(cascade):
    cascade.vlm = FakeVLM("1")
    cascade.config.max_candidates = 2
    cascade.answer("cutlery")
    assert len(cascade.vlm.shown) == 2


def test_calibrate_covers_the_calibration_commands(graph):
    graph.config.set_method = "threshold"
    knife = next(o.id for o in graph.objects.values() if o.label == "knife")
    graph.calibrate([("coca_cola", {"obj_001"}), ("cutlery", {knife})] * 10)
    assert len(graph.candidates(graph.parse("cutlery"))) == 3

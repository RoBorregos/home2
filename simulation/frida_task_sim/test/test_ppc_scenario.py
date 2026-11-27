"""Scenario generation and scoring checks that need no simulator."""

from frida_task_sim import scenario as scenarios, scoring, venue as venues, world


def test_same_seed_same_scenario():
    assert scenarios.generate("ppc", 7) == scenarios.generate("ppc", 7)
    assert scenarios.generate("ppc", 7) != scenarios.generate("ppc", 8)


def test_ppc_follows_the_rulebook_setup():
    for seed in range(20):
        objects = scenarios.generate("ppc", seed)["objects"]
        table = [o for o in objects if o["surface"] == "dinner_table"]
        roles = sorted(o["role"] for o in table)
        assert roles == ["cutlery", "other", "other", "tableware", "tableware", "trash"]
        assert sum(o["role"] == "common" for o in objects) == 2
        assert {o["name"] for o in objects if o["role"] == "breakfast"} == {
            "bowl",
            "spoon",
            "milk",
            "cornflakes",
        }
        assert len({o["id"] for o in objects}) == len(objects)


def test_objects_spawn_on_their_surface():
    scenario = scenarios.generate("ppc", 3)
    venue = venues.load(scenario["venue"])
    for obj in scenario["objects"]:
        location, level = venues.location_of(venue, *obj["pose"][:3])
        assert (location, level) == (obj["surface"], obj["level"]), obj["id"]


def test_world_contains_every_entity():
    scenario = scenarios.generate("ppc", 1)
    sdf = world.build(scenario)
    for obj in scenario["objects"]:
        assert f'<model name="{obj["id"]}">' in sdf
    assert '<model name="cabinet">' in sdf


def _event(kind, t=1.0, **data):
    return {"t": t, "type": kind, **data}


def test_scoring_a_clean_pick_and_place():
    scenario = scenarios.generate("ppc", 1)
    cup = next(o for o in scenario["objects"] if o["name"] == "cup")
    events = [
        _event("at_table"),
        _event(
            "recognize", name="cup", at="dinner_table", correct=True, object=cup["id"]
        ),
        _event(
            "pick",
            object=cup["id"],
            name="cup",
            role="tableware",
            source="dinner_table",
            first=True,
        ),
        _event(
            "place",
            object=cup["id"],
            name="cup",
            role="tableware",
            category="dish",
            location="dishwasher",
            level=None,
            drop=0.02,
        ),
    ]
    result = scoring.score(scenario, events, {})
    rows = {r["id"]: r["total"] for r in result["rows"]}
    assert rows["navigate_to_table"] == 15
    assert rows["recognize_object"] == 10
    assert rows["pick"] == 50 and rows["first_pick"] == 100
    assert rows["place"] == 40
    assert result["total"] == 215


def test_scoring_penalizes_drops_and_wrong_places():
    scenario = scenarios.generate("ppc", 1)
    cup = next(o for o in scenario["objects"] if o["name"] == "cup")
    pick = _event(
        "pick",
        object=cup["id"],
        name="cup",
        role="tableware",
        source="dinner_table",
        first=True,
    )
    place = dict(
        object=cup["id"], name="cup", role="tableware", category="dish", level=None
    )
    wrong = scoring.score(
        scenario,
        [pick, _event("place", location="cabinet", drop=0.02, **{**place, "level": 1})],
        {},
    )
    assert {r["id"]: r["total"] for r in wrong["rows"]}["place"] == 0
    floor = scoring.score(
        scenario, [pick, _event("place", location="floor", drop=0.6, **place)], {}
    )
    assert {r["id"]: r["total"] for r in floor["rows"]}["dropped_floor"] == -40
    thrown = scoring.score(
        scenario, [pick, _event("place", location="dishwasher", drop=0.3, **place)], {}
    )
    assert {r["id"]: r["total"] for r in thrown["rows"]}["thrown"] == -40

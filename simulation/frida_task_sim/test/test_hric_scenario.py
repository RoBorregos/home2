"""HRIC scenarios are reproducible and the scoresheet counts the director's judgements."""

from frida_task_sim import scenario as scenarios, scoring, world


def _judge(row, good=True):
    return {
        "t": 1.0,
        "type": "judge",
        "row": row,
        "good": good,
        "title": "",
        "detail": row,
    }


def test_same_seed_same_scenario():
    assert scenarios.generate("hric", 4) == scenarios.generate("hric", 4)
    assert scenarios.generate("hric", 4) != scenarios.generate("hric", 5)


def test_hric_follows_the_rulebook_setup():
    scenario = scenarios.generate("hric", 2)
    cast = {p["id"]: p for p in scenario["people"]}
    assert set(cast) == {"host", "guest_1", "guest_2"}
    assert len({p["name"] for p in cast.values()}) == 3
    assert cast["guest_1"]["drink"] != cast["guest_2"]["drink"]
    assert cast["host"]["seat"] in range(len(scenario["seats"]))
    built = world.build(scenario)
    assert all(f'name="{name}"' in built for name in (*cast, "bag", "seat_0"))


def test_scoring_counts_judgements_and_the_bag():
    scenario = scenarios.generate("hric", 2)
    events = [
        _judge("doorbell"),
        _judge("doorbell"),
        _judge("bag_handover"),
        _judge("", False),
    ]
    drop = scenario["drop"]["center"]
    final = {
        "bag": {
            "position": [drop[0] + 0.5, drop[1], 0.0],
            "location": "floor",
            "level": None,
        }
    }
    rows = {r["id"]: r["total"] for r in scoring.score(scenario, events, final)["rows"]}
    assert (rows["doorbell"], rows["bag_handover"], rows["bag_drop_area"]) == (
        60,
        50,
        50,
    )
    far = {
        "bag": {
            "position": [drop[0] + 5.0, drop[1], 0.0],
            "location": "floor",
            "level": None,
        }
    }
    assert scoring.score(scenario, events, far)["total"] == 110


def test_wrong_guest_information_is_penalized():
    scenario = scenarios.generate("hric", 2)
    events = [_judge("intro_info"), _judge("wrong_info", False)]
    assert scoring.score(scenario, events, {})["total"] == 30 - 40

#!/usr/bin/env python3
"""
Headless end-to-end test of the brief execution loop.

Drives select -> build tree -> tick -> update world against a scripted fake robot, with
a simulated clock. No ROS and no hardware, so it can gate CI: it is the cheapest place
to catch "the robot picks the same object twice" or "it never gets to the pours".
"""

import os
import sys
import time
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


def _stub_rclpy() -> None:
    if "rclpy" in sys.modules:
        return
    rclpy = types.ModuleType("rclpy")
    action = types.ModuleType("rclpy.action")
    client = types.ModuleType("rclpy.client")
    action.ActionClient = type("ActionClient", (), {})
    client.Client = type("Client", (), {})
    rclpy.action = action
    rclpy.client = client
    sys.modules.update({"rclpy": rclpy, "rclpy.action": action, "rclpy.client": client})


_stub_rclpy()

import py_trees  # noqa: E402

from task_manager.executor.objective_tree import build_objective_tree  # noqa: E402
from task_manager.gpsr.skill_runner import SkillRunner  # noqa: E402
from task_manager.planner.brief import briefs_dir, load_brief, parse_brief  # noqa: E402
from task_manager.planner.manifest import Manifest  # noqa: E402
from task_manager.planner.selector import choose, constant_travel  # noqa: E402
from task_manager.utils.status import Status  # noqa: E402
from task_manager.world.world_model import WorldModel  # noqa: E402


class FakeRobot:
    """Scripted subtask manager: every skill succeeds unless named in `fails`."""

    def __init__(self, fails=()):
        self.fails = set(fails)
        self.calls = []
        self.nav = self
        self.vision = self
        self.manipulation = self
        self.hri = self

    def _run(self, name, **kwargs):
        self.calls.append((name, kwargs))
        if name in self.fails:
            return Status.TARGET_NOT_FOUND, None
        return Status.EXECUTION_SUCCESS, f"{name} ok"

    # names here must match Skill.method in the registry
    def move_to_location(self, location, sublocation=""):
        return self._run("move_to_location", location=location, sublocation=sublocation)

    def pick_object(self, object_name):
        return self._run("pick_object", object_name=object_name)

    def place(self, close_to="", is_trash=False):
        return self._run("place", close_to=close_to, is_trash=is_trash)

    def place_on_shelf(self, plane_height, tolerance=0):
        return self._run("place_on_shelf", plane_height=plane_height)

    def pour(self, pour_object_name, pour_container_name):
        return self._run("pour", pour_object_name=pour_object_name)

    def detect_objects(self, label="all"):
        return self._run("detect_objects", label=label)

    def move_to_position(self, named_position):
        return self._run("move_to_position", named_position=named_position)

    def say(self, text, wait=True):
        return self._run("say", text=text)

    def confirm(self, text):
        return self._run("confirm", text=text)

    def skill_names(self):
        return [name for name, _ in self.calls]


def run_tree(tree_root, limit_s: float = 10.0):
    terminal = (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)
    start = time.time()
    while time.time() - start < limit_s:
        tree_root.tick_once()
        if tree_root.status in terminal:
            return tree_root.status
        time.sleep(0.005)
    raise AssertionError("objective tree never finished")


def test_objective_tree_runs_steps_in_order() -> None:
    brief = parse_brief(
        {
            "task": "unit",
            "budget_s": 420,
            "objectives": [
                {
                    "id": "fetch",
                    "points": 90,
                    "template": [
                        "go_to(kitchen, dinner_table)",
                        "pick_object(apple)",
                        "place(close_to=bowl)",
                    ],
                }
            ],
        }
    )
    robot = FakeRobot()
    runner = SkillRunner()
    root = build_objective_tree(brief.objectives[0].template, {}, robot, runner, name="fetch")
    root.setup_with_descendants()

    assert run_tree(root) == py_trees.common.Status.SUCCESS
    assert robot.skill_names() == ["move_to_location", "pick_object", "place"]
    assert robot.calls[0][1]["sublocation"] == "dinner_table"
    runner.shutdown()
    print("✓ objective tree runs its steps in order with bound arguments")


def test_failed_step_stops_the_objective() -> None:
    brief = parse_brief(
        {
            "task": "unit",
            "budget_s": 420,
            "objectives": [
                {
                    "id": "fetch",
                    "points": 90,
                    "template": ["pick_object(apple)", "place()"],
                }
            ],
        }
    )
    robot = FakeRobot(fails={"pick_object"})
    runner = SkillRunner()
    root = build_objective_tree(brief.objectives[0].template, {}, robot, runner)
    root.setup_with_descendants()

    assert run_tree(root) == py_trees.common.Status.FAILURE
    assert "place" not in robot.skill_names(), "must not place after a failed pick"
    runner.shutdown()
    print("✓ a failed pick aborts the objective instead of placing thin air")


def simulate_run(brief, robot, manifest, world, budget_s):
    """Mirror BriefRunner's loop with a simulated clock."""
    runner = SkillRunner()
    elapsed = 0.0
    done = []

    while elapsed < budget_s:
        candidate = choose(brief, world, manifest, budget_s - elapsed, constant_travel())
        if candidate is None:
            break

        steps = candidate.objective.template
        if candidate.use_fallback:
            fallback = brief.objective(candidate.objective.fallback)
            steps = fallback.template if fallback else steps

        root = build_objective_tree(steps, candidate.binding, robot, runner)
        root.setup_with_descendants()
        status = run_tree(root)

        elapsed += candidate.total_s
        target = candidate.binding.get("obj") or candidate.binding.get("item")
        world.note_attempt(candidate.objective.id, target)
        succeeded = status == py_trees.common.Status.SUCCESS

        if succeeded:
            world.mark_done(candidate.objective.id)
            done.append(candidate.objective.id)
            if target is not None:
                target.placed = True
            # BriefRunner folds effects in per skill, and only on success
            from task_manager.skills.registry import REGISTRY

            for step in steps:
                skill = REGISTRY.get(step.skill)
                if skill:
                    world.predicates.update(skill.sets)
                    world.predicates.difference_update(skill.clears)
        elif target is not None and world.attempts_for(candidate.objective.id, target) >= (
            candidate.objective.max_attempts
        ):
            target.skipped = True

        if candidate.objective.at:
            world.arrive(candidate.objective.at)

    runner.shutdown()
    return done, elapsed


def build_world(names) -> WorldModel:
    world = WorldModel()
    for name in names:
        world.observe(name, surface="dinner_table", destination="cabinet")
    world.predicates.add("objects_known")
    world.arrive("dinner_table")
    return world


def test_full_run_stays_inside_the_budget() -> None:
    brief = load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))
    robot = FakeRobot()
    world = build_world(["pringles", "apple", "coca_cola"])

    done, elapsed = simulate_run(brief, robot, Manifest(), world, brief.budget_s)

    assert elapsed <= brief.budget_s, f"overran the budget: {elapsed:.0f}s"
    assert done, "the run accomplished nothing"
    print(f"✓ full pick_and_place run fits in {elapsed:.0f}s of {brief.budget_s:.0f}s")


def test_no_object_is_worked_twice() -> None:
    """The old FSM's failure mode: repeating a pick because state was tracked by index."""
    brief = load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))
    robot = FakeRobot()
    world = build_world(["pringles", "apple"])

    simulate_run(brief, robot, Manifest(), world, brief.budget_s)

    picked = [kwargs["object_name"] for name, kwargs in robot.calls if name == "pick_object"]
    assert len(picked) == len(set(picked)), f"picked something twice: {picked}"
    print("✓ no object is picked twice in a run")


def pick_only_brief(max_attempts: int = 2):
    """Isolates pick_place so retry behaviour is not masked by richer objectives."""
    return parse_brief(
        {
            "task": "unit",
            "budget_s": 420,
            "objectives": [
                {
                    "id": "pick_place",
                    "points": 90,
                    "repeat_for": "table_objects",
                    "requires": ["arm_free"],
                    "max_attempts": max_attempts,
                    "template": ["pick_object({obj.name})", "place()"],
                }
            ],
        }
    )


def test_failing_gripper_does_not_wedge_the_run() -> None:
    """If picking never works the run must terminate, not spin forever on one object."""
    brief = pick_only_brief(max_attempts=2)
    robot = FakeRobot(fails={"pick_object"})
    world = build_world(["pringles", "apple"])

    _, elapsed = simulate_run(brief, robot, Manifest(), world, brief.budget_s)

    assert elapsed <= brief.budget_s
    picked = [kwargs["object_name"] for name, kwargs in robot.calls if name == "pick_object"]
    # two objects, two attempts each, and then it gives up rather than thrashing
    assert len(picked) == 4, picked
    assert picked.count("pringles") == 2 and picked.count("apple") == 2, picked
    print(f"✓ a broken gripper retries each object twice then stops ({len(picked)} attempts)")


def test_one_bad_object_does_not_block_the_others() -> None:
    """Attempts are per target: a stuck object must not retire the whole objective."""
    brief = pick_only_brief(max_attempts=2)

    class PickyRobot(FakeRobot):
        """Only the plate is unpickable — it needs a flat grasp the arm cannot do."""

        def pick_object(self, object_name):
            self.calls.append(("pick_object", {"object_name": object_name}))
            if object_name == "red_plate":
                return Status.TARGET_NOT_FOUND, None
            return Status.EXECUTION_SUCCESS, f"picked {object_name}"

    robot = PickyRobot()
    world = build_world(["red_plate", "pringles", "apple"])
    simulate_run(brief, robot, Manifest(), world, brief.budget_s)

    attempted = [kwargs["object_name"] for name, kwargs in robot.calls if name == "pick_object"]
    assert attempted.count("red_plate") == 2, f"plate should stop after 2 tries: {attempted}"
    assert "pringles" in attempted and "apple" in attempted, attempted
    assert world.objects["red_plate"].skipped, "the stuck object should be retired"
    assert world.objects["pringles"].placed and world.objects["apple"].placed
    print("✓ one unpickable object does not block the rest")


if __name__ == "__main__":
    test_objective_tree_runs_steps_in_order()
    test_failed_step_stops_the_objective()
    test_full_run_stays_inside_the_budget()
    test_no_object_is_worked_twice()
    test_failing_gripper_does_not_wedge_the_run()
    test_one_bad_object_does_not_block_the_others()
    print("\nAll brief execution tests passed.")

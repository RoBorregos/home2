"""
Declarative catalogue of the robot's skills.

Single source of truth for four consumers that used to disagree with each other:
behaviour-tree leaf dispatch, the selector's precondition checks, the MCP tool list,
and anything that needs to know what a skill costs or changes.

Dispatch used to be a bare ``getattr(handler, action_name)``, which made every public
method on a subtask manager reachable by name — including ``timeout`` and
``test_function`` — with no arity checking. Going through the registry closes that.

ROS-free: this module is pure data so tooling can import it without a workspace.
"""

from dataclasses import dataclass
from typing import Any, Optional

# world predicates used by requires/sets/clears
ARM_FREE = "arm_free"
HOLDING = "holding"
OBJECTS_KNOWN = "objects_known"
PERSON_FOUND = "person_found"


@dataclass(frozen=True)
class SkillArg:
    name: str
    type: str = "str"
    required: bool = True
    description: str = ""
    default: Any = None


@dataclass(frozen=True)
class Skill:
    """One callable robot capability."""

    name: str
    area: str  # attribute on SubtaskManager: nav | vision | manipulation | hri
    method: str  # method name on that area object
    summary: str
    args: tuple[SkillArg, ...] = ()
    requires: tuple[str, ...] = ()
    sets: tuple[str, ...] = ()
    clears: tuple[str, ...] = ()
    # read-only skills are safe to expose without an explicit opt-in over MCP
    mutates_world: bool = True
    # rough prior in seconds, replaced per-context by the measured manifest
    prior_duration_s: float = 20.0
    prior_success: float = 0.7

    def arg(self, name: str) -> Optional[SkillArg]:
        for candidate in self.args:
            if candidate.name == name:
                return candidate
        return None


def _skills() -> tuple[Skill, ...]:
    return (
        # ---------------- navigation ----------------
        Skill(
            name="go_to",
            area="nav",
            method="move_to_location",
            summary="Drive to a named area/subarea from the map.",
            args=(
                SkillArg("location", description="room or furniture name"),
                SkillArg("sublocation", required=False, default="", description="dock point"),
            ),
            prior_duration_s=31.0,
            prior_success=0.95,
        ),
        Skill(
            name="path_distance",
            area="nav",
            method="get_path_info",
            summary="Real path distance in metres between two areas, without moving.",
            args=(
                SkillArg("location_b"),
                SkillArg("sublocation_b", required=False, default=""),
                SkillArg("location_a", required=False, default=""),
                SkillArg("sublocation_a", required=False, default=""),
            ),
            mutates_world=False,
            prior_duration_s=0.5,
            prior_success=0.98,
        ),
        Skill(
            name="dock_table",
            area="nav",
            method="dock_table",
            summary="Approach and align to the table in front of the robot.",
            args=(SkillArg("offset", type="float", required=False, default=0.0),),
            prior_duration_s=12.0,
        ),
        Skill(
            name="check_door",
            area="nav",
            method="check_door",
            summary="Report whether the arena door in front of the robot is open.",
            mutates_world=False,
            prior_duration_s=1.0,
        ),
        Skill(
            name="return_to_origin",
            area="nav",
            method="return_to_origin",
            summary="Drive back to the run's start pose.",
            prior_duration_s=31.0,
        ),
        Skill(
            name="rotate_in_place",
            area="nav",
            method="rotate_in_place",
            summary="Rotate the base by a relative angle in degrees.",
            args=(SkillArg("degrees", type="float"),),
            prior_duration_s=5.0,
        ),
        # ---------------- vision ----------------
        Skill(
            name="detect_objects",
            area="vision",
            method="detect_objects",
            summary="Detect objects in view; returns bounding boxes with labels.",
            args=(
                SkillArg("label", required=False, default="all", description="'all' or a label"),
            ),
            sets=(OBJECTS_KNOWN,),
            mutates_world=False,
            prior_duration_s=6.0,
            prior_success=0.9,
        ),
        Skill(
            name="detect_person",
            area="vision",
            method="detect_person",
            summary="Detect whether a person is in view.",
            sets=(PERSON_FOUND,),
            mutates_world=False,
            prior_duration_s=4.0,
        ),
        Skill(
            name="find_seat",
            area="vision",
            method="find_seat",
            summary="Find a free seat and return its bearing.",
            mutates_world=False,
            prior_duration_s=8.0,
        ),
        Skill(
            name="count_objects",
            area="vision",
            method="count_objects",
            summary="Count objects of a category in view.",
            args=(SkillArg("object"),),
            mutates_world=False,
            prior_duration_s=6.0,
        ),
        Skill(
            name="describe_person",
            area="vision",
            method="describe_person",
            summary="Produce a visual description of the person in view.",
            mutates_world=False,
            prior_duration_s=12.0,
        ),
        Skill(
            name="visual_query",
            area="vision",
            method="moondream_query",
            summary="Ask a free-form visual question about the current view.",
            args=(SkillArg("prompt"),),
            mutates_world=False,
            prior_duration_s=15.0,
        ),
        Skill(
            name="detect_hand",
            area="vision",
            method="detect_hand",
            summary="Locate an outstretched hand for a handover.",
            mutates_world=False,
            prior_duration_s=6.0,
        ),
        Skill(
            name="get_customer",
            area="vision",
            method="get_customer",
            summary="Find a calling or waving customer and return their position.",
            mutates_world=False,
            prior_duration_s=8.0,
            prior_success=0.7,
        ),
        Skill(
            name="save_face",
            area="vision",
            method="save_face_name",
            summary="Remember the face currently in view under a name.",
            args=(SkillArg("name"),),
            prior_duration_s=5.0,
            prior_success=0.85,
        ),
        # ---------------- manipulation ----------------
        Skill(
            name="pick_object",
            area="manipulation",
            method="pick_object",
            summary="Pick a named object from the surface in front of the robot.",
            args=(SkillArg("object_name"),),
            requires=(ARM_FREE,),
            sets=(HOLDING,),
            clears=(ARM_FREE,),
            prior_duration_s=35.0,
            prior_success=0.6,
        ),
        Skill(
            name="place",
            area="manipulation",
            method="place",
            summary="Place the held object on the surface in front of the robot.",
            args=(
                SkillArg("close_to", required=False, default="", description="anchor object"),
                SkillArg("is_trash", type="bool", required=False, default=False),
            ),
            requires=(HOLDING,),
            sets=(ARM_FREE,),
            clears=(HOLDING,),
            prior_duration_s=25.0,
            prior_success=0.75,
        ),
        Skill(
            name="place_on_shelf",
            area="manipulation",
            method="place_on_shelf",
            summary="Place the held object on a specific shelf level of a cabinet.",
            args=(
                SkillArg("plane_height", type="int"),
                SkillArg("tolerance", type="int", required=False, default=0),
            ),
            requires=(HOLDING,),
            sets=(ARM_FREE,),
            clears=(HOLDING,),
            prior_duration_s=90.0,
            prior_success=0.75,
        ),
        Skill(
            name="place_on_floor",
            area="manipulation",
            method="place_on_floor",
            summary="Put the held object down on the floor.",
            requires=(HOLDING,),
            sets=(ARM_FREE,),
            clears=(HOLDING,),
            prior_duration_s=25.0,
        ),
        Skill(
            name="pour",
            area="manipulation",
            method="pour",
            summary="Pour the held container into a target container.",
            args=(SkillArg("pour_object_name"), SkillArg("pour_container_name")),
            requires=(HOLDING,),
            prior_duration_s=35.0,
            prior_success=0.7,
        ),
        Skill(
            name="open_gripper",
            area="manipulation",
            method="open_gripper",
            summary="Open the gripper.",
            sets=(ARM_FREE,),
            clears=(HOLDING,),
            prior_duration_s=2.0,
            prior_success=0.99,
        ),
        Skill(
            name="close_gripper",
            area="manipulation",
            method="close_gripper",
            summary="Close the gripper.",
            prior_duration_s=2.0,
            prior_success=0.99,
        ),
        Skill(
            name="move_arm_to",
            area="manipulation",
            method="move_to_position",
            summary="Move the arm to a named configuration (e.g. nav_pose).",
            args=(SkillArg("named_position"),),
            prior_duration_s=8.0,
            prior_success=0.95,
        ),
        Skill(
            name="point_at",
            area="manipulation",
            method="point",
            summary="Point the arm at a bearing in degrees.",
            args=(SkillArg("degrees", type="float"),),
            prior_duration_s=6.0,
        ),
        Skill(
            name="go_to_hand",
            area="manipulation",
            method="go_to_hand",
            summary="Move the gripper to a detected hand for a handover.",
            args=(SkillArg("point", type="point"),),
            prior_duration_s=15.0,
        ),
        Skill(
            name="move_arm_vertical",
            area="manipulation",
            method="move_arm_vertical",
            summary="Raise or lower the gripper by a distance in metres.",
            args=(
                SkillArg("distance", type="float"),
                SkillArg("descend", type="bool", required=False, default=False),
            ),
            prior_duration_s=6.0,
            prior_success=0.9,
        ),
        Skill(
            name="follow_face",
            area="manipulation",
            method="follow_face",
            summary="Track a person's face with the arm-mounted camera.",
            args=(SkillArg("follow", type="bool", required=False, default=True),),
            mutates_world=False,
            prior_duration_s=1.0,
            prior_success=0.9,
        ),
        # ---------------- following ----------------
        Skill(
            name="follow_person",
            area="nav",
            method="follow_person",
            summary="Drive after a person until told to stop.",
            args=(SkillArg("follow", type="bool", required=False, default=True),),
            prior_duration_s=120.0,
            prior_success=0.6,
        ),
        # ---------------- interaction ----------------
        Skill(
            name="say",
            area="hri",
            method="say",
            summary="Speak a sentence out loud.",
            args=(
                SkillArg("text"),
                SkillArg("wait", type="bool", required=False, default=True),
            ),
            mutates_world=False,
            prior_duration_s=4.0,
            prior_success=0.98,
        ),
        Skill(
            name="hear",
            area="hri",
            method="hear",
            summary="Listen and transcribe what a person says.",
            mutates_world=False,
            prior_duration_s=8.0,
            prior_success=0.8,
        ),
        Skill(
            name="confirm",
            area="hri",
            method="confirm",
            summary="Ask a yes/no question and return the answer.",
            args=(SkillArg("text"),),
            mutates_world=False,
            prior_duration_s=10.0,
            prior_success=0.8,
        ),
        Skill(
            name="ask_and_confirm",
            area="hri",
            method="ask_and_confirm",
            summary="Ask an open question and confirm the extracted answer.",
            args=(SkillArg("question"),),
            mutates_world=False,
            prior_duration_s=15.0,
            prior_success=0.75,
        ),
        Skill(
            name="answer_question",
            area="hri",
            method="answer_question",
            summary="Answer a question using the knowledge base.",
            mutates_world=False,
            prior_duration_s=12.0,
        ),
        Skill(
            name="categorize_objects",
            area="hri",
            method="categorize_objects",
            summary="Group detected objects onto shelf levels by category.",
            mutates_world=False,
            prior_duration_s=5.0,
        ),
        Skill(
            name="query_location",
            area="hri",
            method="query_location",
            summary="Resolve free text to a named map area.",
            mutates_world=False,
            prior_duration_s=1.0,
            prior_success=0.9,
        ),
        Skill(
            name="show_step",
            area="hri",
            method="publish_display_step",
            summary="Publish the current task step to the operator display.",
            args=(SkillArg("step"),),
            mutates_world=False,
            prior_duration_s=0.1,
            prior_success=0.99,
        ),
    )


REGISTRY: dict[str, Skill] = {skill.name: skill for skill in _skills()}

# guard against a copy-paste duplicate silently shadowing a skill
assert len(REGISTRY) == len(_skills()), "duplicate skill name in registry"


def resolve(subtask_manager: Any, name: str):
    """Return the bound method implementing ``name``, or raise KeyError/AttributeError."""
    skill = REGISTRY[name]
    area = getattr(subtask_manager, skill.area)
    method = getattr(area, skill.method, None)
    if not callable(method):
        raise AttributeError(f"skill '{name}' maps to missing {skill.area}.{skill.method}")
    return method


def call_skill(subtask_manager: Any, name: str, **kwargs: Any):
    """Invoke a registered skill after checking its declared arguments."""
    skill = REGISTRY[name]
    unknown = set(kwargs) - {arg.name for arg in skill.args}
    if unknown:
        raise TypeError(f"skill '{name}' got unexpected argument(s): {sorted(unknown)}")
    missing = [arg.name for arg in skill.args if arg.required and arg.name not in kwargs]
    if missing:
        raise TypeError(f"skill '{name}' missing required argument(s): {missing}")
    return resolve(subtask_manager, name)(**kwargs)


def describe(name: str) -> dict:
    """JSON-serializable description, used to build MCP tool schemas."""
    skill = REGISTRY[name]
    return {
        "name": skill.name,
        "summary": skill.summary,
        "area": skill.area,
        "mutates_world": skill.mutates_world,
        "args": [
            {
                "name": arg.name,
                "type": arg.type,
                "required": arg.required,
                "description": arg.description,
                "default": arg.default,
            }
            for arg in skill.args
        ],
        "requires": list(skill.requires),
        "sets": list(skill.sets),
        "clears": list(skill.clears),
    }


def read_only_skills() -> tuple[str, ...]:
    """Skills safe to expose without an explicit actuation opt-in."""
    return tuple(sorted(name for name, s in REGISTRY.items() if not s.mutates_world))

"""Pick strategy profiles: the tuning data behind each pick behavior.

Profiles are loaded once at node startup from ``config/pick_profiles.yaml`` and
validated eagerly, so a malformed value fails at launch rather than mid-descent
with the arm in cartesian-velocity mode.
"""

import os
from dataclasses import MISSING, dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union, get_args, get_origin

import yaml

STRATEGY_FORCE_GUARDED = "force_guarded"
STRATEGY_FIXED_DISTANCE = "fixed_distance"
STRATEGY_DIRECT = "direct"
STRATEGY_CARTESIAN = "cartesian"

KNOWN_STRATEGY_KINDS = (
    STRATEGY_FORCE_GUARDED,
    STRATEGY_FIXED_DISTANCE,
    STRATEGY_DIRECT,
    STRATEGY_CARTESIAN,
)

PROFILES_FILE_ENV_VAR = "FRIDA_PICK_PROFILES_FILE"

_DEFAULTS_KEY = "defaults"


class ProfileError(ValueError):
    """Raised when the profile file is missing, malformed or inconsistent."""


@dataclass(frozen=True)
class ForceGuardProfile:
    """Contact-detection tuning for the effort-guarded descent.

    Contact is declared when the per-joint effort jump over ``jump_window``
    exceeds ``jump_trip`` for ``jump_sustain`` consecutive samples, or when any
    joint exceeds ``hard_ceiling`` outright.
    """

    descent_speed: float  # mm/s, xArm SDK convention
    timeout: float  # s
    grace_period: float  # s
    contact_settle: float  # s
    min_contact_descent: float  # m
    ignore_joints: Tuple[int, ...]
    jump_window: float  # s
    jump_trip: float  # N
    jump_sustain: int  # consecutive samples
    hard_ceiling: float  # N
    # m along the drive (on the TCP) where it ends without contact; None measures elapsed x speed
    max_descent: Optional[float] = None

    def validate(self, profile_name: str) -> None:
        _require_positive(
            self.descent_speed, f"{profile_name}.force_guard.descent_speed"
        )
        _require_positive(self.timeout, f"{profile_name}.force_guard.timeout")
        _require_positive(self.jump_window, f"{profile_name}.force_guard.jump_window")
        _require_positive(self.jump_trip, f"{profile_name}.force_guard.jump_trip")
        _require_positive(self.hard_ceiling, f"{profile_name}.force_guard.hard_ceiling")
        if self.jump_sustain < 1:
            raise ProfileError(
                f"{profile_name}.force_guard.jump_sustain must be >= 1, got {self.jump_sustain}"
            )
        if self.jump_trip >= self.hard_ceiling:
            raise ProfileError(
                f"{profile_name}.force_guard.jump_trip ({self.jump_trip}) must be below "
                f"hard_ceiling ({self.hard_ceiling}); otherwise the ceiling never fires"
            )
        for joint in self.ignore_joints:
            if joint < 0:
                raise ProfileError(
                    f"{profile_name}.force_guard.ignore_joints has a negative index: {joint}"
                )
        if self.max_descent is not None:
            _require_positive(
                self.max_descent, f"{profile_name}.force_guard.max_descent"
            )
            if self.min_contact_descent >= self.max_descent:
                raise ProfileError(
                    f"{profile_name}.force_guard.min_contact_descent "
                    f"({self.min_contact_descent} m) must be below max_descent "
                    f"({self.max_descent} m), or contact is never honored"
                )
            reachable = self.timeout * self.descent_speed / 1000.0
            if reachable < self.max_descent:
                raise ProfileError(
                    f"{profile_name}.force_guard.timeout ({self.timeout} s) at "
                    f"{self.descent_speed} mm/s only covers {reachable:.3f} m, "
                    f"less than max_descent {self.max_descent} m"
                )


@dataclass(frozen=True)
class ShelfProfile:
    """Shelf numbers, in m and deg: the frontal grasp into a compartment and the way
    out, facing a level, the place inside the board and how far the base may move.

    Fingertip heights are in the base frame: GPD's tilted grasps do not fit a compartment.
    """

    board_clearance: float  # fingertips this far above the board
    ceiling_clearance: float  # ...and this far under the ceiling
    top_margin: float  # never above the object's top minus this
    z_steps: Tuple[float, ...]  # heights tried around the object's middle
    width_margin: float  # the object must fit the open gripper with this to spare
    max_half_depth: float
    past_center: float  # the finger pads sit behind the fingertip point
    pitches: Tuple[float, ...]  # tilt down from horizontal, in order
    open_top_pitches: Tuple[float, ...]  # also tried with nothing above
    exit_lift: float  # straight MoveIt path up off the board...
    exit_retreat: float  # ...then back out along the way in
    view_distance: float  # facing a level for a pick: camera past the board front...
    view_height: float  # ...this high above the board, looking at its centre...
    view_max_reach: float  # ...this far from the centre when farther from the base
    place_view_height: float  # facing a level for a place: camera above the board
    place_front_margin: float  # the released object's centre this far inside the board
    place_back_margin: float
    place_side_margin: float
    max_base_shift: float  # the base may move this far before the scan is stale...
    max_base_turn: float  # ...or turn this much (deg)

    def validate(self, profile_name: str) -> None:
        for key in (
            "board_clearance",
            "ceiling_clearance",
            "exit_lift",
            "exit_retreat",
            "view_distance",
            "max_base_shift",
            "max_base_turn",
        ):
            _require_positive(getattr(self, key), f"{profile_name}.shelf.{key}")
        if not self.pitches:
            raise ProfileError(f"{profile_name}.shelf.pitches must not be empty")


@dataclass(frozen=True)
class PickProfile:
    """Everything that varies between pick behaviors."""

    name: str
    strategy: str
    tip_offset_param: str
    num_alternatives: int
    alternative_step: float
    pre_grasp_height: float
    post_grasp_height: float
    pre_grasp_velocity: float
    close_settle: float
    validate_endpoint: bool
    lift_after_grasp: bool
    attach_collision_object: bool
    grasp_z_tweak: float = 0.0
    descent_speed: float = 0.0
    descent_distance: Optional[float] = None
    post_contact_retract: float = 0.0
    force_guard: Optional[ForceGuardProfile] = None
    fallback_to_gpd: bool = False
    shelf: Optional[ShelfProfile] = None

    @property
    def effective_descent_distance(self) -> float:
        """Distance the fixed-distance descent should travel.

        Rim, bowl and peak all satisfy ``descent = pre_grasp_height -
        grasp_z_tweak``: the arm rises to the pre-grasp, then descends back down
        past it by the tweak. An explicit ``descent_distance`` overrides this.
        """
        if self.descent_distance is not None:
            return self.descent_distance
        return self.pre_grasp_height - self.grasp_z_tweak

    def validate(self) -> None:
        if self.strategy not in KNOWN_STRATEGY_KINDS:
            raise ProfileError(
                f"{self.name}.strategy is '{self.strategy}', expected one of "
                f"{list(KNOWN_STRATEGY_KINDS)}"
            )
        if self.num_alternatives < 1:
            raise ProfileError(
                f"{self.name}.num_alternatives must be >= 1, got {self.num_alternatives}"
            )
        if self.pre_grasp_height < 0.0:
            raise ProfileError(
                f"{self.name}.pre_grasp_height must be >= 0, got {self.pre_grasp_height}"
            )
        _require_positive(self.pre_grasp_velocity, f"{self.name}.pre_grasp_velocity")
        if self.close_settle < 0.0:
            raise ProfileError(
                f"{self.name}.close_settle must be >= 0, got {self.close_settle}"
            )
        if self.post_contact_retract < 0.0:
            raise ProfileError(
                f"{self.name}.post_contact_retract must be >= 0 (it retracts upward), "
                f"got {self.post_contact_retract}"
            )

        if self.strategy in (STRATEGY_FIXED_DISTANCE, STRATEGY_CARTESIAN):
            # A non-positive descent would drive the arm upward into the plan,
            # or not move at all -- both are configuration mistakes.
            if self.effective_descent_distance <= 0.0:
                raise ProfileError(
                    f"{self.name}: descent distance resolves to "
                    f"{self.effective_descent_distance} m (pre_grasp_height "
                    f"{self.pre_grasp_height} - grasp_z_tweak {self.grasp_z_tweak}); "
                    "it must be positive"
                )
            _require_positive(self.descent_speed, f"{self.name}.descent_speed")

        if self.shelf is not None:
            self.shelf.validate(self.name)

        if self.strategy == STRATEGY_FORCE_GUARDED:
            if self.force_guard is None:
                raise ProfileError(
                    f"{self.name}.strategy is '{STRATEGY_FORCE_GUARDED}' but no "
                    "force_guard block was given"
                )
            self.force_guard.validate(self.name)
            # The guarded descent is open-loop on time; the timeout has to cover
            # the full pre-grasp height or a valid contact can be cut short.
            reachable = (
                self.force_guard.timeout * self.force_guard.descent_speed / 1000.0
            )
            if reachable < self.pre_grasp_height:
                raise ProfileError(
                    f"{self.name}: force_guard.timeout ({self.force_guard.timeout} s) at "
                    f"{self.force_guard.descent_speed} mm/s only covers {reachable:.3f} m, "
                    f"less than pre_grasp_height {self.pre_grasp_height} m"
                )
            if self.force_guard.min_contact_descent > self.pre_grasp_height:
                raise ProfileError(
                    f"{self.name}: force_guard.min_contact_descent "
                    f"({self.force_guard.min_contact_descent} m) exceeds pre_grasp_height "
                    f"({self.pre_grasp_height} m), so contact could never be honored"
                )


def _require_positive(value: float, label: str) -> None:
    if value <= 0.0:
        raise ProfileError(f"{label} must be > 0, got {value}")


def default_profiles_path() -> Path:
    """Resolve the profile file: env override, then the installed share dir.

    Mirrors ``xarm_utils.shelf_levels.levels_file`` so operators can point at a
    hand-tuned file without reinstalling the package.
    """
    override = os.environ.get(PROFILES_FILE_ENV_VAR)
    if override:
        return Path(override)
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("pick_and_place"))
        / "config"
        / "pick_profiles.yaml"
    )


def _coerce(raw: Any, field_type: Any, label: str) -> Any:
    """Convierte un valor del YAML al tipo declarado del campo."""
    if get_origin(field_type) is Union:  # Optional[X] -> X
        if raw is None:
            return None
        field_type = next(a for a in get_args(field_type) if a is not type(None))

    if is_dataclass(field_type):  # force_guard
        if not isinstance(raw, dict):
            raise ProfileError(f"{label} must be a mapping, got {raw!r}")
        return _build(field_type, raw, label)

    if get_origin(field_type) is tuple:  # ignore_joints
        if not isinstance(raw, (list, tuple)):
            raise ProfileError(f"{label} must be a list, got {raw!r}")
        item_type = get_args(field_type)[0]
        return tuple(_coerce(item, item_type, label) for item in raw)

    if field_type is bool:
        if not isinstance(raw, bool):
            raise ProfileError(f"{label} must be a boolean, got {raw!r}")
        return raw

    try:
        return field_type(raw)
    except (TypeError, ValueError):
        raise ProfileError(f"{label} must be a {field_type.__name__}, got {raw!r}")


def _build(cls, raw: Dict[str, Any], label: str, **preset):
    """Construye un dataclass de perfil desde un mapping, campo a campo.

    Las claves desconocidas y las faltantes son error: una errata en el YAML debe
    fallar al arrancar, no coger un default en silencio.
    """
    expected = {f.name: f for f in fields(cls) if f.name not in preset}
    unknown = set(raw) - set(expected)
    if unknown:
        raise ProfileError(f"{label} has unknown keys: {sorted(unknown)}")

    values = dict(preset)
    for key, field in expected.items():
        if key in raw:
            values[key] = _coerce(raw[key], field.type, f"{label}.{key}")

    missing = [
        f.name for f in fields(cls) if f.default is MISSING and f.name not in values
    ]
    if missing:
        raise ProfileError(f"{label} is missing required keys: {sorted(missing)}")
    return cls(**values)


def load_profiles(path: Optional[Path] = None) -> Dict[str, PickProfile]:
    """Load and validate every pick profile.

    Raises ProfileError on anything malformed so the node fails at startup.
    """
    path = Path(path) if path is not None else default_profiles_path()
    if not path.is_file():
        raise ProfileError(f"Pick profile file not found: {path}")

    try:
        document = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ProfileError(f"Could not parse {path}: {exc}") from exc

    if not isinstance(document, dict):
        raise ProfileError(f"{path} must contain a mapping of profile name -> settings")

    # The `defaults` anchor is expanded by the YAML parser into each profile;
    # the key itself is not a profile.
    raw_profiles = {
        name: dict(body) for name, body in document.items() if name != _DEFAULTS_KEY
    }
    if not raw_profiles:
        raise ProfileError(f"{path} defines no profiles")
    for name, body in raw_profiles.items():
        if not isinstance(body, dict):
            raise ProfileError(f"Profile '{name}' must be a mapping, got {body!r}")

    profiles = {}
    for name, body in raw_profiles.items():
        profile = _build(PickProfile, body, f"Profile '{name}'", name=name)
        profile.validate()
        profiles[name] = profile
    return profiles

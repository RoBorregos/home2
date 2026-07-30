"""Pick strategy profiles: the tuning data behind each pick behavior.

Profiles are loaded once at node startup from ``config/pick_profiles.yaml`` and
validated eagerly, so a malformed value fails at launch rather than mid-descent
with the arm in cartesian-velocity mode.
"""

import os
from dataclasses import MISSING, dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

STRATEGY_FORCE_GUARDED = "force_guarded"
STRATEGY_FIXED_DISTANCE = "fixed_distance"
STRATEGY_DIRECT = "direct"

KNOWN_STRATEGY_KINDS = (
    STRATEGY_FORCE_GUARDED,
    STRATEGY_FIXED_DISTANCE,
    STRATEGY_DIRECT,
)

PROFILES_FILE_ENV_VAR = "FRIDA_PICK_PROFILES_FILE"
PROFILES_PARAM_PREFIX = "pick_profile"

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


@dataclass(frozen=True)
class PickProfile:
    """Everything that varies between pick behaviors."""

    name: str
    strategy: str
    tip_offset_param: str
    num_alternatives: int
    alternative_step: float
    pre_grasp_height: float
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

        if self.strategy == STRATEGY_FIXED_DISTANCE:
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
    """Coerce a YAML/param scalar into the dataclass field's type.

    YAML gives ints where floats are wanted, and ROS params arrive as strings
    when set from the command line.
    """
    if field_type is bool:
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, str):
            lowered = raw.strip().lower()
            if lowered in ("true", "1", "yes"):
                return True
            if lowered in ("false", "0", "no"):
                return False
        raise ProfileError(f"{label} must be a boolean, got {raw!r}")
    try:
        if field_type is int:
            return int(raw)
        if field_type is float:
            return float(raw)
    except (TypeError, ValueError):
        raise ProfileError(f"{label} must be a {field_type.__name__}, got {raw!r}")
    return raw


def _build_force_guard(raw: Dict[str, Any], profile_name: str) -> ForceGuardProfile:
    expected = {f.name: f for f in fields(ForceGuardProfile)}
    unknown = set(raw) - set(expected)
    if unknown:
        raise ProfileError(
            f"{profile_name}.force_guard has unknown keys: {sorted(unknown)}"
        )
    missing = set(expected) - set(raw)
    if missing:
        raise ProfileError(
            f"{profile_name}.force_guard is missing keys: {sorted(missing)}"
        )

    values: Dict[str, Any] = {}
    for name, field in expected.items():
        label = f"{profile_name}.force_guard.{name}"
        if name == "ignore_joints":
            joints = raw[name]
            if not isinstance(joints, (list, tuple)):
                raise ProfileError(
                    f"{label} must be a list of joint indices, got {joints!r}"
                )
            values[name] = tuple(_coerce(j, int, label) for j in joints)
        else:
            values[name] = _coerce(raw[name], field.type, label)
    return ForceGuardProfile(**values)


def _build_profile(name: str, raw: Dict[str, Any]) -> PickProfile:
    expected = {f.name: f for f in fields(PickProfile) if f.name != "name"}
    unknown = set(raw) - set(expected)
    if unknown:
        raise ProfileError(f"Profile '{name}' has unknown keys: {sorted(unknown)}")

    values: Dict[str, Any] = {"name": name}
    for key, field in expected.items():
        if key not in raw:
            continue
        label = f"{name}.{key}"
        if key == "force_guard":
            block = raw[key]
            if not isinstance(block, dict):
                raise ProfileError(f"{label} must be a mapping, got {block!r}")
            values[key] = _build_force_guard(block, name)
        elif key == "descent_distance":
            values[key] = None if raw[key] is None else _coerce(raw[key], float, label)
        elif key == "strategy" or key == "tip_offset_param":
            values[key] = str(raw[key])
        else:
            values[key] = _coerce(raw[key], field.type, label)

    missing = [
        f.name
        for f in fields(PickProfile)
        if f.name != "name" and f.default is MISSING and f.name not in values
    ]
    if missing:
        raise ProfileError(
            f"Profile '{name}' is missing required keys: {sorted(missing)}"
        )

    return PickProfile(**values)


def _apply_overrides(
    raw_profiles: Dict[str, Dict[str, Any]], overrides: Dict[str, Any]
) -> None:
    """Apply flat ROS-param overrides onto the parsed YAML, in place.

    Keys look like ``pick_profile.flat.pre_grasp_height`` or
    ``pick_profile.flat.force_guard.jump_trip``.
    """
    for key, value in overrides.items():
        parts = key.split(".")
        if parts and parts[0] == PROFILES_PARAM_PREFIX:
            parts = parts[1:]
        if len(parts) < 2:
            raise ProfileError(
                f"Override '{key}' is malformed; expected "
                f"'{PROFILES_PARAM_PREFIX}.<profile>.<field>[.<subfield>]'"
            )
        profile_name, path = parts[0], parts[1:]
        if profile_name not in raw_profiles:
            raise ProfileError(
                f"Override '{key}' targets unknown profile '{profile_name}'; "
                f"known profiles: {sorted(raw_profiles)}"
            )
        target = raw_profiles[profile_name]
        for step in path[:-1]:
            nested = target.get(step)
            if not isinstance(nested, dict):
                raise ProfileError(
                    f"Override '{key}' targets a non-mapping at '{step}'"
                )
            target = nested
        target[path[-1]] = value


def load_profiles(
    path: Optional[Path] = None, overrides: Optional[Dict[str, Any]] = None
) -> Dict[str, PickProfile]:
    """Load, override and validate every pick profile.

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

    if overrides:
        _apply_overrides(raw_profiles, overrides)

    profiles = {}
    for name, body in raw_profiles.items():
        profile = _build_profile(name, body)
        profile.validate()
        profiles[name] = profile
    return profiles

"""Test bootstrap.

The pipeline tests exercise manipulation logic, not ROS transport. Where a
sibling ROS package is not built in the current environment, a stub is
installed so the modules under test can be imported; when the package IS built
(CI, the dev container after a full colcon build) the real module is used and
nothing is stubbed.

Only packages that carry no logic under test are eligible -- the pipelines,
strategies, profiles and the RobotArm/Perception classes themselves are always
the real thing.
"""

import importlib
import sys
import types
from unittest.mock import MagicMock

# Sibling ROS packages that only provide transport helpers to the code under
# test. Each entry is imported for real first; a stub appears only on failure.
_OPTIONAL_MODULES = [
    "frida_motion_planning",
    "frida_motion_planning.utils",
    "frida_motion_planning.utils.ros_utils",
    "frida_motion_planning.utils.service_utils",
    "frida_motion_planning.utils.tf_utils",
    "frida_pymoveit2",
    "frida_pymoveit2.robots",
    "frida_pymoveit2.robots.xarm6",
    "pymoveit2",
    "xarm_msgs",
    "xarm_msgs.msg",
    "xarm_msgs.srv",
    "xarm_utils",
    "xarm_utils.shelf_levels",
]

# Values a stub must provide concretely, because the code destructures them.
_STUB_ATTRIBUTES = {
    "frida_pymoveit2.robots.xarm6": {
        "JOINT_POSITION_LIMITS": {f"joint{i}": (-6.28, 6.28) for i in range(1, 7)},
    },
}

_STUBBED = []


def _install_stubs():
    for name in _OPTIONAL_MODULES:
        try:
            importlib.import_module(name)
            continue
        except Exception:
            pass

        module = types.ModuleType(name)
        module.__getattr__ = lambda attribute: MagicMock()  # noqa: ARG005
        for attribute, value in _STUB_ATTRIBUTES.get(name, {}).items():
            setattr(module, attribute, value)
        sys.modules[name] = module
        _STUBBED.append(name)

        # Attach to the parent so `from pkg.sub import x` resolves.
        if "." in name:
            parent, _, child = name.rpartition(".")
            if parent in sys.modules:
                setattr(sys.modules[parent], child, module)


_install_stubs()


def pytest_report_header(config):
    if not _STUBBED:
        return "manipulation deps: all real (nothing stubbed)"
    return (
        f"manipulation deps: stubbed {len(_STUBBED)} unbuilt module(s) "
        f"-> {', '.join(sorted(set(_STUBBED)))}"
    )

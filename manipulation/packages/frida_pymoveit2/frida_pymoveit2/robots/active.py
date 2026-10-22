"""Arm description selected by the FRIDA_ARM env var: xarm6 (default) or a2."""

import os

if os.environ.get("FRIDA_ARM", "xarm6") == "a2":
    from frida_pymoveit2.robots.a2 import *  # noqa: F401,F403
else:
    from frida_pymoveit2.robots.xarm6 import *  # noqa: F401,F403

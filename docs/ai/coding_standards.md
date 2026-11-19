# Coding Standards & Conventions

To maintain a high-quality, maintainable codebase, all contributions should adhere to the following standards.

## ROS 2 Node Structure (Python)

When writing Python nodes, follow the modular structure demonstrated in `docs/ROS2.md`:

- **Inheritance**: Always inherit from `rclpy.node.Node`.
- **Initialization**: Set up publishers, subscribers, services, and timers within the `__init__` method.
- **Callbacks**: Keep callback logic concise. Delegate complex processing to helper methods.
- **Logger**: Use the built-in node logger (`self.get_logger()`) instead of `print()`.

### Example Node Pattern

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node_name")
        self.get_logger().info("Node initialized")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Interface Usage

Always prefer using the custom messages, services, and actions defined in the **`frida_interfaces`** package for robot-specific interactions. Reference `docs/interfaces.md` for existing definitions.

## Tooling & Quality Control

### Formatting
- **Ruff**: This project uses `ruff` for Python linting and formatting. Ensure your code passes all ruff checks.
- **Pre-commit Hooks**: Always run `pre-commit install` to set up automated checks (ruff, linting, etc.) before committing.

### Linting Scope — not repo-wide
- `ruff.toml` explicitly **excludes** `navigation/`, `robot_description/`, and the `manipulation/` git submodules from linting. Code in those areas may not follow the same formatting rules as the rest of the repo — don't assume a failing/passing lint check there means the same thing it does elsewhere.
- `.pre-commit-config.yaml` applies **different rules to `task_manager/`** than to the rest of the repo. Check the config directly before assuming one global rule set applies everywhere.

### Type Hints
- Use Python type hints wherever possible to improve code clarity and enable better IDE/AI assistance.

### Documentation
- Provide docstrings for all classes and public methods following a consistent format (e.g., Google or NumPy style).

## Testing

- **CI does not run automated tests.** The `ros2-build.yml` workflow only runs `colcon build --symlink-install`; it does not execute `colcon test` or `pytest`.
- Real tests live in `task_manager/scripts/test/test_*.py`. These are **executable scripts** (run manually via `ros2 run`), not an automated pytest suite that runs on its own.
- Never state that "tests pass" or that something "is tested" without having actually run the relevant script yourself and confirmed the result. A green CI build only means the code compiles — it says nothing about correctness.

## Script Hygiene — `task_manager/scripts/misc/`

- `task_manager`'s `CMakeLists.txt` installs executables via `file(GLOB MISC_NODES scripts/misc/*.py)` — **every** `.py` file placed in that folder gets compiled and installed automatically, whether it's used or not.
- A successful build is **not** evidence that a file in `scripts/misc/` is in use. Check launch files and imports directly before trusting a file's presence.
- Never leave copies, backups, or versioned variants of a script in `scripts/misc/` (e.g. `foo copy.py`, `foo_v2.py`, `temp_foo.py`). Delete experimental or superseded files before finishing a task/PR, or move them outside `scripts/misc/` if they need to be kept for reference.

## Third-Party / Submodule Code

Several packages under `manipulation/` and `navigation/` are git submodules pointing to external or forked repositories. Do not refactor this code to match internal team conventions (naming, structure, docstring style, etc.) without confirming with the maintainers first — this is also why these paths are excluded from linting.

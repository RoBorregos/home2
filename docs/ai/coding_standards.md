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

### Type Hints
- Use Python type hints wherever possible to improve code clarity and enable better IDE/AI assistance.

### Documentation
- Provide docstrings for all classes and public methods following a consistent format (e.g., Google or NumPy style).

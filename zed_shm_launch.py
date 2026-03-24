"""Patch the ZED launch file to disable parameter_events for ALL nodes.

Prevents iceoryx TOO_MANY_CHUNKS_HELD_IN_PARALLEL errors that block
TF delivery when CycloneDDS SHM is enabled.

For composable nodes, parameters set via YAML are applied AFTER node
construction, so start_parameter_event_publisher has no effect. Instead,
we inject it into the container's --ros-args which ARE processed during
node initialization.

Run this BEFORE `ros2 launch zed_wrapper zed_camera.launch.py`.
"""


def patch_zed_launch():
    launch_path = (
        "/opt/zed_ws/install/zed_wrapper/share/zed_wrapper/launch"
        "/zed_camera.launch.py"
    )

    with open(launch_path) as f:
        content = f.read()

    # Already patched?
    if "start_parameter_event_publisher" in content:
        return

    # 1) Patch robot_state_publisher Node: add parameter
    content = content.replace(
        "'robot_description': Command(xacro_command)\n        }]",
        "'robot_description': Command(xacro_command),\n"
        "            'start_parameter_event_publisher': False\n"
        "        }]",
    )

    # 2) Patch ComposableNodeContainer arguments: inject --ros-args -p
    #    This is processed during node construction, BEFORE parameters are loaded
    content = content.replace(
        "arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info']",
        "arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info', "
        "'-p', 'start_parameter_event_publisher:=false']",
    )

    with open(launch_path, "w") as f:
        f.write(content)

    print("[SHM] Patched ZED launch: disabled parameter_events for all nodes")


if __name__ == "__main__":
    patch_zed_launch()

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # command_interpreter_config = os.path.join(
    #     get_package_share_directory(
    #         "nlp"), "config", "command_interpreter.yaml"
    # )

    return LaunchDescription(
        [
            Node(
                package="task_manager",
                executable="demo_task_manager.py",
                name="demo_task_manager",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

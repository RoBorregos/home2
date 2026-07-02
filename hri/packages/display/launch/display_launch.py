from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():
    task_arg = DeclareLaunchArgument(
        "task",
        default_value="default",
        description=(
            "Display view to show: default, gpsr, hric, laundry, ppc, "
            "restaurant or storing_groceries"
        ),
    )

    video_topic_arg = DeclareLaunchArgument(
        "default_video_topic",
        default_value="",
        description="Initial video topic; empty uses the task's default",
    )

    display_ui_node = launch_ros.actions.Node(
        package="display",
        executable="display_ui.py",
        name="display_ui",
        output="screen",
        parameters=[
            {
                "task": LaunchConfiguration("task"),
                "default_video_topic": LaunchConfiguration("default_video_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            task_arg,
            video_topic_arg,
            display_ui_node,
        ]
    )

from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch import LaunchDescription


def launch_setup(context, *args, **kwargs):
    task_manager = Node(
        package="task_manager",
        executable="doing_laundry_task_manager.py",
        output="screen",
    )

    return_launch = [task_manager]
    return return_launch


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

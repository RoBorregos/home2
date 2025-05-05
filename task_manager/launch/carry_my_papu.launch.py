
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch import LaunchDescription


def launch_setup(context, *args, **kwargs):
    task_manager = Node(
            package='task_manager',
            executable='help_me_carry.py',
            output='screen',
    )
    follow_person = Node(
            package='task_manager',
            executable='follow_person_node.py',
            output='screen'
    )

    return_launch = [
        task_manager,
        follow_person
    ]
    return return_launch

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
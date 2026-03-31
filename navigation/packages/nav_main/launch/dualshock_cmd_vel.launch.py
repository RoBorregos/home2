
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description."""
    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value=TextSubstitution(text='/cmd_vel'))

    joy_container = ComposableNodeContainer(
        name='joy_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                namespace='',
            ),
            ComposableNode(
                package='nav_main',
                plugin='nav_main::ControllerDualshock',
                name='controller_dualshock',
                namespace='',
                parameters=[{
                    'linear_speed': 0.2,
                    'rotation_speed': 0.1,
                    'deadzone': 0.8,
                }],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )


    ld = LaunchDescription()

    ld.add_action(topic_name_arg)
    ld.add_action(joy_container)

    return ld

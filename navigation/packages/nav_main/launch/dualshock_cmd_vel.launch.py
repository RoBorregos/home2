
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description."""
    hw_type_arg = DeclareLaunchArgument(
        'hw_type', default_value=TextSubstitution(text='DualShock4'))
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
                package='p9n_node',
                plugin='p9n_node::TeleopTwistJoyNode',
                name='teleop_twist_joy_node',
                namespace='',
                parameters=[{
                        'hw_type': LaunchConfiguration('hw_type')
                }],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )


    ld = LaunchDescription()

    ld.add_action(hw_type_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(joy_container)

    return ld

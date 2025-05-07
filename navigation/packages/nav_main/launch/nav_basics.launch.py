
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import PythonExpression
from launch import LaunchDescription
import os
def launch_setup(context, *args, **kwargs):
    config_topics = os.path.join(get_package_share_directory('nav_main'),
                                         'config', 'tmux_config.yaml')
    publish_urdf = LaunchConfiguration('publish_tf', default='false')
    use_sim = LaunchConfiguration('use_sim', default='false')
    use_dualshock = LaunchConfiguration('use_dualshock', default='true')
    cmd_topic = LaunchConfiguration('cmd_topic', default='/cmd_vel')
    cmd_topic_joy = LaunchConfiguration('cmd_topic_joy', default='/cmd_vel_joy')

    dashgo_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver"),
                    "launch",
                    "dashgo_driver.launch.py",
                ]
            )
            ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'cmd_topic': cmd_topic
        }.items(),
        )
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver"),
                    "launch",
                    "ekf.launch.py",
                ]
            )
        ), 
        )
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("urdf_launch"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments={
            'urdf_package': 'frida_description',
            'urdf_package_path': PathJoinSubstitution(['urdf','TMR2025','FRIDA_Real.urdf.xacro'])
        }.items(),
    )

    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "rplidar_fixed.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_sim)
        )
    
    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    dualshock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "dualshock_cmd_vel.launch.py",
                ]
            ),
            
        ),
        launch_arguments={
            'topic_name': cmd_topic_joy
            }.items(),
        condition=IfCondition(use_dualshock)
        )
   
    tmux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings=[
                ('/cmd_vel_out', cmd_topic),
            ],
            parameters=[config_topics],
    )
    emergency_node = Node(
            package='nav_main',
            executable='virtual_stop_button.py',
            condition=IfCondition(use_dualshock)
    )

    if(publish_urdf.perform(context) == 'true' and use_sim.perform(context) == 'false'):
        print("entre papu")
        return_launch = [
        dashgo_driver,
        ekf_launch,
        robot_description_launch,
        joint_state,
        laser_launch,
        # dualshock_launch,
        # tmux_node,
        # emergency_node,
        
    ]
    else:
        print("no entre papu")
        return_launch = [
        dashgo_driver,
        ekf_launch,
        laser_launch,
        # dualshock_launch,
        # tmux_node,
        # emergency_node,
        
    ]
    return return_launch

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
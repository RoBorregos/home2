#!/usr/bin/env python3
"""SLAM (mapping) for the mecanum omnibase — slam_toolbox online async.

What this brings up (run on the ROBOT computer):
    slam_toolbox (async)  ──>  TF map->odom  +  /map  (occupancy grid)
    [optional] ekf        ──>  TF odom->base_link  +  /odometry/filtered
    [optional] dashboard  ──>  /odrive/odom, /odrive/imu  (serial bridge)

NOT started here (you already run it on the robot):
    * the RPLIDAR C1 driver, publishing /scan in the base_link frame.

TF chain SLAM requires:  map -> odom -> base_link -> <scan frame>
    map  -> odom        : slam_toolbox (this launch)
    odom -> base_link   : the EKF (use_ekf:=true, default)
    base_link -> scan   : identity (/scan is published in base_link)

Typical use
-----------
Everything except the lidar, in one command:
    ros2 launch odrive_comm slam.launch.py use_dashboard:=true

If you already run the dashboard + EKF separately, just add SLAM:
    ros2 launch odrive_comm slam.launch.py use_ekf:=false

Override the tuning file:
    ros2 launch odrive_comm slam.launch.py params_file:=/path/to/my_params.yaml

Saving the map (while this is running) — slam_toolbox does it natively, no nav2 needed.
Use an ABSOLUTE path on the ROBOT computer and create the directory first
(`mkdir -p ~/omnibase_maps`); service calls do not expand ~ or $HOME:
    # occupancy grid for navigation (writes arena.pgm + arena.yaml):
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/<user>/omnibase_maps/arena'}}"
    # pose-graph for resuming/localization later (writes arena.posegraph + arena.data):
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/<user>/omnibase_maps/arena'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('odrive_comm')
    default_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    use_ekf = LaunchConfiguration('use_ekf')
    use_dashboard = LaunchConfiguration('use_dashboard')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Also start the EKF (ekf.launch.py), which provides the odom->base_link TF '
                    'that SLAM needs. Set false if you already run the EKF separately.')
    declare_use_dashboard = DeclareLaunchArgument(
        'use_dashboard', default_value='true',
        description='Forwarded to ekf.launch.py: also start odrive_dashboard (serial bridge + web GUI). '
                    'Only one process may open /dev/ttyACM0.')
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Forwarded to ekf.launch.py: topic the dashboard listens on for velocity. '
                    'Keep "cmd_vel" while mapping by teleop.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock instead of wall time (true only when replaying a rosbag in sim time).')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox parameter YAML.')

    # Bring up the EKF (odom->base_link). Reuses this package's ekf.launch.py and
    # passes use_dashboard straight through.
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('odrive_comm'), 'launch', 'ekf.launch.py'])),
        condition=IfCondition(use_ekf),
        launch_arguments={
            'use_dashboard': use_dashboard,
            'cmd_vel_topic': cmd_vel_topic,
        }.items(),
    )

    # slam_toolbox online-async mapper: publishes map->odom + /map.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        respawn=True,            # survive a crash mid-round
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_ekf,
        declare_use_dashboard,
        declare_cmd_vel_topic,
        declare_use_sim_time,
        declare_params_file,
        ekf_launch,
        slam_node,
    ])

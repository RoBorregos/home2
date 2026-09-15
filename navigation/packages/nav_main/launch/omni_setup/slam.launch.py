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

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_main')
    default_params = os.path.join(pkg_share, 'config','omni_config', 'mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock instead of wall time (true only when replaying a rosbag in sim time).')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox parameter YAML.')
    # slam_toolbox online-async mapper: publishes map->odom + /map.
    # It's a lifecycle node — it does nothing (no /scan subscription, no /map
    # publisher) until driven through configure -> activate. The events below
    # do that automatically as soon as the process starts, so this behaves
    # like a normal always-on node instead of requiring an external lifecycle
    # manager (nav2's, or a manual `ros2 lifecycle set` call).
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        emulate_tty=True,
        respawn=True,            # survive a crash mid-round
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        slam_node,
        configure_on_start,
        activate_on_configured,
    ])

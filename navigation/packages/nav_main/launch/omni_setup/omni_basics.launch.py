#!/usr/bin/env python3
"""Navigation basics for omnibase.

Data flow:
    odrive_dashboard  ──>  /odrive/odom  (nav_msgs/Odometry, body-frame vx/vy)
                      ──>  /odrive/imu   (sensor_msgs/Imu, BNO085 yaw + yaw-rate)
                      ──>  TF odom->base_link  (the on-MCU EKF pose, publish_tf)
    lidar setup ----> /scan1 , /scan2 - > /scan

The dashboard is the SOLE publisher of odom->base_link. robot_localization is
configured below but deliberately NOT launched — see the ekf_node block.

Prerequisites:
  * odrive_dashboard must be running (it owns /dev/ttyACM0 and publishes the two
    inputs above). Pass use_dashboard:=true to start it from here.
  * The dashboard publishes /odrive/odom twist in the base_link frame (it
    un-rotates the firmware's world-frame ODOM_vx/ODOM_vy). If the EKF is ever
    re-enabled it rotates that body-frame twist by the IMU heading, so it must
    NOT be world-frame here or it gets rotated twice.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Topic the dashboard listens on for velocity commands. Keep '
                    '"cmd_vel" for teleop/mapping. Under nav2 with the '
                    'velocity_smoother, set "cmd_vel_smoothed" so the base follows '
                    'the smoothed output instead of the raw controller command.')


    lidar_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav_main'), 'launch', 'omni_setup', 'lidar_setup.launch.py'])),
        # launch_arguments={
        #     'use_dashboard': use_dashboard,
        #     'cmd_vel_topic': cmd_vel_topic,
        # }.items(),
    )


    # NOT LAUNCHED: left out of the LaunchDescription below (24a57a3e6) — the
    # dashboard owns odom->base_link. To re-enable, add it back AND set
    # 'publish_tf': False on dashboard_node, or both fight over the same TF.
    # Fuses body-frame vx/vy only (wheels slip); IMU owns heading.
    # robot_localization state-vector layout for the *_config arrays (15 values):
    #   X      Y      Z
    #   roll   pitch  yaw
    #   vx     vy     vz
    #   vroll  vpitch vyaw
    #   ax     ay     az

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        emulate_tty=True,
        respawn=True,            # survive a crash mid-round
        respawn_delay=2.0,
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.2,   # telemetry is ~25-35 Hz; 0.2 s tolerates a few dropped frames
            'two_d_mode': True,      # planar base: zero Z / roll / pitch
            'publish_tf': True,      # inert unless this node is re-added (see above)

            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',   # estimate odom -> base_link only (no map yet)

            # --- Wheel odometry (mecanum): fuse body-frame vx, vy ONLY.
            # vx (index 6) and vy (index 7) are True; everything else False.
            # Wheel-derived x/y/yaw drift from slip, so they are not fused.
            'odom0': '/odrive/odom',
            'odom0_config': [False, False, False,
                             False, False, False,
                             True,  True,  False,
                             False, False, False,
                             False, False, False],
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_queue_size': 10,

            # --- IMU: owns heading. Fuse absolute yaw (index 5) + yaw rate
            # (index 11). imu0_relative zeroes the start heading (no magnetometer
            # north reference needed). Acceleration is NOT fused (noisy, and the
            # BNO085 already removes gravity).
            'imu0': '/odrive/imu',
            'imu0_config': [False, False, False,
                            False, False, True,
                            False, False, False,
                            False, False, True,
                            False, False, False],
            'imu0_differential': False,
            'imu0_relative': True,
            'imu0_queue_size': 10,
            'imu0_remove_gravitational_acceleration': True,
        }],
    )

    dashboard_node = Node(
        package='omnidriver',
        executable='odrive_dashboard',
        name='odrive_dashboard_node',
        output='screen',
        # Nav2 1.4.0 publishes geometry_msgs/TwistStamped on cmd_vel
        # (enable_stamped_cmd_vel defaults true), so subscribe stamped.
        parameters=[{'use_stamped_cmd_vel': True}],
        remappings=[('cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription([
        declare_cmd_vel_topic,
        dashboard_node,
        lidar_setup,
    ])

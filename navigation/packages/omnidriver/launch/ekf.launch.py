#!/usr/bin/env python3
"""robot_localization EKF for the mecanum omnibase.

Data flow:
    odrive_dashboard  ──>  /odrive/odom  (nav_msgs/Odometry, body-frame vx/vy)
                      ──>  /odrive/imu   (sensor_msgs/Imu, BNO085 yaw + yaw-rate)
                                │
                                ▼
                        ekf_node (this launch)  ──>  /odometry/filtered  +  TF odom->base_link

Fusion strategy for a mecanum base:
  * Wheels are trusted ONLY for body-frame linear velocity (vx, vy). They slip,
    so we never fuse wheel x/y/yaw.
  * The BNO085 IMU OWNS heading: we fuse absolute yaw + yaw rate, which are
    immune to wheel slip.

Prerequisites:
  * odrive_dashboard must be running (it owns /dev/ttyACM0 and publishes the two
    inputs above). Pass use_dashboard:=true to start it from here.
  * The dashboard publishes /odrive/odom twist in the base_link frame (it
    un-rotates the firmware's world-frame ODOM_vx/ODOM_vy). robot_localization
    rotates that body-frame twist by the IMU heading, so it must NOT be
    world-frame here or it gets rotated twice.

Usage:
    ros2 launch odrive_comm ekf.launch.py
    ros2 launch odrive_comm ekf.launch.py use_dashboard:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_dashboard = LaunchConfiguration('use_dashboard')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    declare_use_dashboard = DeclareLaunchArgument(
        'use_dashboard', default_value='false',
        description='Also start the odrive_dashboard node (serial bridge + web GUI). '
                    'Leave false if you already run it separately — only one '
                    'process may open /dev/ttyACM0.')

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Topic the dashboard listens on for velocity commands. Keep '
                    '"cmd_vel" for teleop/mapping. Under nav2 with the '
                    'velocity_smoother, set "cmd_vel_smoothed" so the base follows '
                    'the smoothed output instead of the raw controller command.')

    # The frame_id the /odrive/imu message uses (matches imu_frame_id in the
    # dashboard). robot_localization needs base_link -> imu_link in the TF tree.
    imu_frame = 'imu_link'

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
            'publish_tf': True,      # publishes odom -> base_link (do NOT also run odom_to_tf.py)

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

    # base_link -> imu_link static transform (IDENTITY -- keep it this way).
    # The BNO085's inverted yaw is corrected at the SOURCE (odrive_dashboard
    # negates yaw + yaw-rate), NOT here. A 180-deg roll in this TF does NOT work:
    # with two_d_mode the filter flattens roll/pitch to zero, and a 180-deg
    # rolled frame then collapses the fused yaw to 0 (gimbal-lock ambiguity).
    # Only change x/y/z here if the IMU is physically translated from base_link.
    # Optional: start the serial bridge + web dashboard from here too.
    dashboard_node = Node(
        package='odrive_comm',
        executable='odrive_dashboard',
        name='odrive_dashboard_node',
        output='screen',
        condition=IfCondition(use_dashboard),
        remappings=[('cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription([
        declare_use_dashboard,
        declare_cmd_vel_topic,
        dashboard_node,
        ekf_node,
    ])

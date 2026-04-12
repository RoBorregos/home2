
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def launch_setup(context, *args, **kwargs):
    use_dualshock = LaunchConfiguration('use_dualshock', default='true')
    log_output = 'own_log' if os.getenv('NAV_QUIET') == '1' else 'screen'

    dashgo_config = os.path.join(
        get_package_share_directory('dashgo_driver_cpp'),
        'config',
        'dashgo_params.yaml'
    )

    dashgo_driver = Node(
        package='dashgo_driver_cpp',
        executable='dashgo_driver_cpp',
        name='DashgoDriver',
        output=log_output,
        emulate_tty=True,
        parameters=[dashgo_config],
        remappings=[('/cmd_vel', LaunchConfiguration('cmd_topic', default='/cmd_vel'))],
        respawn=True,
        respawn_delay=2.0,
    )

    ekf_launch = Node(
        package='robot_localization',
        executable='ekf_node',
        output=log_output,
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        remappings=[('odometry/filtered', '/odom')],
        parameters=[{
                'output_frame': 'odom',
                'frequency': 20.0,
                'sensor_timeout': 0.2,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'odom0': 'dashgo_odom',
                'imu0': 'imu',
                #  x, y, z 
                # roll, pitch, yaw 
                #  vx, vy, vz 
                # vroll, vpitch, vyaw
                # ax, ay, az
                'odom0_config': [False, False, False,
                                False, False, False,
                                True, False, False,
                                False, False, True,
                                False, False, False],
                'imu0_config': [False, False, False,
                                 False, False, True,
                                 False, False, False,
                                 False, False, True,
                                 False, False, False],
                'imu0_differential': False,

             }],
    )

    laser_launch = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSBlidar2',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': True,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'ignore_array': '-139.9, -122.4, -86.4, -78.4, -40.8, -30.3, 0, 21.8, 30.8, 40.3, 78.4, 93.9, 120.9, 138.4',
            'min_range': 0.12,
        }],
        output=log_output,
        respawn=True,
        respawn_delay=2.0,
    )

    dualshock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "dualshock_cmd_vel.launch.py",
                ],
            ),
        ),
        condition=IfCondition(use_dualshock),
    )

    # Shutdown handler for each critical node — triggers after max respawn retries
    def make_shutdown_handler(node, name):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    LogInfo(msg=f"{name} exited after max respawn retries. Shutting down."),
                    EmitEvent(event=Shutdown(reason=f"{name} failed after 5 retries.")),
                ]
            )
        )

    return_launch = [
        make_shutdown_handler(dashgo_driver, "DashgoDriver"),
        make_shutdown_handler(laser_launch, "Laser"),
        dashgo_driver,
        ekf_launch,
        laser_launch,
        dualshock_launch,
    ]
    return return_launch

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

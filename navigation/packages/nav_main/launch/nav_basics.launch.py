
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
def launch_setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration('use_sim', default='false')
    use_dualshock = LaunchConfiguration('use_dualshock', default='true')

    dashgo_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver_cpp"),
                    "launch",
                    "dashgo_driver_cpp.launch.py",
                ]
            )
            ),
        condition=UnlessCondition(use_sim),
        )
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver_cpp"),
                    "launch",
                    "ekf.launch.py",
                ]
            )
        ), 
        )
    
    # Static transforms from URDF (lidar.xacro) — avoids needing xacro/robot_state_publisher
    # base_link -> laser: xyz="0.16 0 0.002" rpy="0 0 3.14"
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_broadcaster',
        arguments=['0.16', '0', '0.002', '3.14', '0', '0', 'base_link', 'laser'],
    )
    # base_link -> imu_base: xyz="0.15026 0 0.002" rpy="3.1416 3.1416 1.57"
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_broadcaster',
        arguments=['0.15026', '0', '0.002', '1.57', '3.1416', '3.1416', 'base_link', 'imu_base'],
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
        output='screen',
        condition=UnlessCondition(use_sim)
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

    # Event handler: shut down the entire launch if any node exits with an error
    shutdown_on_failure = RegisterEventHandler(
        OnProcessExit(
            on_exit=[
                LogInfo(msg="A navigation node has exited. Checking return code..."),
                EmitEvent(event=Shutdown(reason="A critical navigation node exited unexpectedly.")),
            ]
        )
    )

    return_launch = [
        shutdown_on_failure,
        dashgo_driver,
        ekf_launch,
        laser_tf,
        imu_tf,
        laser_launch,
        dualshock_launch,
    ]
    return return_launch

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
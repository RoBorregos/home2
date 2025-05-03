import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('frida_description'),
        'urdf', 'TMR2025', 'bowl.xacro'
    )

    # Process the xacro file into URDF
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    # Write the processed URDF to a temporary file
    urdf_file = '/tmp/bowl.urdf'
    with open(urdf_file, 'w') as f:
        f.write(robot_description)

    # Path to Gazebo world file (modify as needed)
    gazebo_world_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'worlds', 'empty.world'  # Modify to your world if needed
    )

    return LaunchDescription([
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'simple_object',
                '-file', urdf_file,  # Pass the URDF file to spawn
                "-x", "0.0",  # X position
                "-y", "-1.35",  # Y position
                "-z", "2.0",  # Z position
                "-R", "0",    # Roll
                "-P", "0",    # Pitch
                "-Y", "0"     # Yaw
            ],
        ),
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Base directory for xacro files
    xacro_dir = os.path.join(
        get_package_share_directory('frida_description'),
        'urdf', 'TMR2025'
    )

    # Paths to xacro files
    bowl_xacro_file = os.path.join(xacro_dir, 'bowl.xacro')
    house_xacro_file = os.path.join(xacro_dir, 'house.xacro')

    # Process bowl.xacro
    bowl_doc = xacro.process_file(bowl_xacro_file)
    bowl_urdf = bowl_doc.toxml()
    bowl_urdf_path = '/tmp/bowl.urdf'
    with open(bowl_urdf_path, 'w') as f:
        f.write(bowl_urdf)

    # Process house.xacro
    house_doc = xacro.process_file(house_xacro_file)
    house_urdf = house_doc.toxml()
    house_urdf_path = '/tmp/house.urdf'
    with open(house_urdf_path, 'w') as f:
        f.write(house_urdf)

    return LaunchDescription([
        # Spawn bowl
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_bowl',
            output='screen',
            arguments=[
                '-entity', 'bowl',
                '-file', bowl_urdf_path,
                "-x", "0.0",
                "-y", "-1.35",
                "-z", "2.0",
                "-R", "0",
                "-P", "0",
                "-Y", "0"
            ],
        ),

        # Spawn house
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_house',
            output='screen',
            arguments=[
                '-entity', 'house',
                '-file', house_urdf_path,
                "-x", "1.0",  # adjust position as needed
                "-y", "0.0",
                "-z", "0.0",
                "-R", "0",
                "-P", "0",
                "-Y", "0"
            ],
        ),
    ])


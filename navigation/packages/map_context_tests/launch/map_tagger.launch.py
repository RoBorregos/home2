from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = Path(get_package_share_directory("map_context_tests")) / "rviz" / "map_tagger_home.rviz"

    print(rviz_config_path)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', str(rviz_config_path)],
    )

    map_tagger_terminal = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'ros2', 'run', 'map_context_tests', 'map_tagger'
        ],
        shell=False
    )


    return LaunchDescription([
      rviz_node,
      map_tagger_terminal         
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_function(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route, 'config', 'rtabmap', 'rtabmap_mapping_config.yaml')
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    
    #Values to select base
    default_base = LaunchConfiguration('default_base', default='omnibase') # Other option "dashgo"
    default_base_value = default_base.perform(context)
    nav_type = LaunchConfiguration('nav_type', default='2d') # Other 3d
    nav_type_value = nav_type.perform(context) 
    # Default to a dedicated mapping DB — never reuse the navigatiodken database.
    # The mapping config sets delete_db_on_start=true so each session starts clean.
    # Override with: map_name:=mymap.db
    rtabmap_map_name = LaunchConfiguration('map_name', default='mapping_session.db')
    
    nav_central_node = Node(
        package='nav_main',
        executable='nav_central.py',
        name='nav_central',
        namespace='',
        output='screen',
        parameters=[{
            'mapping': True,
            'localization': False,
            'map_name': rtabmap_map_name,
            'rtab_mapping_config': rtab_params,
            'rtab_localization_config': rtab_params,
            'default_base': default_base,
            'nav_type': nav_type,
        }],
    )

    nav_ui_node = Node(
        package='map_context',
        executable='nav_ui.py',
        name='nav_ui',
        output='screen',
        parameters=[{
            'mode': 'mapping',
            'map_name': rtabmap_map_name,
            'default_base': default_base,
            'nav_type': nav_type,
        }],
    )

    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch","dashgo_base", "nav_basics.launch.py"])
        ),
    
    )

    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "dashgo_base", "rtabnav2.launch.py"])
        ),
        launch_arguments={
            'localization': 'false',
            'rtab_config_file': rtab_params,
            'nav2': 'false',
            'map_name': rtabmap_map_name,
        }.items(),
    )

    omni_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch","omni_setup", "omni_basics.launch.py"])
        ),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch","omni_setup", "slam.launch.py"])
        ),
    )

    launch_actions = [
        nav_central_node,
        nav_ui_node,
    ]

    # When running on the omnibase, nav_basics and rtabmapnav are provided
    # elsewhere (omnibase bringup), so they must not start here.
    if default_base_value != 'omnibase':
        launch_actions.append(nav_basics)
        launch_actions.append(rtabmapnav)
    else:
        launch_actions.append(omni_basics)
        if nav_type_value == "2d":
            launch_actions.append(slam_toolbox)
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])

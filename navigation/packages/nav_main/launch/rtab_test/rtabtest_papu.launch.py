from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_default_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')

    rtab_params_ = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    # docking = LaunchConfiguration('use_docking', default='false')
    rtabmap_map_name = LaunchConfiguration('map_name', default='rtabmap_robocupmuerte.db')
    

    # nav2_params = ParameterFile(nav2_params_, allow_substs=True)
    map_substitution  = {'database_path': ['/workspace/src/navigation/rtabmapdbs/', rtabmap_map_name]}
    rtab_params = ParameterFile(
        RewrittenYaml(
            source_file=rtab_params_,
            root_key='',
            param_rewrites=map_substitution,
            convert_types=True),
        allow_substs=True)


    remappings=[
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return_list = [
         Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtab_params],
	        arguments=["-d"]
            ),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtab_params,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}]),
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[rtab_params],
            remappings=remappings),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtab_params],
            remappings=remappings),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "nav2_composition.launch.py",
                ]
            )),
        launch_arguments={'nav2_config_file': nav2_params}.items(),
        condition=IfCondition(nav2_activate)
        )
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
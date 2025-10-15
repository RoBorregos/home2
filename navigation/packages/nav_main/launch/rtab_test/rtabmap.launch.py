
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_default_config.yaml')
    localization = LaunchConfiguration('localization', default='true')
    rtabmap_map_name = LaunchConfiguration('map_name', default='rtabmap_robocuptesting.db')
    rtab_params_ = LaunchConfiguration('rtab_config_file', default=rtab_params_file)

    map_substitution  = {'database_path': ['/workspace/src/navigation/rtabmapdbs/', rtabmap_map_name]}
    rtab_params = ParameterFile(
        RewrittenYaml(
            source_file=rtab_params_,
            root_key='',
            param_rewrites=map_substitution,
            convert_types=True),
        allow_substs=True)


    return_list = [
        # SLAM Mode:
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
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
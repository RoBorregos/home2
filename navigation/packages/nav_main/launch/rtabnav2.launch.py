from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_default_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')

    # Disable parameter event publisher to avoid iceoryx TOO_MANY_CHUNKS_HELD_IN_PARALLEL
    # and reduce total publisher count to stay under RouDi's 512 publisher port limit
    shm_params = {'start_parameter_event_publisher': False}

    rtab_params_ = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params_ = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='false')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    docking = LaunchConfiguration('use_docking', default='false')
    rtabmap_map_name = LaunchConfiguration('map_name', default='rtabmap_map.db')

    nav2_params = ParameterFile(nav2_params_, allow_substs=True)
    map_substitution = {'database_path': ['/workspace/src/navigation/rtabmapdbs/', rtabmap_map_name]}
    rtab_params = ParameterFile(
        RewrittenYaml(
            source_file=rtab_params_,
            root_key='',
            param_rewrites=map_substitution,
            convert_types=True),
        allow_substs=True)

    sync_remapping = [
        ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
        ('depth/image', '/zed/zed_node/depth/depth_registered')]

    # Container 1: RTAB-Map (heavy SLAM + image sync)
    # Isolated so GPU/CPU-heavy SLAM cycles don't starve Nav2 threads
    rtabmap_container = ComposableNodeContainer(
        name='rtabmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                condition=IfCondition(localization),
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[rtab_params, shm_params,
                    {'Mem/IncrementalMemory': 'False',
                     'Mem/InitWMWithAllNodes': 'True',
                     'Mem/LocalizationReadOnly': 'True'}],
            ),
            ComposableNode(
                condition=UnlessCondition(localization),
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[rtab_params, shm_params,
                    {'delete_db_on_start': True,
                     'RGBD/LinearUpdate': '0.1',
                     'RGBD/AngularUpdate': '0.1',
                     'Rtabmap/TimeThr': '0',
                     'Mem/NotLinkedNodesKept': 'true'}]
            ),
            ComposableNode(
                package='rtabmap_sync',
                plugin='rtabmap_sync::RGBDSync',
                name='rgbd_sync',
                parameters=[rtab_params, shm_params],
                remappings=sync_remapping
            ),
        ],
    )

    # Container 2: Nav2 (time-critical control loop)
    # Dedicated thread pool ensures controller hits its target rate
    nav2_container = ComposableNodeContainer(
        condition=IfCondition(nav2_activate),
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        parameters=[nav2_params],
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[nav2_params, shm_params],
            ),
            ComposableNode(
                package='opennav_docking',
                plugin='opennav_docking::DockingServer',
                name='docking_server',
                parameters=[nav2_params, shm_params],
                condition=IfCondition(docking),
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[shm_params, {
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'velocity_smoother',
                    ]
                }],
            ),
        ],
    )

    return LaunchDescription([rtabmap_container, nav2_container])

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    localization = LaunchConfiguration('localization', default='true')
    bringup_dir = get_package_share_directory('nav_main')
    # params=os.path.join(bringup_dir, 'config', 'nav2_params_original.yaml')
    params=os.path.join(bringup_dir, 'config', 'nav2_params_tmr.yaml')
    # nav_yaml=os.path.join(bringup_dir, 'maps', 'tmr2025.yaml')
    params_file = LaunchConfiguration('params_file', default=params)

    rtabmap_slam_parameters={
            'subscribe_rgbd':True,
            'subscribe_depth': False,
            'subscribe_scan':True,
            'use_sim_time':False,
            'odom_sensor_sync': True,
            'frame_id':'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'database_path': '/workspace/rtabmap2.db',
            'approx_sync ': True,
            'odom_sensor_sync': True,
            #RTABMAP PARAMETERS
            'Rtabmap/DetectionRate': '15',
            'Mem/NotLinkedNodesKept':'false',
            'Kp/NNStrategy': '4',
            'Kp/MaxFeatures': '500',
            'Kp/DetectorStrategy': '8', #GFTT AND ORB
            'GFTT/Gpu' : 'true',
            'ORB/Gpu': 'true',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/ForceOdom3DoF': 'true',
            'RGBD/StartAtOrigin': 'false',
            #G2O Optimization
            'Optimizer/Strategy': '1', #g2o
            'Optimizer/Iterations': '20',
            'Optimizer/Epsilon': '0.0',
            'Optimizer/Robust': 'true', #por checar qpd
            'Optimizer/GravitySigma': '0.3',
            'g2o/Solver':'3',# defined by orb
            #Odom strategy
            'Odom/Strategy': '1', # WITH F2F SUPPORT
            'Odom/ResetCountdown': '0', #Check later
            'Odom/Holonomic': 'false',
            'Odom/FilteringStrategy': '1', #CHeck later
            'Odom/ParticleSize': '400',
            #Strategy visICP
            'Reg/Strategy': '2',
            'Reg/Force3DoF': 'true',
            'Vis/PnPRefineIterations': '0', #g2o and orb
            'Vis/MinInliers': '20',
            'Vis/Iterations': '300',
            'Vis/FeatureType': '8',
            'Vis/MaxFeatures': '1000',
            'Vis/BundleAdjustment': '1', #g2o and orb
            'GMS/WithRotation': 'true', #pa checar
            'GMS/WithScale': 'true', # pa checar
            'GMS/ThresholdFactor': '6.0', # pa checar
            'Icp/Strategy': '1',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/PointToPlane': 'true',
            #grid
            'Grid/Sensor': '2', # 2 para grid 3d y 0 para 2d
            'Grid/RangeMax': '5.0',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'false',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',  
        }
        
    param_substitutions = {
        'use_sim_time': 'false',
        'autostart': 'true'}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    nav2_nodes = [
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=[('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                 ] ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[{
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
            
            ComposableNode(
                 package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap_slam',
                parameters=[rtabmap_slam_parameters, {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
                condition=IfCondition(localization)
            ),

    ]

    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nav2_nodes,
        # parameters=[rtabmap_slam_parameters],
        output='screen',
    )

    return LaunchDescription([container])

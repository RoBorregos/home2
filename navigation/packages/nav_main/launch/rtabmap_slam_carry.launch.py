
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    localization = LaunchConfiguration('localization', default='true')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz', default='false')
    use_3d_grid = LaunchConfiguration('3d_grid', default='false')

    icp_parameters={
          'odom_frame_id':'icp_odom',
          'guess_frame_id':'odom'
    }

    rtabmap_parameters={
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'odom_sensor_sync': True,
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false'
    }

    if(use_3d_grid.perform(context) == 'true'):

        # Shared parameters between different nodes
        shared_parameters={
            'frame_id':'base_link',
            'use_sim_time':use_sim_time,
            'wait_for_transform_duration': 0.8,
            'queue_size': 3,
            'approx_sync ': True,
            'approx_sync_max_interval': '0.04',
            # RTAB-Map's parameters should be strings:
            'Reg/Strategy':'1',
            'Reg/Force3DoF':'true',
            'Mem/NotLinkedNodesKept':'false',
            'Icp/PointToPlaneMinComplexity':'0.05',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',  
            'RGBD/NeighborLinkRefining':'True',
            'Grid/CellSize': '0.04',
            'Vis/MaxFeatures': '100',
            'Rtabmap/DetectionRate': '15',
            #   'Grid/RayTracing':'true', # Fill empty space
            'Grid/3D':'false', # Use 2D occupancy
            'Grid/RangeMax':'3',
            'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
            'Grid/Sensor':'2', # Use both laser scan and camera for obstacle detection in global map
            'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
        }
    else:
         shared_parameters={
            'Mem/IncrementalMemory': 'true',
            'RGBD/OptimizeMaxError': '3.0',
            'RGBD/LinearUpdate': '0.05',

            'frame_id':'base_link',
            'use_sim_time':use_sim_time,
            'wait_for_transform_duration': 0.8,
            'queue_size': 200,
            'approx_sync ': False,

            ##### TEST ####
            # 'approx_sync ': True,
            # 'approx_sync_max_interval': '0.04',
            ###############
            # RTAB-Map's parameters should be strings:
            'Rtabmap/ScanMaxRange' : '2.0',
            'Reg/Strategy':'1',
            'Mem/BinDataKept': 'false',
            'RGBD/CreateOccupancyGrid': 'false',
            'Reg/Force3DoF':'true',
            'Mem/NotLinkedNodesKept':'false',
            'Icp/PointToPlaneMinComplexity':'0.05',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',  
            'RGBD/NeighborLinkRefining':'True',
            'Grid/CellSize': '0.04',
            'Vis/MaxFeatures': '100',
            'Rtabmap/DetectionRate': '30',
            # 'Optimizer/Iterations': '60',
            'Grid/RayTracing':'false', # Fill empty space
            'Grid/3D':'false', # Use 2D occupancy
            'Grid/RangeMax':'3',
            # 'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
            #   'Grid/Sensor':'2', # Use both laser scan and camera for obstacle detection in global map
            'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)

            'Grid/NormalsSegmentation': 'true',
            'Grid/MinClusterSize': '20',
            'Grid/FlatObstacleDetected': 'true',
        }


    remappings=[
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return_list = [

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
            remappings=remappings),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[icp_parameters, shared_parameters],
            remappings=remappings,
            arguments=["--ros-args", "--log-level", 'icp_odometry:=warn']),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
	    arguments=["-d"]
            ),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            condition=IfCondition(rtabmap_viz),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings),
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    localization = LaunchConfiguration('localization', default='false')
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
    }

    if(use_3d_grid.perform(context) == 'true'):

        # Shared parameters between different nodes
        shared_parameters={
            'frame_id':'base_link',
            'use_sim_time':use_sim_time,
            'wait_for_transform_duration': 0.8,
            'queue_size': 3,
            'approx_sync ': True,
            'approx_sync_max_interval': 0.01,    
            'Reg/Strategy':'1',
            'Reg/Force3DoF':'true',
            'Vis/MinInliers': '15',
            'Vis/EstimationType': '1',
            'Vis/CorType': '2',
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '500',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'True',
            'Odom/FeatureType': '6',
            'Mem/NotLinkedNodesKept':'false',
            'Icp/PointToPlaneMinComplexity':'0.05',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',  
            'RGBD/NeighborLinkRefining':'True',
            'Grid/CellSize': '0.04',
            'Rtabmap/DetectionRate': '15',
            'Grid/3D':'false',
            'Grid/RangeMax':'3',
            'Grid/NormalsSegmentation':'false',
            'Grid/Sensor':'2',
            'Optimizer/GravitySigma':'0'
        }
    else:
         shared_parameters={
            'frame_id':'base_link',
            'use_sim_time':use_sim_time,
            'wait_for_transform_duration': 0.8,
            'queue_size': 5,
            'approx_sync ': True,
            'approx_sync_max_interval': 0.01,    
            'Reg/Strategy':'1',
            'Reg/Force3DoF':'true',
            'Vis/MinInliers': '15',
            'Vis/EstimationType': '1',
            'Vis/CorType': '2',
            'Vis/CorNNType': '4',
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'True',
            'Odom/FeatureType': '6',
            'Optimizer/GravitySigma':'0',
            'Optimizer/Slam2D':'true',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',  
            'Grid/CellSize': '0.04',
            'Grid/FromDepth':'false', 
            'Grid/3D':'false',
            'Grid/FromScan':'true',
            'Grid/RangeMax':'3',
            'Grid/NormalsSegmentation':'false',
            'Grid/Sensor':'1',
            'Mem/NotLinkedNodesKept':'false',
            'Icp/PointToPlaneMinComplexity':'0.05',
            'RGBD/NeighborLinkRefining':'True',
            'Rtabmap/DetectionRate': '15',
            'ORB/Gpu': 'true',
            'Kp/NNStrategy': '4',
            'SURF/GpuVersion': 'true',
            'FAST/Gpu': 'true',

        }


    remappings=[
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return_list = [
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}, shared_parameters],
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
        # Node(
        #     package='rtabmap_util', executable='obstacles_detection', output='screen',
        #     parameters=[rtabmap_parameters,shared_parameters],
        #     remappings=[('cloud', '/zed/zed_node/point_cloud/cloud_registered'),
        #                 ('obstacles', '/camera/obstacles'),
        #                 ('ground', '/camera/ground')]),
        Node(
            condition=IfCondition(rtabmap_viz),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings),
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
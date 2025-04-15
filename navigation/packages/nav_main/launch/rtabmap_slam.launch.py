
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

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

    # Shared parameters between different nodes
    shared_parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'Mem/NotLinkedNodesKept':'false',
          'Icp/PointToPlaneMinComplexity':'0.04', # to be more robust to long corridors with low geometry
          'Grid/MaxGroundHeight':'0.2', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1.2',  # All points over 1 meter are ignored
    }

    remappings=[
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true', choices=['true', 'false'],
            description='Launch rtabmap_viz for visualization.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
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
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[rtabmap_parameters,shared_parameters],
            remappings=[('cloud', '/zed/zed_node/point_cloud/cloud_registered'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])

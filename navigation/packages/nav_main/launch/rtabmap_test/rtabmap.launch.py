
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    localization = LaunchConfiguration('localization', default='false')

    rtabmap_slam_parameters={
            'subscribe_rgbd':True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_scan':True,
            'use_sim_time':False,
            'odom_sensor_sync': True,
            'frame_id':'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'database_path': '/workspace/rtabmap3.db',
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
            'RGBD/ProximityPathMaxNeighbors' : '0',
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
            'Grid/Sensor': '0', # 2 para grid 3d y 0 para 2d
            'Grid/RangeMax': '3.0',
            'Grid/CellSize': '0.07',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'false',
            'Grid/MaxGroundHeight':'0.1', 
            'Grid/MaxObstacleHeight':'2',
            'GridGlobal/Eroded': 'true',
            'GridGlobal/FootprintRadius': '0.2'  

        }
    return_list = [
        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_slam_parameters],
	        arguments=["-d"]
            ),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_slam_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}]),
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
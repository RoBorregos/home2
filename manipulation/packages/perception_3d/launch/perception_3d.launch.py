#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic", default="/point_cloud")
    sim_time_param = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="/point_cloud",
                description="Point cloud topic consumed by the cluster extraction. Real robot uses /point_cloud; sim overrides to /filtered_cloud (MoveIt self-filtered) so robot meshes don't pollute the cluster.",
            ),
            Node(
                package="perception_3d",
                executable="pick_primitives",
                name="pick_primitives",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="perception_3d",
                executable="plane_service",
                name="plane_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {"point_cloud_topic": point_cloud_topic},
                    sim_time_param,
                ],
            ),
            Node(
                package="perception_3d",
                executable="test_only_orchestrator",
                name="test_only_orchestrator",
                output="screen",
                respawn=True,
                emulate_tty=True,
                parameters=[
                    {"testing": False, "point_cloud_topic": point_cloud_topic},
                    sim_time_param,
                ],
            ),
        ]
    )

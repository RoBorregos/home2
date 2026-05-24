#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # When the sim includes this launch it overrides use_sim_time to true so
    # gpd_service, manipulation_core, pick/place/pour servers and perception
    # all look up TFs against /clock. On the real robot the default (false)
    # keeps everything on wall time -- same launch, no sim-specific code.
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    sim_time_param = {"use_sim_time": use_sim_time}

    def grasp_detector_node(context, *args, **kwargs):
        grasp_backend = LaunchConfiguration("grasp_backend").perform(context)

        # ee_link_offset differs per backend:
        #   GPD  reports the palm-root center → offset -0.125 m
        #   CGN  reports the contact point on the object surface → offset -0.09 m
        ee_link_offset = -0.125
        pick_server_node = Node(
            package="pick_and_place",
            executable="pick_server.py",
            name="pick_server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"ee_link_offset": ee_link_offset},
                sim_time_param,
            ],
        )

        if grasp_backend == "contact_graspnet":
            install_deps = ExecuteProcess(
                cmd=["pip3", "install", "-q", "trimesh", "tqdm"],
                output="screen",
                name="install_cgn_deps",
            )
            cgn_node = Node(
                package="contact_graspnet_ros",
                executable="contact_graspnet_node",
                name="contact_graspnet_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "ckpt_dir": LaunchConfiguration("ckpt_dir").perform(context),
                        "forward_passes": 1,
                        "z_range": [0.2, 1.8],
                        "pick_min_height": float(
                            LaunchConfiguration("pick_min_height").perform(context)
                        ),
                    }
                ],
            )
            return [
                install_deps,
                RegisterEventHandler(
                    OnProcessExit(target_action=install_deps, on_exit=[cgn_node])
                ),
                pick_server_node,
            ]
        # default: gpd
        return [
            Node(
                package="arm_pkg",
                executable="gpd_service",
                name="gpd_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[sim_time_param],
            ),
            pick_server_node,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            DeclareLaunchArgument(
                "grasp_backend",
                default_value="gpd",
                description="Grasp detection backend: 'gpd' (default) or 'contact_graspnet'.",
            ),
            DeclareLaunchArgument(
                "ckpt_dir",
                default_value="checkpoints/contact_graspnet",
                description="Path to Contact-GraspNet checkpoint dir (used only when grasp_backend=contact_graspnet).",
            ),
            DeclareLaunchArgument(
                "pick_min_height",
                default_value="0.03",
                description="Minimum height (m) above cluster bottom a grasp must clear. "
                "Lower for flat objects (banana=0.01, normal=0.03).",
            ),
            OpaqueFunction(function=grasp_detector_node),
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="place_server.py",
                name="place_server",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="pour_server.py",
                name="pour_server",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            # perception_3d.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perception_3d"),
                            "launch",
                            "perception_3d.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "point_cloud_topic": LaunchConfiguration(
                        "point_cloud_topic", default="/point_cloud"
                    ),
                }.items(),
            ),
            Node(
                package="place",
                executable="heatmapPlace_Server.py",
                parameters=[sim_time_param],
            ),
            Node(
                package="frida_motion_planning",
                executable="motion_planning_server.py",
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="fix_position_to_plane.py",
                name="fix_position_to_plane",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="perception_3d",
                executable="flat_grasp_estimator.py",
                name="flat_grasp_estimator",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # When the sim includes this launch it overrides use_sim_time to true so
    # contact_graspnet_node / gpd_service, manipulation_core, pick/place/pour
    # servers and perception all look up TFs against /clock.  On the real robot
    # the default (false) keeps everything on wall time — same launch, no
    # sim-specific code.
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    sim_time_param = {"use_sim_time": use_sim_time}

    def grasp_nodes(context, *args, **kwargs):
        # ee_link_offset: scalar applied along the approach axis in pick_server.
        # For a top-down grasp (approach=[0,0,-1]) with offset=-0.125:
        #   IK target z = contact_z + 0.125
        # This places gripper_grasp_frame (palm) 12.5 cm above the contact
        # point so the finger tips reach the object surface. Same value works
        # for both backends (GPD reports palm-root, CGN reports contact point —
        # the 12.5 cm offset puts the palm correctly in both cases).
        ee_link_offset = -0.125

        backend = LaunchConfiguration("grasp_backend").perform(context).lower()
        if backend not in ("contact_graspnet", "gpd"):
            raise RuntimeError(
                f"Invalid grasp_backend '{backend}'. "
                "Must be 'contact_graspnet' or 'gpd'."
            )

        # pick_server reads grasp_backend so PickManager can switch behaviour
        # (GPD uses CFG_PATHS + reversible grasps; CGN takes top-N by score).
        pick_server_node = Node(
            package="pick_and_place",
            executable="pick_server.py",
            name="pick_server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "ee_link_offset": ee_link_offset,
                    "grasp_backend": backend,
                },
                sim_time_param,
            ],
        )

        # manipulation_core hosts PickManager, which also needs the backend.
        manipulation_core_node = Node(
            package="pick_and_place",
            executable="manipulation_core.py",
            name="manipulation_core",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"grasp_backend": backend},
                sim_time_param,
            ],
        )

        nodes = [pick_server_node, manipulation_core_node]

        if backend == "contact_graspnet":
            nodes.append(
                Node(
                    package="contact_graspnet_ros",
                    executable="contact_graspnet_node",
                    name="contact_graspnet_node",
                    output="screen",
                    emulate_tty=True,
                    respawn=True,
                    parameters=[
                        {
                            "ckpt_dir": LaunchConfiguration("ckpt_dir").perform(
                                context
                            ),
                            "forward_passes": 1,
                            "z_range": [0.2, 1.8],
                            "pick_min_height": float(
                                LaunchConfiguration("pick_min_height").perform(context)
                            ),
                        }
                    ],
                )
            )
        else:  # gpd
            nodes.append(
                Node(
                    package="arm_pkg",
                    executable="gpd_service",
                    name="gpd_service",
                    output="screen",
                    emulate_tty=True,
                    respawn=True,
                    parameters=[sim_time_param],
                )
            )

        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            DeclareLaunchArgument(
                "grasp_backend",
                default_value="contact_graspnet",
                description="Grasp detection backend: 'contact_graspnet' (default, "
                "PyTorch model, ~4-5s warm) or 'gpd' (C++ geometric, ~1-3s).",
            ),
            DeclareLaunchArgument(
                "ckpt_dir",
                default_value="checkpoints/contact_graspnet",
                description="Path to Contact-GraspNet checkpoint dir "
                "(absolute or relative to submodule root). Ignored when backend=gpd.",
            ),
            DeclareLaunchArgument(
                "pick_min_height",
                default_value="0.03",
                description="Minimum height (m) above cluster bottom a grasp must clear. "
                "Lower for flat objects (e.g. banana=0.01, normal=0.03). CGN only.",
            ),
            OpaqueFunction(function=grasp_nodes),
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

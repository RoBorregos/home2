import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "config",
        "parameters_zero_shot.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Bridge /vision/zero_shot_detections -> /vision/detections so pick_server
    # and keyboard_input see detections under the same topic name they use on
    # the real robot (where the pretrained detector fills /vision/detections
    # directly).  topic_tools isn't in the vision image, so inline a tiny
    # rclpy relay instead of adding a new package dependency.
    detections_relay = ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            (
                "import rclpy\n"
                "from rclpy.node import Node\n"
                "from frida_interfaces.msg import ObjectDetectionArray\n"
                "rclpy.init()\n"
                "n = Node('zero_shot_to_detections_relay')\n"
                "pub = n.create_publisher(ObjectDetectionArray, '/vision/detections', 10)\n"
                "n.create_subscription(ObjectDetectionArray, '/vision/zero_shot_detections', lambda m: pub.publish(m), 10)\n"
                "n.get_logger().info('zero_shot -> /vision/detections relay up')\n"
                "rclpy.spin(n)\n"
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            Node(
                package="object_detector_2d",
                executable="zero_shot_object_detector_node.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=False,
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
            detections_relay,
        ]
    )

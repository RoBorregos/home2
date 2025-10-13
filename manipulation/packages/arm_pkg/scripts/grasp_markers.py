# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from frida_interfaces.srv import GraspDetection, ReadPcdFile
import colorsys
import tf_transformations
import numpy as np


class GraspVisualizer(Node):
    def __init__(self):
        super().__init__("grasp_visualizer")

        self.declare_parameter("input_mode", "pcd")  # 'pcd' or 'topic'

        # Configuration based on input mode
        if self.get_parameter("input_mode").value == "pcd":
            self.setup_pcd_mode()
        else:
            self.setup_topic_mode()

        # Configuration for visualization
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.gripper_dimensions = {
            "base": (0.1, 0.1, 0.05),
            "finger": (0.02, 0.02, 0.17),
            "separation": 0.08,
        }

        self.marker_pub = self.create_publisher(
            MarkerArray, "/grasp_markers", qos_profile
        )
        self.pcd_pub = self.create_publisher(
            PointCloud2, "/grasp_pcl", qos_profile
        )  # Publicador de la nube de puntos
        self.grasp_client = self.create_client(GraspDetection, "detect_grasps")

    def setup_pcd_mode(self):
        """PCD file mode"""
        self.declare_parameter(
            "pcd_path",
            "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/tutorials/krylon.pcd",
        )
        self.declare_parameter(
            "cfg_path",
            "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg",
        )

        self.pcd_client = self.create_client(ReadPcdFile, "read_pcd_file")
        while not self.pcd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for PCD service...")

        pcd_path = self.get_parameter("pcd_path").value
        cfg_path = self.get_parameter("cfg_path").value
        self.process_pcd(pcd_path, cfg_path)

    def setup_topic_mode(self):
        """PointCloud2 mode"""
        self.subscription = self.create_subscription(
            PointCloud2,
            "/input_point_cloud",
            self.topic_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.declare_parameter("cfg_path", "")
        self.get_logger().info("Waiting for PointCloud2 messages...")

    def process_pcd(self, pcd_path, cfg_path):
        """Process PCD file"""
        request = ReadPcdFile.Request()
        request.pcd_path = pcd_path
        future = self.pcd_client.call_async(request)
        future.add_done_callback(self.pcd_response_callback)

    def topic_callback(self, msg):
        """Callback para PointCloud2"""
        request = GraspDetection.Request()
        request.cfg_path = self.get_parameter("cfg_path").value
        request.input_cloud = msg
        future = self.grasp_client.call_async(request)
        future.add_done_callback(self.grasp_response_callback)

    def pcd_response_callback(self, future):
        """Response callback for PCD"""
        try:
            response = future.result()
            if response.success:
                self.current_cloud = response.cloud
                self.current_cloud.header.frame_id = (
                    "link_base"  # change frame_id if necessary
                )
                self.publish_pcl()

                request = GraspDetection.Request()
                request.cfg_path = self.get_parameter("cfg_path").value
                request.pcd_path = self.get_parameter("pcd_path").value
                grasp_future = self.grasp_client.call_async(request)
                grasp_future.add_done_callback(self.grasp_response_callback)
        except Exception as e:
            self.get_logger().error(f"PCD error: {str(e)}")

    def publish_pcl(self):
        """Publish PointCloud2 in /grasp_pcl topic"""
        if self.current_cloud is not None:  # This is not initialized in topic mode
            self.get_logger().info(
                f"Publishing PointCloud2 with frame_id: {self.current_cloud.header.frame_id}"
            )
            self.pcd_pub.publish(self.current_cloud)

    def grasp_response_callback(self, future):
        """Process grasp detection (common to both modes)"""
        try:
            response = future.result()
            if response.success:
                self.publish_gripper_markers(
                    response.grasp_poses, response.grasp_scores
                )
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")

    def publish_gripper_markers(self, pose_stamped_array, scores):
        marker_array = MarkerArray()

        for idx, (pose_stamped, score) in enumerate(zip(pose_stamped_array, scores)):
            # Use score for visualization
            color_factor = max(0.2, score)  # Ensure minimum visibility
            gripper_markers = self.create_gripper(
                pose_stamped.pose,
                f"grasp_{idx}",
                frame_id=pose_stamped.header.frame_id,  # Use frame from pose
                hue=color_factor,
            )
            marker_array.markers.extend(gripper_markers)

        self.marker_pub.publish(marker_array)

    def create_gripper(self, base_pose, ns, frame_id, hue=0.5):
        markers = []
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

        q = [
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w,
        ]

        trans = tf_transformations.translation_matrix(
            [base_pose.position.x, base_pose.position.y, base_pose.position.z]
        )
        rot = tf_transformations.quaternion_matrix(q)
        yaw_rot = tf_transformations.euler_matrix(0, 0, np.pi / 2)
        transform = trans @ rot @ yaw_rot

        offset_distance = -0.08  # 8 cm in direction of the gripper
        offset_transform = tf_transformations.translation_matrix(
            [0, 0, offset_distance]
        )
        transform = transform @ offset_transform

        translation = tf_transformations.translation_from_matrix(transform)
        base_quat = tf_transformations.quaternion_from_matrix(transform)

        base_marker = self._create_marker(
            frame_id=frame_id,
            ns=ns,
            marker_id=0,
            marker_type=Marker.CUBE,
            position=translation,
            orientation=base_quat,
            scale=self.gripper_dimensions["base"],
            color=(r * 0.5, g * 0.5, b * 0.5, 0.8),
        )
        markers.append(base_marker)

        # Fingers
        for i in range(2):
            x_offset = ((-1) ** i) * self.gripper_dimensions["separation"] / 2
            finger_transform = transform @ tf_transformations.translation_matrix(
                [x_offset, 0, self.gripper_dimensions["finger"][2] / 2]
            )
            finger_marker = self._create_marker(
                frame_id=frame_id,
                ns=ns,
                marker_id=i + 1,
                marker_type=Marker.CUBE,
                position=tf_transformations.translation_from_matrix(finger_transform),
                orientation=base_quat,
                scale=self.gripper_dimensions["finger"],
                color=(r, g, b, 0.8),
            )
            markers.append(finger_marker)

        return markers

    def _create_marker(
        self, frame_id, ns, marker_id, marker_type, position, orientation, scale, color
    ):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        # Use orientation as list [x, y, z, w]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = GraspVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

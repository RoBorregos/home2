#!/usr/bin/env python3
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

        # Configuración de QoS para sincronización con datos sensoriales
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, "/grasp_markers", qos_profile
        )
        self.pcd_pub = self.create_publisher(PointCloud2, "/grasp_pcl", qos_profile)

        # Clients
        self.grasp_client = self.create_client(GraspDetection, "detect_grasps")
        self.pcd_client = self.create_client(ReadPcdFile, "read_pcd_file")

        # Esperar conexión con servicios
        while not self.grasp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Servicio detect_grasps no disponible...")
        while not self.pcd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Servicio read_pcd_file no disponible...")

        # Parámetros y configuración
        self.gripper_dimensions = {
            "base": (0.1, 0.1, 0.02),
            "finger": (0.02, 0.02, 0.17),
            "separation": 0.08,
        }

        # Almacenamiento de datos
        self.current_cloud = None

        # self.declare_parameter("pcd_path","/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/table_mug.pcd")
        # self.declare_parameter("pcd_path","/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/cluster_(1).pcd")
        # self.declare_parameter("pcd_path","/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/cluster.pcd")
        self.declare_parameter(
            "pcd_path",
            "/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/tuto.pcd",
        )
        # self.declare_parameter("pcd_path","/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/krylon.pcd")
        self.declare_parameter(
            "cfg_path",
            "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg",
        )

        # Obtener valores de los parámetros
        pcd_path = self.get_parameter("pcd_path").value
        cfg_path = self.get_parameter("cfg_path").value

        self.load_and_process_pcd(pcd_path, cfg_path)

    def call_service(self, cfg_path, pcd_path):
        request = GraspDetection.Request()
        request.cfg_path = cfg_path
        request.pcd_path = pcd_path
        future = self.grasp_client.call_async(request)
        future.add_done_callback(self.grasp_response_callback)

    def load_and_process_pcd(self, pcd_path, cfg_path):
        """Secuencia completa: Cargar PCD -> Detectar grasps"""
        self.read_pcd_file(pcd_path)
        self.call_grasp_service(cfg_path, pcd_path)

    def read_pcd_file(self, pcd_path):
        """Solicitar lectura del PCD"""
        request = ReadPcdFile.Request()
        request.pcd_path = pcd_path
        # request.viewpoint = [0.0, 0.0, 0.0]  # Ajusta según la posición real de la cámara
        future = self.pcd_client.call_async(request)
        future.add_done_callback(self.pcd_response_callback)

    def pcd_response_callback(self, future):
        """Procesar respuesta del servicio de PCD"""
        try:
            response = future.result()
            if response.success:
                self.current_cloud = response.cloud
                self.publish_pcl()
            else:
                # Si el servicio devuelve un mensaje de error, úsalo
                error_msg = (
                    response.error_message
                    if hasattr(response, "error_message")
                    else "Error desconocido"
                )
                self.get_logger().error(f"Error leyendo PCD: {error_msg}")
        except Exception as e:
            self.get_logger().error(f"Error en servicio PCD: {str(e)}")

    def publish_pcl(self):
        if self.current_cloud is not None:
            self.get_logger().info(
                f"Frame ID de la nube: {self.current_cloud.header.frame_id}"
            )  # 👈
            self.pcd_pub.publish(self.current_cloud)

    def call_grasp_service(self, cfg_path, pcd_path):
        """Iniciar detección de grasps"""
        request = GraspDetection.Request()
        request.cfg_path = cfg_path
        request.pcd_path = pcd_path
        future = self.grasp_client.call_async(request)
        future.add_done_callback(self.grasp_response_callback)

    def grasp_response_callback(self, future):
        """Procesar detección de grasps"""
        try:
            response = future.result()
            if response.success:
                self.publish_gripper_markers(response.grasp_poses)
            else:
                self.get_logger().warn("Detección de grasps fallida")
        except Exception as e:
            self.get_logger().error(f"Error en detección: {str(e)}")

    def publish_gripper_markers(self, pose_array):
        marker_array = MarkerArray()

        for idx, pose in enumerate(pose_array.poses):
            gripper_markers = self.create_gripper(
                pose, f"grasp_{idx}", idx / len(pose_array.poses)
            )
            marker_array.markers.extend(gripper_markers)

        self.marker_pub.publish(marker_array)

    def create_gripper(self, base_pose, ns, hue):
        markers = []
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

        # Extraer componentes del quaternion correctamente
        q = [
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w,
        ]

        # Matriz de transformación
        trans = tf_transformations.translation_matrix(
            [base_pose.position.x, base_pose.position.y, base_pose.position.z]
        )
        rot = tf_transformations.quaternion_matrix(q)
        yaw_rot = tf_transformations.euler_matrix(0, 0, np.pi / 2)
        transform = trans @ rot @ yaw_rot

        # Base del gripper
        base_quat = tf_transformations.quaternion_from_matrix(transform)
        base_marker = self._create_marker(
            frame_id="link_base",
            ns=ns,
            marker_id=0,
            marker_type=Marker.CUBE,
            position=tf_transformations.translation_from_matrix(transform),
            orientation=base_quat,
            scale=self.gripper_dimensions["base"],
            color=(r * 0.5, g * 0.5, b * 0.5, 0.8),
        )
        base_marker.pose.position.z += self.gripper_dimensions["base"][2] / 2
        markers.append(base_marker)

        # Dedos
        for i in range(2):
            x_offset = ((-1) ** i) * self.gripper_dimensions["separation"] / 2
            finger_transform = transform @ tf_transformations.translation_matrix(
                [x_offset, 0, self.gripper_dimensions["finger"][2] / 2]
            )

            finger_marker = self._create_marker(
                frame_id="link_base",
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

        # Usar la orientación como lista [x, y, z, w]
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

    # node.call_service(
    #     cfg_path="/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg",
    #     # pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/table_mug.pcd",
    #     # pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/cluster_(1).pcd",
    #     # pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/cluster.pcd",
    #     # pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/tuto.pcd",
    #     pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/krylon.pcd",

    # )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

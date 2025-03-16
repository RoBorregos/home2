#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import PoseArray
# from frida_interfaces.srv import GraspDetection
# import colorsys

# import sys

# class GraspVisualizer(Node):
#     def __init__(self):
#         super().__init__('grasp_visualizer')
#         self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)

#         # Cliente para llamar al servicio
#         self.client = self.create_client(GraspDetection, 'detect_grasps')
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Servicio no disponible, esperando...')

#         # Configurar marcadores
#         self.marker_id = 0
#         self.color = [0.0, 1.0, 0.0, 0.8]  # Verde

#     def call_service(self, cfg_path, pcd_path):
#         request = GraspDetection.Request()
#         request.cfg_path = cfg_path
#         request.pcd_path = pcd_path

#         future = self.client.call_async(request)
#         future.add_done_callback(self.handle_response)

#     def handle_response(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.publish_markers(response.grasp_poses)
#         except Exception as e:
#             self.get_logger().error(f'Error: {e}')

#     def publish_markers(self, pose_array):
#         marker_array = MarkerArray()

#         num_poses = len(pose_array.poses)
#         for idx, pose in enumerate(pose_array.poses):

#             marker = Marker()
#             marker.header.frame_id = "link_base"  # ¡Asegúrate que este frame exista!
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "grasps"
#             marker.id = self.marker_id
#             marker.type = Marker.ARROW
#             marker.action = Marker.ADD
#             marker.pose = pose
#             marker.scale.x = 0.1 #Longitud
#             marker.scale.y = 0.02  #  Grosor
#             marker.scale.z = 0.02

#             hue = idx / float(num_poses)  # Valor entre 0 y 1
#             r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

#             marker.color.r = r  #Rojo
#             marker.color.g = g
#             marker.color.b = b
#             marker.color.a = 10.0  # Opacidad alta

#             # marker.color.r = 1.0  #Rojo
#             # marker.color.g = 0.0
#             # marker.color.b = 0.0
#             # marker.color.a = 10.0  # Opacidad alta


#             marker_array.markers.extend([marker,])
#             self.marker_id += 1

#         self.marker_pub.publish(marker_array)


# def main(args=None):
#     rclpy.init(args=args)
#     node = GraspVisualizer()

#     # Llama al servicio con tus rutas
#     node.call_service(
#         cfg_path="/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg",
#         # pcd_path="/home/dominguez/Downloads/cluster.pcd"
#         pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/table_mug.pcd"
#     )

#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# from sensor_msgs.msg import PointCloud2, PointField
from frida_interfaces.srv import GraspDetection  # , ReadPcdFile
import colorsys
import tf_transformations
import numpy as np
# from std_msgs.msg import Header


class GraspVisualizer(Node):
    def __init__(self):
        super().__init__("grasp_visualizer")

        # qos_profile = rclpy.qos.QoSProfile(
        #     depth=10,
        #     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        #     durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        # )

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/grasp_markers", 10)
        # self.pcd_pub = self.create_publisher(PointCloud2, "/grasp_pcl", qos_profile)

        # Clients
        self.grasp_client = self.create_client(GraspDetection, "detect_grasps")
        # self.pcd_client = self.create_client(ReadPcdFile, "read_pcd_file")

        # Esperar conexión con servicios
        while not self.grasp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Servicio detect_grasps no disponible...")
        # while not self.pcd_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Servicio read_pcd_file no disponible...")

        # Parámetros y configuración
        self.gripper_dimensions = {
            "base": (0.1, 0.1, 0.02),
            "finger": (0.02, 0.02, 0.17),
            "separation": 0.08,
        }

        # Almacenamiento de datos
        self.current_cloud = None

        # Iniciar proceso
        # self.load_and_process_pcd(
        #     "/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/table_mug.pcd",
        #     "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg"
        # )

    def call_service(self, cfg_path, pcd_path):
        request = GraspDetection.Request()
        request.cfg_path = cfg_path
        request.pcd_path = pcd_path
        future = self.grasp_client.call_async(request)
        future.add_done_callback(self.grasp_response_callback)

    def load_and_process_pcd(self, pcd_path, cfg_path):
        """Secuencia completa: Cargar PCD -> Detectar grasps"""
        # self.read_pcd_file(pcd_path)
        self.call_grasp_service(cfg_path, pcd_path)

    # def read_pcd_file(self, pcd_path):
    #     """Solicitar lectura del PCD"""
    #     request = ReadPcdFile.Request()
    #     request.pcd_path = pcd_path
    #     future = self.pcd_client.call_async(request)
    #     future.add_done_callback(self.pcd_response_callback)

    def pcd_response_callback(self, future):
        """Procesar respuesta del servicio de PCD"""
        try:
            response = future.result()
            print(response)
            if response.success:
                self.current_cloud = response.cloud
                self.publish_pcl()
            else:
                self.get_logger().error("Error leyendo PCD")
        except Exception as e:
            self.get_logger().error(f"Error en servicio PCD: {str(e)}")

    def publish_pcl(self):
        """Publicar nube de puntos actual"""
        if self.current_cloud is not None:
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

    node.call_service(
        cfg_path="/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg",
        pcd_path="/home/dominguez/roborregos/home_ws/install/perception_3d/share/perception_3d/table_mug.pcd",
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

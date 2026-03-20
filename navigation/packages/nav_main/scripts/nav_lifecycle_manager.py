#!/usr/bin/env python3
import sys
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
import tf2_ros
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition as MsgTransition

class NavDependencyLifecycleManager(LifecycleNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None 
        
        self.required_topics = {'/zed/zed_node/rgb/camera_info'}
        self.required_frames = {'base_link'}
        
        self.ready_to_activate = False
        self.timer = None
        
        self.declare_parameter('autostart', True)
        self.declare_parameter('managed_nodes', [])
        self.managed_nodes = self.get_parameter('managed_nodes').get_parameter_value().string_array_value

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando: Iniciando monitoreo autónomo de dependencias")
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        if self.timer is not None:
            self.timer.cancel()
            
        self.timer = self.create_timer(2.0, self.monitor_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if not self.ready_to_activate:
            self.get_logger().warn("Intento de activación sin dependencias listas")
            return TransitionCallbackReturn.FAILURE
            
        self.get_logger().info("Sistema ACTIVO: Gestionando nodos dependientes")
        
        # Transition managed nodes to ACTIVE
        for node_name in self.managed_nodes:
            self.transition_node(node_name, MsgTransition.TRANSITION_CONFIGURE)
            self.transition_node(node_name, MsgTransition.TRANSITION_ACTIVATE)
            
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando: Deteniendo servicios dependientes")
        
        # Optionally deactivate managed nodes
        for node_name in self.managed_nodes:
            self.transition_node(node_name, MsgTransition.TRANSITION_DEACTIVATE)
            
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    def monitor_callback(self):
        """Callback autónomo para monitoreo de dependencias."""
        if self.ready_to_activate:
            return

        topic_names_and_types = self.get_topic_names_and_types()
        active_topics = {t[0] for t in topic_names_and_types}
        topics_ready = self.required_topics.issubset(active_topics)
        
        try:
            frames_dict = self.tf_buffer.all_frames_as_yaml()
            tf_ready = all(frame in frames_dict for frame in self.required_frames)
        except Exception:
            tf_ready = False
        
        if topics_ready and tf_ready:
            self.get_logger().info("Dependencias encontradas. Transicionando a ACTIVE...")
            self.ready_to_activate = True
            # Transición interna autónoma (sin llamar a servicio externo)
            self.trigger_activate()
        else:
            missing_topics = self.required_topics - active_topics
            self.get_logger().info(f"Esperando dependencias: Tópicos {list(missing_topics)} | TF: {tf_ready}", once=True)

    def transition_node(self, node_name, transition_id):
        """Helper para transicionar un nodo externo via servicio."""
        client = self.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Servicio de ciclo de vida para {node_name} no disponible")
            return

        req = ChangeState.Request()
        req.transition.id = transition_id
        client.call_async(req)
        self.get_logger().info(f"Transición {transition_id} enviada a {node_name}")

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = NavDependencyLifecycleManager('nav_lifecycle_manager')
    executor.add_node(node)
    
    autostart = node.get_parameter('autostart').get_parameter_value().bool_value
    if autostart:
        node.get_logger().info("Autostart habilitado")
        node.trigger_configure()
    
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

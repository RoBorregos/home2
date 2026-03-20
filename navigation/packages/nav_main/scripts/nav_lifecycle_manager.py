#!/usr/bin/env python3
import sys
import threading
import time
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
        self.monitoring_thread = None
        self.stop_monitoring = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Iniciando monitoreo")
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.stop_monitoring = False
        self.monitoring_thread = threading.Thread(target=self.monitor_dependencies)
        self.monitoring_thread.start()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if not self.ready_to_activate:
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("Dependencias externas encontradas")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando")
        self.stop_monitoring = True
        if self.monitoring_thread:
            self.monitoring_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def monitor_dependencies(self):
        self.get_logger().info(f"Monitoreando: Tópicos {self.required_topics} y TFs {self.required_frames}")
        
        client = self.create_client(ChangeState, f'{self.get_name()}/change_state')
        
        while rclpy.ok() and not self.stop_monitoring and not self.ready_to_activate:
            topic_names_and_types = self.get_topic_names_and_types()
            active_topics = {t[0] for t in topic_names_and_types}
            topics_ready = self.required_topics.issubset(active_topics)
            
            try:
                frames_dict = self.tf_buffer.all_frames_as_yaml()
                tf_ready = all(frame in frames_dict for frame in self.required_frames)
            except Exception:
                tf_ready = False
            
            if topics_ready and tf_ready:
                self.get_logger().info("Dependencias encontradas")
                self.ready_to_activate = True
                
                # Transition to ACTIVE
                if client.wait_for_service(timeout_sec=5.0):
                    req = ChangeState.Request()
                    req.transition.id = MsgTransition.TRANSITION_ACTIVATE
                    client.call_async(req)
                break
            
            missing_topics = self.required_topics - active_topics
            self.get_logger().info(f"Faltan Tópicos: {list(missing_topics)} | TF Ready: {tf_ready}")
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = NavDependencyLifecycleManager('nav_lifecycle_manager')
    executor.add_node(node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

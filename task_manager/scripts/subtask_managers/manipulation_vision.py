#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16, SetInt16ById
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class ArmServices(Node):  # Inherit from Node
    def __init__(self):
        super().__init__('arm_services')  # Correct superclass initialization
        
        # Subscription to arm state
        self.subscription = self.create_subscription(
            PoseStamped, '/xarm/robot_states', self.listener_callback, 10
        )
        
        # Create a service
        self.create_service(Trigger, '/arm_turn_on', self.sendTurnRequest)
        
        # Create clients for services
        self.state_client = self.create_client(SetInt16, '/xarm/set_state')
        self.motion_enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.mode_client = self.create_client(SetInt16, '/xarm/set_mode')

        # Ensure services are available
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')        

    def sendTurnRequest(self, request, response):
        """ Fire-and-forget implementation for enabling motion, setting state, and mode """
        self.get_logger().info("Processing turn-on request")

        # Enable motion
        motion_request = SetInt16ById.Request()
        motion_request.id = 8
        motion_request.data = 1
        future_motion = self.motion_enable_client.call_async(motion_request)
        future_motion.add_done_callback(self.motion_response_callback)  # Fire-and-forget

        # Set state
        state_request = SetInt16.Request()
        state_request.data = 0
        future_state = self.state_client.call_async(state_request)
        future_state.add_done_callback(self.state_response_callback)  # Fire-and-forget

        # Set mode
        mode_request = SetInt16.Request()
        mode_request.data = 4
        future_mode = self.mode_client.call_async(mode_request)
        future_mode.add_done_callback(self.mode_response_callback)  # Fire-and-forget

        response.success = True
        response.message = "Motion request sent asynchronously"
        return response

    def motion_response_callback(self, future):
        """ Callback for motion enable service response """
        try:
            result = future.result()
            if result:
                self.get_logger().info('Motion is enabled')
            else:
                self.get_logger().error('Motion is not enabled')
        except Exception as e:
            self.get_logger().error(f"Motion service call failed: {str(e)}")

    def state_response_callback(self, future):
        """ Callback for state service response """
        try:
            result = future.result()
            if result:
                self.get_logger().info(f'Arm status is ')
            else:
                self.get_logger().error('Failed to get arm status')
        except Exception as e:
            self.get_logger().error(f"State service call failed: {str(e)}")

    def mode_response_callback(self, future):
        """ Callback for mode service response """
        try:
            result = future.result()
            if result:
                self.get_logger().info(f'Mode is ')
            else:
                self.get_logger().error('Failed to set mode')
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {str(e)}")

    def get_Pose(self):
        self.get_logger().info("Subscribing to arm pose updates...")
        

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Pose: {msg}')

def main():
    rclpy.init()
    node = ArmServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

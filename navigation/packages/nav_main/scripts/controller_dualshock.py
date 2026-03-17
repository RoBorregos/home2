import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class ControllerDualShock(Node):
    def __init__(self):
        super().__init__('controller_dualshock')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose/_action/cancel_goal')
    
    def joy_callback(self, msg):    
        if msg.buttons[3] == 1: # Square pressed
            self.nav2_client.cancel_goal_async()
        
        if msg.buttons[7] > 0.5: # D-pad Vertical Up
            twist = Twist()
            twist.angular.z += 0.1
            self.publisher.publish(twist)

        if msg.buttons[7] > -0.5: 
            twist = Twist()
            twist.angular.z -= 0.1
            self.publisher.publish(twist)
        
        if msg.buttons[6] > 0.5: # D-pad Horizontal Right
            twist = Twist()
            twist.angular.z -= 0.1
            self.publisher.publish(twist)

        if msg.buttons[6] > -0.5:
            twist = Twist()
            twist.angular.z -= 0.1
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerDualShock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

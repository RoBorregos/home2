import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RawSerialReceiver(Node):
    def __init__(self):
        super().__init__('raw_serial_receiver')

        # Configure serial port
        self.ser = serial.Serial('/dev/ttyOmniSTM32', 230400, timeout=0.2)  # match STM32 USART3

        # Publisher for raw UART RX data
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.raw_pub = self.create_publisher(String, '/uart_rx_raw', qos)

        # Start receiver thread
        self.recv_thread = threading.Thread(target=self.receiver, daemon=True)
        self.recv_thread.start()

    def receiver(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self.raw_pub.publish(String(data=line))
                    self.get_logger().debug(f"📥 UART RX: {line}")
            except Exception as e:
                self.get_logger().error(f"❌ UART RX Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RawSerialReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

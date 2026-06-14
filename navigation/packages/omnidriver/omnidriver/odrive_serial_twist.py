import math
import re
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
import serial


class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        # Serial
        self.ser = serial.Serial('/dev/ttyACM0', 230400, timeout=0.1)  # match STM32 USART3

        # TX command to STM32 UART_RX task:
        # expected format: "vx vy wz\r\n"
        self.count = 0
        self.cmd_names = ['cmd_vx', 'cmd_vy', 'cmd_wz']
        default_cmd = [0.0, 0.0, 0.0]

        for i, name in enumerate(self.cmd_names):
            self.declare_parameter(name, default_cmd[i])

        # Select whether cmd_vel is TwistStamped or Twist
        self.declare_parameter('use_stamped_cmd_vel', True)
        self.use_stamped = self.get_parameter('use_stamped_cmd_vel').value

        self.message_tx = [self.get_parameter(n).value for n in self.cmd_names]
        self.add_on_set_parameters_callback(self.update_parameters)

        # cmd_vel subscription
        if self.use_stamped:
            self.cmd_vel_sub = self.create_subscription(
                TwistStamped,
                'cmd_vel',
                self.cmd_vel_stamped_callback,
                10
            )
            self.get_logger().info('Using TwistStamped for cmd_vel')
        else:
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmd_vel_callback,
                10
            )
            self.get_logger().info('Using Twist for cmd_vel')

        # Periodic sender
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.send_message)

        # Debug publishers
        self.state_pub = self.create_publisher(String, 'odrive/debug', 10)
        self.raw_serial_pub = self.create_publisher(String, 'odrive/raw', 10)

        # Command echo / parsed values
        self.cmd_pub = self.create_publisher(Float32MultiArray, 'odrive/cmd_twist', 10)  # [vx, vy, wz]

        # IMU
        self.imu_pub = self.create_publisher(Float32MultiArray, 'odrive/imu_euler', 10)  # [yaw, roll, pitch]

        # IK
        self.ik_pub = self.create_publisher(Float32MultiArray, 'odrive/ik_wheel_speeds', 10)  # [u0, u1, u2, u3]

        # Odom / twist
        self.odom_pub = self.create_publisher(Float32MultiArray, 'odrive/odom', 10)              # [phi, x, y]
        self.body_twist_pub = self.create_publisher(Float32MultiArray, 'odrive/body_twist', 10)  # [wz, vx, vy]

        # Per-axis data
        self.node_id_pub = self.create_publisher(Int32MultiArray, 'odrive/node_ids', 10)
        self.axis_error_pub = self.create_publisher(Int32MultiArray, 'odrive/axis_errors', 10)
        self.axis_state_pub = self.create_publisher(Int32MultiArray, 'odrive/axis_states', 10)
        self.ctrl_status_pub = self.create_publisher(Int32MultiArray, 'odrive/controller_status', 10)
        self.updated_pub = self.create_publisher(Int32MultiArray, 'odrive/updated', 10)

        self.pos_pub = self.create_publisher(Float32MultiArray, 'odrive/pos_est', 10)
        self.vel_pub = self.create_publisher(Float32MultiArray, 'odrive/vel_est', 10)
        self.shadow_pub = self.create_publisher(Int32MultiArray, 'odrive/encoder_shadow', 10)
        self.cpr_pub = self.create_publisher(Int32MultiArray, 'odrive/encoder_cpr', 10)
        self.vbus_pub = self.create_publisher(Float32MultiArray, 'odrive/bus_voltage', 10)
        self.ibus_pub = self.create_publisher(Float32MultiArray, 'odrive/bus_current', 10)
        self.iq_set_pub = self.create_publisher(Float32MultiArray, 'odrive/iq_setpoint', 10)
        self.iq_meas_pub = self.create_publisher(Float32MultiArray, 'odrive/iq_measured', 10)

        # Reusable messages
        self.cmd_data = Float32MultiArray()
        self.imu_data = Float32MultiArray()
        self.ik_data = Float32MultiArray()
        self.odom_data = Float32MultiArray()
        self.body_twist_data = Float32MultiArray()

        self.node_ids_data = Int32MultiArray()
        self.axis_error_data = Int32MultiArray()
        self.axis_state_data = Int32MultiArray()
        self.ctrl_status_data = Int32MultiArray()
        self.updated_data = Int32MultiArray()

        self.pos_data = Float32MultiArray()
        self.vel_data = Float32MultiArray()
        self.shadow_data = Int32MultiArray()
        self.cpr_data = Int32MultiArray()
        self.vbus_data = Float32MultiArray()
        self.ibus_data = Float32MultiArray()
        self.iq_set_data = Float32MultiArray()
        self.iq_meas_data = Float32MultiArray()

        # Start RX thread
        self.recv_thread = threading.Thread(target=self.receiver, daemon=True)
        self.recv_thread.start()

    def log_cmd(self):
        self.get_logger().info(
            f"cmd_vel received: vx={self.message_tx[0]:.3f}, "
            f"vy={self.message_tx[1]:.3f}, "
            f"wz={self.message_tx[2]:.3f}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.message_tx[0] = msg.linear.x
        self.message_tx[1] = msg.linear.y
        self.message_tx[2] = msg.angular.z
        self.log_cmd()

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.message_tx[0] = msg.twist.linear.x
        self.message_tx[1] = msg.twist.linear.y
        self.message_tx[2] = msg.twist.angular.z
        self.log_cmd()

    def update_parameters(self, params):
        for param in params:
            if param.name in self.cmd_names:
                idx = self.cmd_names.index(param.name)
                self.message_tx[idx] = param.value
        return SetParametersResult(successful=True)

    def f(self, v):
        try:
            x = float(v)
            if not math.isfinite(x):
                return 0.0
            return x
        except Exception:
            return 0.0

    def i(self, v):
        try:
            x = int(float(v))
            return max(-2147483648, min(2147483647, x))
        except Exception:
            return 0

    def send_message(self):
        vx = round(float(self.message_tx[0]), 3)
        vy = round(float(self.message_tx[1]), 3)
        wz = round(float(self.message_tx[2]), 3)

        cmd_str = f"{vx} {vy} {wz}\r\n"
        self.ser.write(cmd_str.encode())
        self.get_logger().info(f"📤 [{self.count}] Sent: {repr(cmd_str)}")
        self.count += 1

    def receiver(self):
        while rclpy.ok():
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            self.raw_serial_pub.publish(String(data=line))

            try:
                # Accept ints and floats, positive/negative
                matches = re.findall(r'(\w+)=([-+]?\d*\.\d+|[-+]?\d+)', line)
                data = {key: float(val) for key, val in matches}

                output = (
                    f"Parsed ODrive State:\n"
                    f"  CMD:   vx={data.get('CMD_vx', 0.0):.3f}, "
                    f"vy={data.get('CMD_vy', 0.0):.3f}, "
                    f"wz={data.get('CMD_wz', 0.0):.3f}\n"
                    f"  IMU:   yaw={data.get('IMU_yaw', 0.0):.2f}, "
                    f"roll={data.get('IMU_roll', 0.0):.2f}, "
                    f"pitch={data.get('IMU_pitch', 0.0):.2f}\n"
                    f"  IK:    u0={data.get('IK_u0', 0.0):.3f}, "
                    f"u1={data.get('IK_u1', 0.0):.3f}, "
                    f"u2={data.get('IK_u2', 0.0):.3f}, "
                    f"u3={data.get('IK_u3', 0.0):.3f}\n"
                    f"  ODOM:  phi={data.get('ODOM_phi', 0.0):.3f}, "
                    f"x={data.get('ODOM_x', 0.0):.3f}, "
                    f"y={data.get('ODOM_y', 0.0):.3f}\n"
                    f"  TWIST: wz={data.get('ODOM_w', 0.0):.3f}, "
                    f"vx={data.get('ODOM_vx', 0.0):.3f}, "
                    f"vy={data.get('ODOM_vy', 0.0):.3f}\n"
                    f"  AX0: node={int(data.get('N0', 0))}, err={int(data.get('E0', 0))}, "
                    f"state={int(data.get('S0', 0))}, ctrl={int(data.get('C0', 0))}, "
                    f"pos={data.get('P0', 0.0):.3f}, vel={data.get('V0', 0.0):.3f}\n"
                    f"  AX1: node={int(data.get('N1', 0))}, err={int(data.get('E1', 0))}, "
                    f"state={int(data.get('S1', 0))}, ctrl={int(data.get('C1', 0))}, "
                    f"pos={data.get('P1', 0.0):.3f}, vel={data.get('V1', 0.0):.3f}\n"
                    f"  AX2: node={int(data.get('N2', 0))}, err={int(data.get('E2', 0))}, "
                    f"state={int(data.get('S2', 0))}, ctrl={int(data.get('C2', 0))}, "
                    f"pos={data.get('P2', 0.0):.3f}, vel={data.get('V2', 0.0):.3f}\n"
                    f"  AX3: node={int(data.get('N3', 0))}, err={int(data.get('E3', 0))}, "
                    f"state={int(data.get('S3', 0))}, ctrl={int(data.get('C3', 0))}, "
                    f"pos={data.get('P3', 0.0):.3f}, vel={data.get('V3', 0.0):.3f}"
                )
                self.state_pub.publish(String(data=output))

                # Command echo = [vx, vy, wz]
                self.cmd_data.data = [
                    self.f(data.get('CMD_vx', 0.0)),
                    self.f(data.get('CMD_vy', 0.0)),
                    self.f(data.get('CMD_wz', 0.0)),
                ]

                # IMU Euler = [yaw, roll, pitch]
                self.imu_data.data = [
                    self.f(data.get('IMU_yaw', 0.0)),
                    self.f(data.get('IMU_roll', 0.0)),
                    self.f(data.get('IMU_pitch', 0.0)),
                ]

                # IK wheel speeds = [u0, u1, u2, u3]
                self.ik_data.data = [
                    self.f(data.get('IK_u0', 0.0)),
                    self.f(data.get('IK_u1', 0.0)),
                    self.f(data.get('IK_u2', 0.0)),
                    self.f(data.get('IK_u3', 0.0)),
                ]

                # Odom = [phi, x, y]
                self.odom_data.data = [
                    self.f(data.get('ODOM_phi', 0.0)),
                    self.f(data.get('ODOM_x', 0.0)),
                    self.f(data.get('ODOM_y', 0.0)),
                ]

                # Body twist = [wz, vx, vy]
                self.body_twist_data.data = [
                    self.f(data.get('ODOM_w', 0.0)),
                    self.f(data.get('ODOM_vx', 0.0)),
                    self.f(data.get('ODOM_vy', 0.0)),
                ]

                self.node_ids_data.data = [self.i(data.get(f'N{i}', 0)) for i in range(4)]
                self.axis_error_data.data = [self.i(data.get(f'E{i}', 0)) for i in range(4)]
                self.axis_state_data.data = [self.i(data.get(f'S{i}', 0)) for i in range(4)]
                self.ctrl_status_data.data = [self.i(data.get(f'C{i}', 0)) for i in range(4)]
                self.updated_data.data = [self.i(data.get(f'U{i}', 0)) for i in range(4)]

                self.pos_data.data = [self.f(data.get(f'P{i}', 0.0)) for i in range(4)]
                self.vel_data.data = [self.f(data.get(f'V{i}', 0.0)) for i in range(4)]
                self.shadow_data.data = [self.i(data.get(f'Sh{i}', 0)) for i in range(4)]
                self.cpr_data.data = [self.i(data.get(f'CPR{i}', 0)) for i in range(4)]
                self.vbus_data.data = [self.f(data.get(f'Vbus{i}', 0.0)) for i in range(4)]
                self.ibus_data.data = [self.f(data.get(f'Ibus{i}', 0.0)) for i in range(4)]
                self.iq_set_data.data = [self.f(data.get(f'IqSet{i}', 0.0)) for i in range(4)]
                self.iq_meas_data.data = [self.f(data.get(f'IqMeas{i}', 0.0)) for i in range(4)]

                # Publish
                self.cmd_pub.publish(self.cmd_data)
                self.imu_pub.publish(self.imu_data)
                self.ik_pub.publish(self.ik_data)
                self.odom_pub.publish(self.odom_data)
                self.body_twist_pub.publish(self.body_twist_data)

                self.node_id_pub.publish(self.node_ids_data)
                self.axis_error_pub.publish(self.axis_error_data)
                self.axis_state_pub.publish(self.axis_state_data)
                self.ctrl_status_pub.publish(self.ctrl_status_data)
                self.updated_pub.publish(self.updated_data)

                self.pos_pub.publish(self.pos_data)
                self.vel_pub.publish(self.vel_data)
                self.shadow_pub.publish(self.shadow_data)
                self.cpr_pub.publish(self.cpr_data)
                self.vbus_pub.publish(self.vbus_data)
                self.ibus_pub.publish(self.ibus_data)
                self.iq_set_pub.publish(self.iq_set_data)
                self.iq_meas_pub.publish(self.iq_meas_data)

            except Exception as e:
                self.get_logger().error(f"❌ UART RX Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
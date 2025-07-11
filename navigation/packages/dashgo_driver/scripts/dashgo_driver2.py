#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int16, Int32, UInt16, Float32, String
from geometry_msgs.msg import Twist, Quaternion
from dashgo_driver.dashgo_stm32 import Stm32
from sensor_msgs.msg import Range, Imu
from math import pi as PI, degrees, radians, sin, cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

ODOM_POSE_COVARIANCE = [float(1e-3), 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, float(1e3)]

ODOM_POSE_COVARIANCE2 = [float(1e-9), 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, float(1e-9)]

ODOM_TWIST_COVARIANCE = [float(1e-3), 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, float(1e3)]

ODOM_TWIST_COVARIANCE2 = [float(1e-9), 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, float(1e-9)]

SERVO_MAX = 180
SERVO_MIN = 0


class DashgoDriver(Node):

    def __init__(self):
        super().__init__('dashgo_driver')
        ##PARAMETER DECLARATION
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.timeout = self.declare_parameter('timeout', 0.5).value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value  
        self.rate = self.declare_parameter('rate', 50).value
        self.useImu = self.declare_parameter("useImu", False).value
        self.wheel_diameter = self.declare_parameter("wheel_diameter", 0.1518).value
        self.wheel_track = self.declare_parameter("wheel_track", 0.375).value
        self.encoder_resolution = self.declare_parameter("encoder_resolution", 42760).value
        self.gear_reduction = self.declare_parameter("gear_reduction",1.0).value
        self.accel_limit = self.declare_parameter("accel_limit", 0.1).value
        self.encoder_min = self.declare_parameter("encoder_min",0).value
        self.start_rotation_limit_w = self.declare_parameter("start_rotation_limit_w",0.4).value
        self.encoder_max = self.declare_parameter("encoder_max",65535).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min ).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min ).value
        self.imu_frame_id = self.declare_parameter('imu_frame_id', 'imu_base').value
        self.imu_offset = self.declare_parameter('imu_offset', 1.01).value

        #Declare simple variables
        self.stopped = False
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
        self.bad_encoder_count = 0
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0
        self.now = self.get_clock().now()
        self.then = self.now
        self.cmd_vel = Twist()
        self.odomBroadcaster = TransformBroadcaster(self)
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = self.now
        self.SUCCESS = 0
        self.FAIL = -1
        self.voltage_val = 0
        self.stm32_version=0
        
        

        #CREATE PUBLISHERS
        self.create_subscription(Twist,'cmd_vel',self.cmdVelCallback,10)
        self.imuPub = self.create_publisher(Imu, 'imu', 5)
        self.imuAnglePub = self.create_publisher(Float32, 'imu_angle', 5)
        #self.odomPub = self.create_publisher(Odometry, 'odom', 5)
        self.odomPub = self.create_publisher(Odometry, 'dashgo_odom', 5)

        self.controller = Stm32(self.port, self.baud, self.timeout)
        self.controller.connect()
        self.get_logger().info("Connected to Stm32 on port " + self.port + " at " + str(self.baud) + " baud")
        self.controller.reset_encoders()
        self.controller.reset_IMU()
        _,stm32_hardware1,stm32_hardware0,stm32_software1,stm32_software0=self.controller.get_hardware_version()
        self.get_logger().info("*************************************************")
        self.get_logger().info("stm32 hardware_version is "+str(stm32_hardware0)+str(".")+str(stm32_hardware1))
        self.get_logger().info("stm32 software_version is "+str(stm32_software0)+str(".")+str(stm32_software1))
        self.get_logger().info("*************************************************")

        
        timer_period = 1 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.base_controller)

    def base_controller(self):
        print(f"voltage = {self.controller.get_voltage()}")

        self.now = self.get_clock().now()
        try:
            stat_, left_enc,right_enc = self.controller.get_encoder_counts()
        except:
            self.bad_encoder_count += 1
            self.get_logger().info("Encoder exception count: " + str(self.bad_encoder_count))
            return
        if (self.useImu == True) :
            try:
                stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = self.controller.get_imu_val()
                if yaw>=18000:
                    yaw = yaw-65535
                yaw = yaw/100.0
                if yaw_vel>=32768:
                    yaw_vel = yaw_vel-65535
                yaw_vel = yaw_vel/100.0
                #rospy.loginfo("yaw: " + str(yaw/100)+" yaw_vel: " + str(yaw_vel))     
                imu_data = Imu()  
                imu_data.header.stamp = self.get_clock().now().to_msg()
                imu_data.header.frame_id = self.imu_frame_id 
                imu_data.orientation_covariance[0] = 1000000
                imu_data.orientation_covariance[1] = 0
                imu_data.orientation_covariance[2] = 0
                imu_data.orientation_covariance[3] = 0
                imu_data.orientation_covariance[4] = 1000000
                imu_data.orientation_covariance[5] = 0
                imu_data.orientation_covariance[6] = 0
                imu_data.orientation_covariance[7] = 0
                imu_data.orientation_covariance[8] = 0.000001
                imu_quaternion = Quaternion()
                imu_quaternion.x = 0.0 
                imu_quaternion.y = 0.0
                imu_quaternion.z = sin(-1*self.imu_offset*yaw*3.1416/(180 *2.0))
                imu_quaternion.w = cos(-1*self.imu_offset*yaw*3.1416/(180 *2.0))
                imu_data.orientation = imu_quaternion
                imu_data.linear_acceleration_covariance[0] = -1
                imu_data.angular_velocity_covariance[0] = -1

                imu_data.angular_velocity.x = 0.0
                imu_data.angular_velocity.y = 0.0
                imu_data.angular_velocity.z = (yaw_vel*3.1416/(180*100))
                self.imuPub.publish(imu_data)
                angle = Float32()
                angle.data = -1.0*self.imu_offset*yaw*3.1416/(180 *2.0)
                self.imuAnglePub.publish(angle)

            except:
                self.bad_encoder_count += 1
                self.get_logger().info("IMU exception count: " + str(self.bad_encoder_count))
                return
            
        dt = self.now - self.then
        self.then = self.now
        dt = dt.nanoseconds / 1e9

        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            if (left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap) :
                self.l_wheel_mult = self.l_wheel_mult + 1     
            elif (left_enc > self.encoder_high_wrap and self.enc_left < self.encoder_low_wrap) :
                self.l_wheel_mult = self.l_wheel_mult - 1
            else:
                    self.l_wheel_mult = 0
            if (right_enc < self.encoder_low_wrap and self.enc_right > self.encoder_high_wrap) :
                self.r_wheel_mult = self.r_wheel_mult + 1     
            elif (right_enc > self.encoder_high_wrap and self.enc_right < self.encoder_low_wrap) :
                self.r_wheel_mult = self.r_wheel_mult - 1
            else:
                    self.r_wheel_mult = 0
            #dright = (right_enc - self.enc_right) / self.ticks_per_meter
            #dleft = (left_enc - self.enc_left) / self.ticks_per_meter
            dleft = 1.0 * (left_enc + self.l_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_left) / self.ticks_per_meter 
            dright = 1.0 * (right_enc + self.r_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_right) / self.ticks_per_meter 

        self.enc_right = right_enc
        self.enc_left = left_enc
        
        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt
            
        if (dxy_ave != 0):
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave
            self.x += (cos(self.th) * dx - sin(self.th) * dy)
            self.y += (sin(self.th) * dx + cos(self.th) * dy)

        if (dth != 0):
            self.th += dth 

        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        # Create the odometry transform frame broadcaster.
        if (self.useImu == False) :
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            #t.transform.translation.x = float(self.x) if self.x > 0.02 else 0.0
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion.x
            t.transform.rotation.y = quaternion.y
            t.transform.rotation.z = quaternion.z
            t.transform.rotation.w = quaternion.w
            t.child_frame_id = self.base_frame
            t.header.frame_id = 'odom'

            self.odomBroadcaster.sendTransform(t)

        odom = Odometry()
        #odom.header.frame_id = "odom"
        odom.header.frame_id = "dashgo_odom"
        odom.child_frame_id = self.base_frame
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth

        #odom.pose.covariance = ODOM_POSE_COVARIANCE
        #odom.twist.covariance = ODOM_TWIST_COVARIANCE
        odom.pose.covariance = ODOM_POSE_COVARIANCE2
        odom.twist.covariance = ODOM_TWIST_COVARIANCE2

        self.odomPub.publish(odom)
        
        if self.now > (self.last_cmd_vel + Duration(seconds=self.timeout)):
            self.v_des_left = 0
            self.v_des_right = 0
            
        if self.v_left < self.v_des_left:
            self.v_left += self.max_accel
            if self.v_left > self.v_des_left:
                self.v_left = self.v_des_left
        else:
            self.v_left -= self.max_accel
            if self.v_left < self.v_des_left:
                self.v_left = self.v_des_left
        
        if self.v_right < self.v_des_right:
            self.v_right += self.max_accel
            if self.v_right > self.v_des_right:
                self.v_right = self.v_des_right
        else:
            self.v_right -= self.max_accel
            if self.v_right < self.v_des_right:
                self.v_right = self.v_des_right
                  

        # Set motor speeds in encoder ticks per PID loop
        #if not self.stopped:
        if ((not self.stopped)):
            # print(f"vel l {self.v_left} and vel r {self.v_right}")
            self.controller.drive(int(self.v_left), int(self.v_right))


    def cmdVelCallback(self, req):
            self.last_cmd_vel = self.get_clock().now()
            x = req.linear.x         # m/s
            th = req.angular.z
            if x == 0:
                if th>0.0 and th<0.15:
                    th=0.15
                elif th>-0.15 and th<0.0:
                    th=-0.15
                right = th * self.wheel_track  * self.gear_reduction / 2.0
                left = -right
            elif th == 0:
                left = right = x
            else:
                if (th>0.0 and th<self.start_rotation_limit_w) and (x>-0.1 and x<0):
                    th=self.start_rotation_limit_w
                if (th<0.0 and th >-1.0*self.start_rotation_limit_w) and (x>-0.1 and x<0):
                    th=-1.0*self.start_rotation_limit_w


                left = x - th * self.wheel_track  * self.gear_reduction / 2.0
                right = x + th * self.wheel_track  * self.gear_reduction / 2.0
                
            self.v_des_left = int(left * self.ticks_per_meter / self.controller.PID_RATE)
            self.v_des_right = int(right * self.ticks_per_meter / self.controller.PID_RATE)






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DashgoDriver()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

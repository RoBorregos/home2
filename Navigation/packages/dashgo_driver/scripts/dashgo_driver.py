#!/usr/bin/env python3
from serial.serialutil import SerialException
from serial import Serial
import sys, traceback
import os
from math import pi as PI,sin, cos
import struct
import math
import binascii

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist,Quaternion,TransformStamped
from std_msgs.msg import Int16, Int32, UInt16, Float32, String
from sensor_msgs.msg import Range, Imu, PointCloud2
from nav_msgs.msg import Odometry
import tf2_ros 
import time as t

ODOM_POSE_COVARIANCE = [float(1e-3), 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, float(1e3)]
ODOM_TWIST_COVARIANCE = [float(1e-3), 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, float(1e-3), 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, float(1e6), 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, float(1e6), 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, float(1e6), 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, float(1e3)]
N_ANALOG_PORTS = 6
N_DIGITAL_PORTS = 12


class Stm32:
    
    def __init__(self,node, port, baudrate, timeout,stm32_timeout):
        # Initialize the values
        self.port = port
        self.stm32_timeout = stm32_timeout
        self.baudrate = baudrate
        self.timeout = timeout
        self.node = node
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Stm32 PID controller.
        self.PID_INTERVAL = 1000 / 30
        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0 = 0xff
        self.HEADER1 = 0xaa
        self.SUCCESS = 0
        self.FAIL = -1
        self.receive_state_ = self.WAITING_FF
        self.receive_check_sum_ = 0
        self.payload_command = b''
        self.payload_ack = b''
        self.payload_args = b''
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * N_ANALOG_PORTS
         # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * N_DIGITAL_PORTS

    def connect(self):
        try:
            self.node.get_logger().info(f"Connecting to Stm32 on port {self.port} ...")
            # self.port = Serial(port="/dev/port1", baudrate=115200, timeout=0.1, writeTimeout=0.1)
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            counter = 0
            t.sleep(1)
            state_, val = self.get_baud()
            while val!=self.baudrate:
                t.sleep(1)
                state_, val  = self.get_baud()   
                counter+=1
                if counter >= self.stm32_timeout:
                    raise SerialException
            self.node.get_logger().info("Connected at" + str(self.baudrate))
            self.node.get_logger().info("Stm32 is ready.")

        except SerialException:
            self.node.get_logger().error("Serial Exception:")
            self.node.get_logger().error(str(sys.exc_info()))
            self.node.get_logger().error("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            self.node.get_logger().error("Cannot connect to Stm32!")
            os._exit(1)
    def receiveFiniteStates(self, rx_data):
        if self.receive_state_ == self.WAITING_FF:
            #print str(binascii.b2a_hex(rx_data))
            if rx_data == b'\xff':
                self.receive_state_ = self.WAITING_AA
                self.receive_check_sum_ =0
                self.receive_message_length_ = 0
                self.byte_count_=0
                self.payload_ack = b''
                self.payload_args = b''
                self.payload_len = 0


        elif self.receive_state_ == self.WAITING_AA :
             if rx_data == b'\xaa':
                 self.receive_state_ = self.RECEIVE_LEN
                 self.receive_check_sum_ = 0
             else:
                 self.receive_state_ = self.WAITING_FF

        elif self.receive_state_ == self.RECEIVE_LEN:
             self.receive_message_length_, = struct.unpack("B",rx_data)
             self.receive_state_ = self.RECEIVE_PACKAGE
             self.receive_check_sum_ = self.receive_message_length_
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
             if self.byte_count_==0:
                 self.payload_ack = rx_data
             else:
                 self.payload_args += rx_data
             uc_tmp_, = struct.unpack("B",rx_data)
             self.receive_check_sum_ = self.receive_check_sum_ + uc_tmp_
             self.byte_count_ +=1
             #print "byte:"+str(byte_count_) +","+ "rece_len:"+str(receive_message_length_)
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            #print "checksun:" + str(rx_data) + " " + str(self.receive_check_sum_%255)
            #uc_tmp_, = struct.unpack("B",rx_data)
            #print "checksum:" + str(uc_tmp_) +" " + str((self.receive_check_sum_)%255)
            #if uc_tmp_ == (self.receive_check_sum_)%255:
            if 1:
                self.receive_state_ = self.WAITING_FF
                #print str(binascii.b2a_hex(value))
                #left, right, = struct.unpack('hh', value)
                #print "left:"+str(left)+", right:"+str(right)
                return 1 
            else:
                self.receive_state_ = self.WAITING_FF
        else:
            self.receive_state_ = self.WAITING_FF
        return 0
    
    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Stm32
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        #print str(binascii.b2a_hex(c))
        while self.receiveFiniteStates(c) != 1:
            c = self.port.read(1)
            #print str(binascii.b2a_hex(c))
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return 0
        return 1

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x00) + struct.pack("B", 0x01)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, 0
        
    def reset_IMU(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x41) + struct.pack("B", 0x42)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
    def start_automatic_recharge(self):
        ''' start for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x01, 0x00) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("start")
           return  self.SUCCESS
        else:
           return self.FAIL
    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1
    def get_firmware_version(self):
        ''' Get the current version of the firmware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x01) + struct.pack("B", 0x02)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1
    def get_pid(self, cmd):
        ''' Get the current value of the imu.
        '''
        check_number_list = [0x01, cmd]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, cmd) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val_l,val_r = struct.unpack('HH', self.payload_args)
           lreal=float(val_l)/100.0
           rreal=float(val_r)/100.0
           return  self.SUCCESS, lreal, rreal
        else:
           return self.FAIL, -1, -1
    def set_pid(self, cmd, left, right):
        ''' set pid.
        '''
        lpid = int(left*100)
        lpid_l = lpid & 0xff
        lpid_h = (lpid >> 8) & 0xff
        rpid = int(right*100)
        rpid_l = rpid & 0xff
        rpid_h = (rpid >> 8) & 0xff
        check_number_list = [0x05, cmd, lpid_h, lpid_l, rpid_h, rpid_l]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("8B", self.HEADER0, self.HEADER1, 0x05, cmd, lpid_h, lpid_l, rpid_h, rpid_l) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS 
        else:
           return self.FAIL
    def reset_encoders(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x03) + struct.pack("B", 0x04)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def get_check_sum(self,list):
        list_len = len(list)
        cs = 0
        for i in range(list_len):
            #print i, list[i]
            cs += list[i]
        cs=cs%255
        return cs

    def reset_system(self):
        ''' reset system.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x40) + struct.pack("B", 0x41)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_encoder_counts(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           left_enc, right_enc, = struct.unpack('HH', self.payload_args)
           return  self.SUCCESS, left_enc, right_enc
        else:
           return self.FAIL, 0, 0
    def get_infrareds(self):
        ''' Get the current distance on the infrareds.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0F) + struct.pack("B", 0x10)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0, val1, val2, val3, val4, val5 = struct.unpack('HHHHHH', self.payload_args)
           return  self.SUCCESS, val0, val1, val2, val3, val4, val5
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1
    
    def get_recharge_way(self):
        ''' Get the way of the recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x17) + struct.pack("B", 0x18)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #print("payload:"+str(binascii.b2a_hex(self.payload_args)))
           way, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, way
        else:
           return self.FAIL, -1
    def get_automatic_recharge_status(self):
        ''' Get the status of automatic recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x11) + struct.pack("B", 0x12)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return self.SUCCESS, val 
        else:
           return self.FAIL, -1
    def get_embtn_recharge(self):
        ''' Get the status of the emergency button and recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           em,rech = struct.unpack('HH', self.payload_args)
           return  self.SUCCESS, em, rech
        else:
           return self.FAIL, -1, -1
    def get_sonar_range(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0D) + struct.pack("B", 0x0E)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, sonar0, sonar1, sonar2, sonar3, sonar4, sonar5
        else:
           return self.FAIL, 0, 0, 0, 0, 0, 0
    def get_imu_val(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x05) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           yaw, yaw_vel, x_acc, y_acc, z_acc, = struct.unpack('5H', self.payload_args)
           return  self.SUCCESS, yaw, yaw_vel, x_acc, y_acc, z_acc
        else:
           return self.FAIL, 0, 0, 0, 0, 0
    def drive(self, left, right):
        data1 = struct.pack("h", int(left))
        d1, d2 = struct.unpack("BB", data1)

        data2 = struct.pack("h", int(right))
        c1, c2 = struct.unpack("BB", data2)

        self.check_list = [0x05,0x04, d1, d2, c1, c2]
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x04) + struct.pack("hh", int(left), int(right)) + struct.pack("B", self.check_num)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
    def get_voltage(self):
        ''' Get the current voltage the battery.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x12) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           vol1, vol2, vol3, vol4, vol5, vol6 = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, vol1, vol2, vol3, vol4, vol5, vol6
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1
    def stop_automatic_recharge(self):
        ''' stop for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x00, 0x00) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("stop")
           return  self.SUCCESS
        else:
           return self.FAIL
    def execute(self, cmd):
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd)
            res = self.recv(self.timeout)
            while attempts < ntries and res !=1 :
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    res = self.recv(self.timeout)
                    #print "response : " + str(binascii.b2a_hex(res))
                except:
                    self.node.get_logger().error("Exception executing command: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.node.get_logger().error("Exception executing command: " + str(binascii.b2a_hex(cmd)))
            return 0
        
        return 1

class BaseController:
    def __init__(
        self,node,rate, Stm32, base_frame, wheel_diameter,
        wheel_track, encoder_resolution, gear_reduction, accel_limit,
        motors_reversed, start_rotation_limit_w,use_smotheer,useImu,useSonar,
        encoder_min, encoder_max,sonar_height, sonar0_offset_yaw,sonar0_offset_x,sonar0_offset_y,
        sonar1_offset_yaw,sonar1_offset_x,sonar1_offset_y, sonar2_offset_yaw,sonar2_offset_x,
        sonar2_offset_y,sonar3_offset_yaw,sonar3_offset_x,sonar3_offset_y,sonar4_offset_yaw,
        sonar4_offset_x,sonar4_offset_y,imu_frame_id,imu_offset,PubEncoders,voltage_pub,
        show_statics,base_controller_timeout,encoder_high_wrap,encoder_low_wrap
        ):
        self.node = node
        self.timeout = base_controller_timeout
        self.encoder_high_wrap = encoder_high_wrap
        self.encoder_low_wrap = encoder_low_wrap
        self.voltage_pub = voltage_pub
        self.show_statics = show_statics
        self.useImu = useImu
        self.imu_frame_id = imu_frame_id
        self.sonar_height = sonar_height
        self.encoder_min=encoder_min
        self.encoder_max=encoder_max
        self.imu_offset = imu_offset
        self.PubEncoders = PubEncoders
        self.Stm32 = Stm32
        self.rate = rate
        self.useSonar = useSonar
        self.use_smotheer = use_smotheer
        self.base_frame = base_frame
        self.wheel_diameter = wheel_diameter
        self.wheel_track = wheel_track
        self.encoder_resolution = encoder_resolution
        self.gear_reduction = gear_reduction
        self.accel_limit = accel_limit
        self.motors_reversed = motors_reversed
        self.start_rotation_limit_w = start_rotation_limit_w
        self.sonar0_offset_yaw = sonar0_offset_yaw
        self.sonar0_offset_x = sonar0_offset_x
        self.sonar0_offset_y = sonar0_offset_y
        self.sonar1_offset_yaw = sonar1_offset_yaw
        self.sonar1_offset_x = sonar1_offset_x
        self.sonar1_offset_y = sonar1_offset_y
        self.sonar2_offset_yaw = sonar2_offset_yaw
        self.sonar2_offset_x = sonar2_offset_x
        self.sonar2_offset_y = sonar2_offset_y
        self.sonar3_offset_yaw = sonar3_offset_yaw
        self.sonar3_offset_x = sonar3_offset_x
        self.sonar3_offset_y = sonar3_offset_y
        self.sonar4_offset_yaw = sonar4_offset_yaw
        self.sonar4_offset_x = sonar4_offset_x
        self.sonar4_offset_y = sonar4_offset_y
        self.stopped = False
        self.bad_encoder_count = 0
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0
        self.SUCCESS = 0
        self.FAIL = -1

        now = self.node.get_clock().now()
        self.then = now
        self.t_delta = Duration(seconds=1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now
        self.lwheel_ele = 0
        self.rwheel_ele = 0
        self.recharge_way=0
        self.sonar_r0 =0.0
        self.sonar_r1 =0.0
        self.sonar_r2 =0.0
        self.sonar_r3 =0.0
        self.sonar_r4 =0.0
        self.safe_range_0 = 10
        self.safe_range_1 = 30
        self.sonar_maxval = 3.5
        self.sonar_cloud = [[100.0,0.105,0.1],[100.0,-0.105,0.1],[0.2,100.0,0.1],[0.2,-100.0,0.1],[-100.0,0.0,0.1]]
        self.voltage_val = 0
        self.voltage_str = ""
        self.emergencybt_val = 0
        self.is_recharge = False
        self.recharge_status = 0
        self.isPassed = True

        if (self.use_smotheer == True):
            self.robot_cmd_vel_pub = self.node.create_publisher(Twist, 'robot_cmd_vel', 5)
            self.node.create_subscription(Int16,'is_passed',self.isPassedCallback,10) # Probably not needed
            self.node.create_subscription(Twist,'smoother_cmd_vel',self.cmdVelCallback,10)
        else:
            self.node.create_subscription(Twist,'cmd_vel',self.cmdVelCallback,10)

        #RESET VALUES OF STM32
        self.Stm32.reset_encoders()
        self.Stm32.reset_IMU()

        #SONAR DECLARATION
        if (self.useSonar == True):
            self.sonar0_pub = self.node.create_publisher(Range, 'sonar0', 5)
            self.sonar1_pub = self.node.create_publisher(Range, 'sonar1', 5)
            self.sonar2_pub = self.node.create_publisher(Range, 'sonar2', 5)
            self.sonar3_pub = self.node.create_publisher(Range, 'sonar3', 5)
            self.sonar4_pub = self.node.create_publisher(Range, 'sonar4', 5)

            self.sonar_pub_cloud = self.node.create_publisher(PointCloud2, 'sonar_cloudpoint', 5)
           
            self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
            self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
            self.sonar_cloud[0][2] = self.sonar_height

            self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
            self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
            self.sonar_cloud[1][2] = self.sonar_height

            self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
            self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
            self.sonar_cloud[2][2] = self.sonar_height

            self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
            self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
            self.sonar_cloud[3][2] = self.sonar_height
            
            self.sonar_cloud[4][0] = self.sonar4_offset_x + self.sonar_maxval * math.cos(self.sonar4_offset_yaw)
            self.sonar_cloud[4][1] = self.sonar4_offset_y + self.sonar_maxval * math.sin(self.sonar4_offset_yaw)
            self.sonar_cloud[4][2] = self.sonar_height

        #IMU DECLARATION
        self.imuPub = self.node.create_publisher(Imu, 'imu', 5)
        self.imuAnglePub = self.node.create_publisher(Float32, 'imu_angle', 5)
        #Set up odometry broadcaster
        self.odomPub = self.node.create_publisher(Odometry, 'odom', 5)
        self.odomBroadcaster = tf2_ros.TransformBroadcaster(self.node)

        self.node.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.node.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

        if self.PubEncoders == True:
            self.lEncoderPub = self.node.create_publisher(UInt16, 'Lencoder', 5)
            self.rEncoderPub = self.node.create_publisher(UInt16, 'Rencoder', 5)
            self.lVelPub=self.node.create_publisher(Int16, 'Lvel', 5)
            self.rVelPub=self.node.create_publisher(Int16, 'Rvel', 5)
        
        
        
        if self.voltage_pub == True:
            self.voltagePub = self.node.create_publisher(Int32, 'voltage_value', 5)
            self.voltage_percentage_pub = self.node.create_publisher(Int32, 'voltage_percentage', 5)
            self.voltage_str_pub = self.node.create_publisher(String, 'voltage_str', 5)

        if self.show_statics == True:
            self.emergencybt_pub = self.node.create_publisher(Int16, 'emergencybt_status', 5)
            self.recharge_ir_pub = self.node.create_publisher(Int16, 'recharge_ir_status', 5)
            self.node.create_subscription(Int16,'recharge_handle',self.handleRechargeCallback,10)
            self.recharge_pub = self.node.create_publisher(Int16,'recharge_status',5)
            self.node.create_subscription(Int16,'ware_version_req',self.reqVersionCallback,10)
            self.version_pub = self.node.create_publisher(String,'ware_version',5)
            self.pid_p_pub = self.node.create_publisher(String,'pid_p',5)
            self.pid_i_pub = self.node.create_publisher(String,'pid_i',5)
            self.pid_d_pub = self.node.create_publisher(String,'pid_d',5)
            self.node.create_subscription(String,'pid_req', self.reqPidCallback,10)
            self.node.create_subscription( String, 'pid_set', self.reqSetPidCallback,10)
            self.recharge_way_pub = self.node.create_publisher(Int32,'recharge_way',5)
            self.lwheel_ele_pub = self.node.create_publisher(Int32,'lwheel_ele', 5)
            self.rwheel_ele_pub = self.node.create_publisher(Int32,'rwheel_ele', 5)
            self.ir0_pub = self.node.create_publisher(Int32,'ir0',5)
            self.ir1_pub = self.node.create_publisher(Int32,'ir1',5)
            self.ir2_pub = self.node.create_publisher(Int32,'ir2',5)
            self.ir3_pub = self.node.create_publisher(Int32,'ir3',5)
            self.ir4_pub = self.node.create_publisher(Int32,'ir4',5)
            self.ir5_pub = self.node.create_publisher(Int32,'ir5',5)


        self.node.create_subscription(Int16,'imu_reset',self.resetImuCallback,10)
        self.node.create_subscription(Int16,'encoder_reset',self.resetEncoderCallback,10)
        self.node.create_subscription(Int16,'system_reset',self.resetSystemCallback,10)


        self.stm32_version=0
        _,stm32_hardware1,stm32_hardware0,stm32_software1,stm32_software0=self.Stm32.get_hardware_version()
        self.node.get_logger().info("*************************************************")
        self.node.get_logger().info("stm32 hardware_version is "+str(stm32_hardware0)+str(".")+str(stm32_hardware1))
        self.node.get_logger().info("stm32 software_version is "+str(stm32_software0)+str(".")+str(stm32_software1))
        self.node.get_logger().info("*************************************************")



    def resetSystemCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_system()
                if res==self.FAIL:
                    self.node.get_logger().error("reset system failed ")
            except:
                    self.node.get_logger().error("request to reset system exception ")

    def resetEncoderCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_encoders()
                if res==self.FAIL:
                    self.node.get_logger().error("reset encoder failed ")
            except:
                self.node.get_logger().error("request to reset encoder exception ")

    def reqSetPidCallback(self, req):
        if req.data!='':
            set_list=req.data.split(",")
            if set_list[0]=='P':
                try:
                    res=self.Stm32.set_pid(0x06, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        self.node.get_logger().error("set the P of PID failed ")
                except:
                    self.node.get_logger().error("set the P of PID exception ")
            if set_list[0]=='I':
                try:
                    res=self.Stm32.set_pid(0x08, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        self.node.get_logger().error("set the I of PID failed ")
                except:
                    self.node.get_logger().error("set the I of PID exception ")
            if set_list[0]=='D':
                try:
                    res=self.Stm32.set_pid(0x0A, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        self.node.get_logger().error("set the D of PID failed ")
                except:
                    self.node.get_logger().error("set the D of PID exception ")

    def reqPidCallback(self, req):
        if req.data=='P':
            try:
                res,pl,pr = self.Stm32.get_pid(0x07)
                self.pid_p_pub.publish(str(pl) + "," + str(pr))
                if res==self.FAIL:
                    self.node.get_logger().error("request the P of PID failed ")
            except:
                    self.pid_p_pub.publish("")
                    self.node.get_logger().error("request the P of PID exception ")
        if req.data=='I':
            try:
                res,il,ir = self.Stm32.get_pid(0x09)
                self.pid_i_pub.publish(str(il) + "," + str(ir))
                if res==self.FAIL:
                    self.node.get_logger().error("request the I of PID failed ")
            except:
                    self.pid_i_pub.publish("")
                    self.node.get_logger().error("request the I of PID exception ")
        if req.data=='D':
            try:
                res,dl,dr = self.Stm32.get_pid(0x0B)
                self.pid_d_pub.publish(str(dl) + "," + str(dr))
                if res==self.FAIL:
                    self.node.get_logger().error("request the D of PID failed ")
            except:
                self.pid_d_pub.publish("")
                self.node.get_logger().error("request the D of PID exception ")

    def reqVersionCallback(self, req):
        if req.data==1:
            try:
                res,ver0,ver1,ver2,ver3 = self.Stm32.get_hardware_version()
                self.version_pub.publish(str(ver0)+"."+str(ver1)+"-"+str(ver2)+"."+str(ver3))
                if res==self.FAIL:
                    self.node.get_logger().error("request the version of hardware failed ")
            except:
                self.version_pub.publish("")
                self.node.get_logger().error("request the version of hardware exception ")
        if req.data==2:
            try:
                res,ver0,ver1,ver2,ver3 = self.Stm32.get_firmware_version()
                self.version_pub.publish(str(ver0)+"."+str(ver1)+"-"+str(ver2)+"."+str(ver3))
                if res==self.FAIL:
                    self.node.get_logger().error("request the version of firmware failed ")
            except:
                self.version_pub.publish("")
                self.node.get_logger().error("request the version of firmware exception ")

    def resetImuCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_imu()
                if res==self.FAIL:
                    self.node.get_logger().error("reset imu failed ")
            except:
                self.node.get_logger().error("request to reset imu exception ")

    def handleRechargeCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.start_automatic_recharge()
                self.is_recharge = True
            except:
                self.node.get_logger().error("start automatic recharge exception ")
        else:
            try:
                res = self.Stm32.stop_automatic_recharge()
                self.is_recharge = False
            except:
                self.node.get_logger().error("stop automatic recharge exception ")
            
    # Probably not needed 
    def isPassedCallback(self, msg):
        if(msg.data>2):
            self.isPassed = False
        else:
            self.isPassed = True

    #Only for smoother
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = self.node.get_clock().now()
        
        robot_cmd_vel = Twist()
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if self.emergencybt_val==1:
            robot_cmd_vel.linear.x = 0.0
            robot_cmd_vel.linear.y = 0.0
            robot_cmd_vel.angular.z = 0.0
        else:
            robot_cmd_vel.linear.x = x
            robot_cmd_vel.linear.y = 0.0
            robot_cmd_vel.angular.z = th
        
        if (self.use_smotheer == True):
            self.robot_cmd_vel_pub.publish(robot_cmd_vel)

        if not self.isPassed and x>0 :
            x = 0

        if (self.useSonar == True) :
            #sonar0
            if((self.sonar_r0<=self.safe_range_0 and self.sonar_r0>=2) and (x<0)):
                x= 0.0
                self.node.get_logger().warn("sonar0 smaller than safe_range_0, cannot back")
            #sonar1
            if((self.sonar_r1<=self.safe_range_0 and self.sonar_r1>=2) and (x>0)):
                x=0.0
                th=0.2
                self.node.get_logger().warn("sonar1 smaller than safe_range_0, only trun left")
            
            if((self.sonar_r1<=self.safe_range_0 and self.sonar_r1>=2) and (th<0)):
                x=0.0
                th=0.2
            #sonar2
            if((self.sonar_r2<=self.safe_range_0 and self.sonar_r2>=2) and (x>0)):
                x=0.0
                th=0.2
                self.node.get_logger().warn("sonar2 smaller than safe_range_0, only trun left")

            #sonar3
            if((self.sonar_r3<=self.safe_range_0 and self.sonar_r3>=2) and (x>0)):
                x=0.0
                th=-0.2
                self.node.get_logger().warn("sonar3 smaller than safe_range_0, only trun left")

            if((self.sonar_r3<=self.safe_range_0 and self.sonar_r3>=2) and (th>0)):
                x=0.0
                th=-0.2
            #sonar4
            if((self.sonar_r4<=self.safe_range_0 and self.sonar_r0>=2) and (x<0)):
                x= 0.0
                self.node.get_logger().warn("sonar4 smaller than safe_range_0, cannot back")



        if x == 0:
            # Turn in place
            if th>0.0 and th<0.15:
                th=0.15
            elif th>-0.15 and th<0.0:
                th=-0.15
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            if (th>0.0 and th<self.start_rotation_limit_w) and (x>-0.1 and x<0):
                th=self.start_rotation_limit_w
            if (th<0.0 and th >-1.0*self.start_rotation_limit_w) and (x>-0.1 and x<0):
                th=-1.0*self.start_rotation_limit_w


            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.Stm32.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.Stm32.PID_RATE)

    def volTransPerentage(self, vo):
         if(vo == -1):
             return -1;
         if(vo>4.2*7*1000):
             COUNT = 10*1000
         else:
             COUNT = 7*1000

         if(vo >= 4.0*COUNT):
             return 100
         elif(vo >= 3.965*COUNT):
             return 95
         elif(vo >= 3.93*COUNT):
             return 90
         elif(vo >= 3.895*COUNT):
             return 85
         elif(vo >= 3.86*COUNT):
             return 80
         elif(vo >= 3.825*COUNT):
             return 75
         elif(vo >= 3.79*COUNT):
             return 70
         elif(vo >= 3.755*COUNT):
             return 65
         elif (vo >= 3.72*COUNT):
             return 60
         elif (vo >= 3.685*COUNT):
             return 55
         elif (vo >= 3.65*COUNT):
             return 50
         elif (vo >= 3.615*COUNT):
             return 45
         elif (vo >= 3.58*COUNT):
             return 40
         elif (vo >= 3.545*COUNT):
             return 35
         elif (vo >= 3.51*COUNT):
             return 30
         elif (vo >= 3.475*COUNT):
             return 25
         elif (vo >= 3.44*COUNT):
             return 20
         elif (vo >= 3.405*COUNT):
             return 15
         elif (vo >= 3.37*COUNT):
             return 10
         elif (vo >= 3.335*COUNT):
             return 5
         else:
             return 0

    def poll(self):
        now = self.node.get_clock().now()
        if now >= self.t_next:
            try:
                stat_, left_enc,right_enc = self.Stm32.get_encoder_counts() 
                if self.PubEncoders == True:
                    self.lEncoderPub.publish(left_enc)
                    self.rEncoderPub.publish(right_enc)
            except:
                self.bad_encoder_count += 1
                self.node.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
                return
            try:
                res,vol1,vol2,vol3,vol4,vol5,vol6 = self.Stm32.get_voltage()
                self.lwheel_ele = vol3*4
                self.rwheel_ele = vol4*4
                self.voltage_val = vol5

                if self.show_statics == True:
                    self.lwheel_ele_pub.publish(self.lwheel_ele)
                    self.rwheel_ele_pub.publish(self.rwheel_ele)
                if self.voltage_pub == True:
                    self.voltagePub.publish(self.voltage_val)
                    self.voltage_str_pub.publish(str(vol1) + "," + str(vol2) + "," + str(vol3) + "," + str(vol4) + "," + str(vol5) + "," + str(vol6))
                    self.voltage_percentage_pub.publish(self.volTransPerentage(self.voltage_val))
            except:
                if self.voltage_pub == True:
                    self.voltagePub.publish(-1)
                    self.voltage_str_pub.publish("")

                if self.show_statics == True:
                    self.lwheel_ele_pub.publish(-1)
                    self.rwheel_ele_pub.publish(-1)

                self.node.get_logger().error("get voltage exception")
            try:
                res,ir1,ir2,ir3,ir4,ir5,ir6 = self.Stm32.get_infrareds()
                if self.show_statics == True:
                    self.ir0_pub.publish(ir1)
                    self.ir1_pub.publish(ir2)
                    self.ir2_pub.publish(ir3)
                    self.ir3_pub.publish(ir4)
                    self.ir4_pub.publish(ir5)
                    self.ir5_pub.publish(ir6)
            except:
                if self.show_statics == True:
                    self.ir0_pub.publish(-1)
                    self.ir1_pub.publish(-1)
                    self.ir2_pub.publish(-1)
                    self.ir3_pub.publish(-1)
                    self.ir4_pub.publish(-1)
                    self.ir5_pub.publish(-1)
                self.node.get_logger().error("get infrared ray exception")
            try:
                res,way = self.Stm32.get_recharge_way()
                self.recharge_way = way
                if self.show_statics == True:
                    self.recharge_way_pub.publish(self.recharge_way)
            except:
                if self.show_statics == True:
                    self.recharge_way_pub.publish(-1)
                self.node.get_logger().error("get recharge way exception")
            try:
                res,status = self.Stm32.get_automatic_recharge_status()
                self.recharge_status = status
                if self.recharge_status == 3:
                    self.is_recharge = False
                if self.show_statics == True:
                    self.recharge_pub.publish(self.recharge_status)
            except:
                if self.show_statics == True:
                    self.recharge_pub.publish(-1)
                self.node.get_logger().error("get recharge status exception")
            if(not self.is_recharge):
                try:
                    res,eme_val,rech_val  = self.Stm32.get_embtn_recharge()
                    self.emergencybt_val = eme_val
                    if self.show_statics == True:
                        self.emergencybt_pub.publish(eme_val)
                        self.recharge_ir_pub.publish(rech_val)
                except:
                    if self.show_statics == True:
                        self.emergencybt_val = -1
                        self.emergencybt_pub.publish(-1)
                        self.recharge_ir_pub.publish(-1)
                    self.node.get_logger().error("get emergency button exception")
            if (self.useSonar == True) :
                pcloud = PointCloud2()
                try:
                    stat_, self.sonar_r0, self.sonar_r1, self.sonar_r2, self.sonar_r3, self.sonar_r4,_ = self.Stm32.get_sonar_range()
                    sonar0_range = Range()
                    sonar0_range.header.stamp = now
                    sonar0_range.header.frame_id = "/sonar0"
                    sonar0_range.radiation_type = Range.ULTRASOUND
                    sonar0_range.field_of_view = 0.3
                    sonar0_range.min_range = 0.04
                    sonar0_range.max_range = 0.8
                    sonar0_range.range = self.sonar_r0/100.0

                    if sonar0_range.range == 0.0:  #sonar0 error or not exist flag
                        sonar0_range.range=1.0
                    elif sonar0_range.range>=sonar0_range.max_range:
                        sonar0_range.range = sonar0_range.max_range
                    self.sonar0_pub.publish(sonar0_range)
                    if sonar0_range.range>=0.5 or sonar0_range.range == 0.0:
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
                    else: 
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + sonar0_range.range * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + sonar0_range.range * math.sin(self.sonar0_offset_yaw)


                    sonar1_range = Range()
                    sonar1_range.header.stamp = now
                    sonar1_range.header.frame_id = "/sonar1"
                    sonar1_range.radiation_type = Range.ULTRASOUND
                    sonar1_range.field_of_view = 0.3
                    sonar1_range.min_range = 0.04
                    sonar1_range.max_range = 0.8 
                    sonar1_range.range = self.sonar_r1/100.0
                   # if sonar1_range.range>=sonar0_range.max_range or sonar1_range.range == 0.0:
                    if sonar1_range.range == 0.0:  #sonar1 error or not exist flag
                        sonar1_range.range=1.0
                    elif sonar1_range.range>=sonar0_range.max_range:
                        sonar1_range.range = sonar1_range.max_range
                    self.sonar1_pub.publish(sonar1_range) 
                    if sonar1_range.range>=0.5 or sonar1_range.range == 0.0:
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
                    else: 
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + sonar1_range.range * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + sonar1_range.range * math.sin(self.sonar1_offset_yaw)

                    sonar2_range = Range()
                    sonar2_range.header.stamp = now
                    sonar2_range.header.frame_id = "/sonar2"
                    sonar2_range.radiation_type = Range.ULTRASOUND
                    sonar2_range.field_of_view = 0.3
                    sonar2_range.min_range = 0.04
                    sonar2_range.max_range = 0.8
                    sonar2_range.range = self.sonar_r2/100.0
                    #if sonar2_range.range>=sonar2_range.max_range or sonar2_range.range == 0.0:
                    if sonar2_range.range == 0.0:  #sonar2 error or not exist flag
                        sonar2_range.range=1.0
                    elif sonar2_range.range>=sonar2_range.max_range :
                        sonar2_range.range = sonar2_range.max_range
                    self.sonar2_pub.publish(sonar2_range)
                    if sonar2_range.range>=0.5 or sonar2_range.range == 0.0:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
                    else:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + sonar2_range.range * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + sonar2_range.range * math.sin(self.sonar2_offset_yaw)

                    sonar3_range = Range()
                    sonar3_range.header.stamp = now
                    sonar3_range.header.frame_id = "/sonar3"
                    sonar3_range.radiation_type = Range.ULTRASOUND
                    sonar3_range.field_of_view = 0.3
                    sonar3_range.min_range = 0.04
                    sonar3_range.max_range = 0.8
                    sonar3_range.range = self.sonar_r3/100.0
                    #if sonar3_range.range>=sonar3_range.max_range or sonar3_range.range == 0.0:
                    if sonar3_range.range == 0.0:  #sonar3 error or not exist flag
                        sonar3_range.range=1.0
                    elif sonar3_range.range>=sonar3_range.max_range :
                        sonar3_range.range = sonar3_range.max_range
                    self.sonar3_pub.publish(sonar3_range)
                    if sonar3_range.range>=0.5 or sonar3_range.range == 0.0:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
                    else:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + sonar3_range.range * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + sonar3_range.range * math.sin(self.sonar3_offset_yaw)

                    sonar4_range = Range()
                    sonar4_range.header.stamp = now
                    sonar4_range.header.frame_id = "/sonar4"
                    sonar4_range.radiation_type = Range.ULTRASOUND
                    sonar4_range.field_of_view = 0.3
                    sonar4_range.min_range = 0.04
                    sonar4_range.max_range = 0.8
                    sonar4_range.range = self.sonar_r4/100.0
                    #if sonar4_range.range>=sonar4_range.max_range or sonar4_range.range == 0.0:
                    if sonar4_range.range == 0.0:  #sonar4 error or not exist flag
                        sonar4_range.range=1.0
                    elif sonar4_range.range>=sonar4_range.max_range :
                        sonar4_range.range = sonar4_range.max_range
                    self.sonar4_pub.publish(sonar4_range)
                    if sonar4_range.range>=0.5 or sonar4_range.range == 0.0:
                        self.sonar_cloud[4][0] = self.sonar4_offset_x + self.sonar_maxval * math.cos(self.sonar4_offset_yaw)
                        self.sonar_cloud[4][1] = self.sonar4_offset_y + self.sonar_maxval * math.sin(self.sonar4_offset_yaw)
                    else:
                        self.sonar_cloud[4][0] = self.sonar4_offset_x + sonar4_range.range * math.cos(self.sonar4_offset_yaw)
                        self.sonar_cloud[4][1] = self.sonar4_offset_y + sonar4_range.range * math.sin(self.sonar4_offset_yaw)

                    pcloud.header.frame_id="/base_footprint"
                    pcloud = PointCloud2.create_cloud_xyz32(pcloud.header, self.sonar_cloud)
                    self.sonar_pub_cloud.publish(pcloud)
                except:
                    self.node.get_logger().error("Get Sonar exception")
                    return
            if (self.useImu == True):
                try:
                    stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = self.Stm32.get_imu_val() 
                    if yaw>=18000:
                        yaw = yaw-65535
                    yaw = yaw/100.0
                    if yaw_vel>=32768:
                        yaw_vel = yaw_vel-65535
                    yaw_vel = yaw_vel/100.0
                    imu_data = Imu()  
                    imu_data.header.stamp = self.node.get_clock().now().to_msg()
                    imu_data.header.frame_id = self.imu_frame_id 
                    imu_data.orientation_covariance[0] = 1000000.0
                    imu_data.orientation_covariance[1] = 0.0
                    imu_data.orientation_covariance[2] = 0.0
                    imu_data.orientation_covariance[3] = 0.0
                    imu_data.orientation_covariance[4] = 1000000.0
                    imu_data.orientation_covariance[5] = 0.0
                    imu_data.orientation_covariance[6] = 0.0
                    imu_data.orientation_covariance[7] = 0.0
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
                    messageFl32 = Float32()
                    messageFl32.data = -1*self.imu_offset*yaw*3.1416/(180 *2.0)
                    self.imuAnglePub.publish(messageFl32)
                except Exception as e:
                    self.bad_encoder_count += 1
                    self.node.get_logger().error("IMU exception count: " + str(self.bad_encoder_count))
                    self.node.get_logger().error(str(e))
                    return
                
            dt = now - self.then
            self.then = now
            dt = dt.nanoseconds * 1e-9
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

            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = "odom"
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion.x
            t.transform.rotation.y = quaternion.y
            t.transform.rotation.z = quaternion.z
            t.transform.rotation.w = quaternion.w

            if (self.useImu == False) :
                self.odomBroadcaster.sendTransform(t)
                
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.pose.pose.position.x = float(self.x)
            odom.pose.pose.position.y = float(self.y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = vth
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE
            self.odomPub.publish(odom)
            if now > (self.last_cmd_vel + Duration(seconds=self.timeout)):
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
            if self.PubEncoders == True:
                self.lVelPub.publish(self.v_left)
                self.rVelPub.publish(self.v_right)  
            if ((not self.stopped) and (not self.is_recharge)):
                self.Stm32.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta

class Stm32ROS(Node):

    def __init__(self):
        super().__init__('DashgoDriver')
        #Declare all parameters
        self.declare_parameter("port", "/dev/ttyUSB0") 
        self.declare_parameter("baudrate", 115200) 
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("base_frame", 'base_footprint')
        self.declare_parameter("rate", 50)
        self.declare_parameter("sensorstate_rate", 10)
        self.declare_parameter("use_base_controller", True)
        self.declare_parameter("stm32_timeout", 3)
        ##Base controller parameters
        self.declare_parameter("base_controller_rate", 10) #self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.declare_parameter("base_controller_timeout", 1.0)
        self.declare_parameter("useImu", False)
        self.declare_parameter("useSonar", False)
        self.declare_parameter("wheel_diameter", 0.1518)
        self.declare_parameter("wheel_track", 0.375)
        self.declare_parameter("encoder_resolution", 42760)
        self.declare_parameter("gear_reduction", 1.0)
        self.declare_parameter("accel_limit", 1.0)
        self.declare_parameter("motors_reversed", False)
        self.declare_parameter("start_rotation_limit_w", 0.4)
        self.declare_parameter("encoder_min", 0)
        self.declare_parameter("encoder_max", 65535)
        self.declare_parameter("use_smotheer",False)
        self.declare_parameter("sonar_height",0.15)
        self.declare_parameter("sonar0_offset_yaw",0.0)
        self.declare_parameter("sonar0_offset_x",0.27)
        self.declare_parameter("sonar0_offset_y",0.19)
        self.declare_parameter("sonar1_offset_yaw",0.0)
        self.declare_parameter("sonar1_offset_x",0.27)
        self.declare_parameter("sonar1_offset_y",-0.19)
        self.declare_parameter("sonar2_offset_yaw",1.57)
        self.declare_parameter("sonar2_offset_x",0.24)
        self.declare_parameter("sonar2_offset_y",0.15)
        self.declare_parameter("sonar3_offset_yaw",-1.57)
        self.declare_parameter("sonar3_offset_x",0.24)
        self.declare_parameter("sonar3_offset_y",-0.15)
        self.declare_parameter("sonar4_offset_yaw",3.14)
        self.declare_parameter("sonar4_offset_x",-0.1)
        self.declare_parameter("sonar4_offset_y",0.0)
        self.declare_parameter("imu_frame_id",'imu_base')
        self.declare_parameter("imu_offset",1.01)
        self.declare_parameter("pub_encoders",False)
        self.declare_parameter("voltage_pub",False)
        self.declare_parameter("show_statics",False)  
        self.encoder_min = self.get_parameter('encoder_min').get_parameter_value().integer_value
        self.encoder_max = self.get_parameter('encoder_max').get_parameter_value().integer_value
        self.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )  
        self.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )  

        #Initial Setup
        
        #Rate reference
        r = self.get_parameter('rate').get_parameter_value().integer_value
        hz = 1.0/r
        self.timer = self.create_timer(hz, self.timer_callback)

        #Sensor Rate reference
        self.sensorstate_rate = self.get_parameter('sensorstate_rate').get_parameter_value().integer_value
        now = self.get_clock().now()
        t_rate = (1.0 / self.sensorstate_rate)
        self.t_delta_sensors = Duration(seconds=t_rate)
        self.t_next_sensors = now + self.t_delta_sensors

        #Starting STM32 instance
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.stm32_timeout = self.get_parameter('stm32_timeout').get_parameter_value().integer_value
        self.controller = Stm32(self,self.port, self.baud, self.timeout, self.stm32_timeout)
        
        #Start STM32 connection
        self.controller.connect()
        self.get_logger().info("Connected to Stm32 on port " + self.port + " at " + str(self.baud) + " baud")

        #Start base controller 
        self.use_base_controller = self.get_parameter('use_base_controller').get_parameter_value().bool_value

        if self.use_base_controller: 
            self.base_controller_rate = self.get_parameter('base_controller_rate').get_parameter_value().integer_value
            self.useImu = self.get_parameter('useImu').get_parameter_value().bool_value
            self.useSonar = self.get_parameter('useSonar').get_parameter_value().bool_value
            self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
            self.wheel_track = self.get_parameter('wheel_track').get_parameter_value().double_value
            self.encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value
            self.gear_reduction = self.get_parameter('gear_reduction').get_parameter_value().double_value
            self.accel_limit = self.get_parameter('accel_limit').get_parameter_value().double_value
            self.motors_reversed = self.get_parameter('motors_reversed').get_parameter_value().bool_value
            self.start_rotation_limit_w = self.get_parameter('start_rotation_limit_w').get_parameter_value().double_value
            self.use_smotheer = self.get_parameter('use_smotheer').get_parameter_value().bool_value
            self.PubEncoders = self.get_parameter('pub_encoders').get_parameter_value().bool_value
            self.voltage_pub = self.get_parameter('voltage_pub').get_parameter_value().bool_value
            self.show_statics = self.get_parameter('show_statics').get_parameter_value().bool_value
            self.encoder_min = self.get_parameter('encoder_min').get_parameter_value().integer_value
            self.encoder_max = self.get_parameter('encoder_max').get_parameter_value().integer_value
            self.sonar_height = self.get_parameter('sonar_height').get_parameter_value().double_value
            self.sonar0_offset_yaw = self.get_parameter('sonar0_offset_yaw').get_parameter_value().double_value
            self.sonar0_offset_x = self.get_parameter('sonar0_offset_x').get_parameter_value().double_value
            self.sonar0_offset_y = self.get_parameter('sonar0_offset_y').get_parameter_value().double_value
            self.sonar1_offset_yaw = self.get_parameter('sonar1_offset_yaw').get_parameter_value().double_value
            self.sonar1_offset_x = self.get_parameter('sonar1_offset_x').get_parameter_value().double_value
            self.sonar1_offset_y = self.get_parameter('sonar1_offset_y').get_parameter_value().double_value
            self.sonar2_offset_yaw = self.get_parameter('sonar2_offset_yaw').get_parameter_value().double_value
            self.sonar2_offset_x = self.get_parameter('sonar2_offset_x').get_parameter_value().double_value
            self.sonar2_offset_y = self.get_parameter('sonar2_offset_y').get_parameter_value().double_value
            self.sonar3_offset_yaw = self.get_parameter('sonar3_offset_yaw').get_parameter_value().double_value
            self.sonar3_offset_x = self.get_parameter('sonar3_offset_x').get_parameter_value().double_value
            self.sonar3_offset_y = self.get_parameter('sonar3_offset_y').get_parameter_value().double_value
            self.sonar4_offset_yaw = self.get_parameter('sonar4_offset_yaw').get_parameter_value().double_value
            self.sonar4_offset_x = self.get_parameter('sonar4_offset_x').get_parameter_value().double_value
            self.sonar4_offset_y = self.get_parameter('sonar4_offset_y').get_parameter_value().double_value
            self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value
            self.imu_offset = self.get_parameter('imu_offset').get_parameter_value().double_value
            self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
            self.base_controller_timeout = self.get_parameter('base_controller_timeout').get_parameter_value().double_value
            self.encoder_low_wrap = self.get_parameter('wheel_low_wrap').get_parameter_value().double_value
            self.encoder_high_wrap = self.get_parameter('wheel_high_wrap').get_parameter_value().double_value
            
            self.myBaseController = BaseController(
                self,self.base_controller_rate,self.controller,self.base_frame,self.wheel_diameter,
                self.wheel_track,self.encoder_resolution,self.gear_reduction,self.accel_limit,
                self.motors_reversed,self.start_rotation_limit_w,self.use_smotheer,self.useImu,
                self.useSonar,self.encoder_min, self.encoder_max, self.sonar_height,
                self.sonar0_offset_yaw,self.sonar0_offset_x,self.sonar0_offset_y, self.sonar1_offset_yaw,
                self.sonar1_offset_x,self.sonar1_offset_y,self.sonar2_offset_yaw,self.sonar2_offset_x,
                self.sonar2_offset_y,self.sonar3_offset_yaw,self.sonar3_offset_x,self.sonar3_offset_y,
                self.sonar4_offset_yaw,self.sonar4_offset_x,self.sonar4_offset_y,self.imu_frame_id,
                self.imu_offset,self.PubEncoders,self.voltage_pub,self.show_statics,self.base_controller_timeout,
                self.encoder_high_wrap ,self.encoder_low_wrap
                 )
        



    def timer_callback(self):
        if(self.use_base_controller):
            self.myBaseController.poll()
        else:
            self.get_logger().info("Base controller is not enabled")



def main():
    rclpy.init()

    Stm32Node = Stm32ROS()

    try:
        rclpy.spin(Stm32Node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
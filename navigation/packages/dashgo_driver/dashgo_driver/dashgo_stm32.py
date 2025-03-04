import _thread
from serial import Serial
import time
from serial.serialutil import SerialException
import sys, traceback
import os
import struct
import binascii
 

class Stm32:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12


    
    def __init__(self, port="/dev/port", baudrate=115200, timeout=0.5):
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Stm32 PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

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
    
        # Keep things thread safe
        self.mutex = _thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print("Connecting to Stm32 on port", self.port, "...")
            # self.port = Serial(port="/dev/port1", baudrate=115200, timeout=0.1, writeTimeout=0.1)
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            state_, val = self.get_baud()
            if val != self.baudrate:
                time.sleep(1)
                state_, val  = self.get_baud()   
                if val != self.baudrate:
                    raise SerialException
            print("Connected at", self.baudrate)
            print("Stm32 is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Stm32!")
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)


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
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Stm32 returning a single integer value.
        '''
        self.mutex.acquire()
        
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
                    print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
            return 0
        
        self.mutex.release()
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

    def get_encoder_counts(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           left_enc, right_enc, = struct.unpack('HH', self.payload_args)
           return  self.SUCCESS, left_enc, right_enc
        else:
           return self.FAIL, 0, 0


    def get_sonar_range(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0D) + struct.pack("B", 0x0E)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, sonar0, sonar1, sonar2, sonar3, sonar4, sonar5
        else:
           return self.FAIL, 0, 0, 0, 0, 0, 0

    def get_ir_range(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0F) + struct.pack("B", 0x10)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           ir0, ir1, ir2, ir3, ir4, ir5 = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, ir0, ir1, ir2, ir3, ir4, ir5
        else:
           return self.FAIL, 0, 0, 0, 0, 0, 0, 0

    def reset_IMU(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x41) + struct.pack("B", 0x42)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_imu_val(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x05) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           yaw, yaw_vel, x_acc, y_acc, z_acc, = struct.unpack('5H', self.payload_args)
           return  self.SUCCESS, yaw, yaw_vel, x_acc, y_acc, z_acc
        else:
           return self.FAIL, 0, 0, 0, 0, 0


    def get_emergency_button(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           emergency_state, _, = struct.unpack('2H', self.payload_args)
           return  self.SUCCESS, emergency_state
        else:
           return self.FAIL, 0

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

    def drive(self, left, right):
        data1 = struct.pack("h", left)
        d1, d2 = struct.unpack("BB", data1)

        data2 = struct.pack("h", right)
        c1, c2 = struct.unpack("BB", data2)

        self.check_list = [0x05,0x04, d1, d2, c1, c2]
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x04) + struct.pack("hh", left, right) + struct.pack("B", self.check_num)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def get_firmware_version(self):
        ''' Get the current version of the firmware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x01) + struct.pack("B", 0x02)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1

    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
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

    def get_infrared(self, ir_id):
        ir_list = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]
        check_number_list = [0x03, 0x0E, ir_list[ir_id], 0x00]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x0E, ir_list[ir_id], 0x00) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
            num,val = struct.unpack('HH', self.payload_args)
            return  self.SUCCESS, num, val
        else:
            return self.FAIL, ir_id, -1

    def get_infrareds(self):
        ''' Get the current distance on the infrareds.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0F) + struct.pack("B", 0x10)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0, val1, val2, val3, val4, val5 = struct.unpack('HHHHHH', self.payload_args)
           return  self.SUCCESS, val0, val1, val2, val3, val4, val5
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1

    def get_voltage(self):
        ''' Get the current voltage the battery.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x12) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           vol1, vol2, vol3, vol4, vol5, vol6 = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, vol1, vol2, vol3, vol4, vol5, vol6
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1


    def start_automatic_recharge(self):
        ''' start for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x01, 0x00) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("start")
           return  self.SUCCESS
        else:
           return self.FAIL

    def stop_automatic_recharge(self):
        ''' stop for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x00, 0x00) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("stop")
           return  self.SUCCESS
        else:
           return self.FAIL

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

    def reset_system(self):
        ''' reset system.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x40) + struct.pack("B", 0x41)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

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

import time
from constants import pwm_max,pwm_min
import struct
import serial
import math
# 定义常量
ACC_RATIO = (2*9.8/32768)
GYRO_RATIO = ((500*math.pi/180)/32768)
DATA_PERIOD = 0.02
ID_CPR2ROS_DATA = 0x10  # 您需要根据实际协议定义这个值

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd        
        self.previous_error = 0
        self.integral = 0
# 输入当前位置，也可通过输入误差
    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.error_previous

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.error_previous = error
        return output

class Motor:
    def __init__(self,ser):
        self.ser = ser
        self.RADTOANGLE = 180*10 / 3.14159  # Example conversion factor
        self.SCALE_FACTOR = 1350 / 180   # 假设最大范围对应180度
        self.rx_buf = bytearray(24)
        self.rx_con = 0
        self.rx_checksum = 0
        self.imu_data_ = {
            'acc_x': 0,
            'acc_y': 0,
            'acc_z': 0,
            'gyro_x': 0,
            'gyro_y': 0,
            'gyro_z': 0
        }
        self.vel_data_ = {
            'linear_x': 0,
            'linear_y': 0,
            'angular_z': 0
        }
        self.bat_vol_data_ = 0
        self.pos_data_ = {
            'pos_x': 0,
            'pos_y': 0,
            'angular_z': 0
        }
        
        
    def recv_callback(self):
        while True:
            res = self.ser.read(1)[0]  # 读取一个字节
            if self.rx_con < 3:
                if self.rx_con == 0:
                    if res == 0xAA:
                        self.rx_buf[0] = res
                        self.rx_con = 1
                elif self.rx_con == 1:
                    if res == 0x55:
                        self.rx_buf[1] = res
                        self.rx_con = 2
                    else:
                        self.rx_con = 0
                else:
                    self.rx_buf[2] = res
                    self.rx_con = 3
                    self.rx_checksum = (0xAA + 0x55) + res
            else:
                if self.rx_con < (self.rx_buf[2] - 1):
                    self.rx_buf[self.rx_con] = res
                    self.rx_con += 1
                    self.rx_checksum += res
                else:
                    self.rx_con = 0
                    if res == self.rx_checksum:
                        self.recv_data_handle(self.rx_buf)

# 接收数据
    def recv_data_handle(self, buffer_data):
        if buffer_data[3] == ID_CPR2ROS_DATA:
            self.imu_data_['acc_x'] = ((int.from_bytes(buffer_data[4:6], 'big', signed=True)) * ACC_RATIO)
            self.imu_data_['acc_y'] = ((int.from_bytes(buffer_data[6:8], 'big', signed=True)) * ACC_RATIO)
            self.imu_data_['acc_z'] = ((int.from_bytes(buffer_data[8:10], 'big', signed=True)) * ACC_RATIO)

            self.imu_data_['gyro_x'] = ((int.from_bytes(buffer_data[10:12], 'big', signed=True)) * GYRO_RATIO)
            self.imu_data_['gyro_y'] = ((int.from_bytes(buffer_data[12:14], 'big', signed=True)) * GYRO_RATIO)
            self.imu_data_['gyro_z'] = ((int.from_bytes(buffer_data[14:16], 'big', signed=True)) * GYRO_RATIO)

            self.vel_data_['linear_x'] = ((int.from_bytes(buffer_data[16:18], 'big', signed=True)) / 1000)
            self.vel_data_['linear_y'] = ((int.from_bytes(buffer_data[18:20], 'big', signed=True)) / 1000)
            self.vel_data_['angular_z'] = ((int.from_bytes(buffer_data[20:22], 'big', signed=True)) / 1000)

            self.bat_vol_data_ = (int.from_bytes(buffer_data[22:24], 'big')) / 100

            self.pos_data_['pos_x'] += (self.vel_data_['linear_x'] * math.cos(self.pos_data_['angular_z']) - 
                                        self.vel_data_['linear_y'] * math.sin(self.pos_data_['angular_z'])) * DATA_PERIOD
            self.pos_data_['pos_y'] += (self.vel_data_['linear_x'] * math.sin(self.pos_data_['angular_z']) + 
                                        self.vel_data_['linear_y'] * math.cos(self.pos_data_['angular_z'])) * DATA_PERIOD
            self.pos_data_['angular_z'] += self.vel_data_['angular_z'] * DATA_PERIOD
            
            
    def send_serial_packet(self, pbuf, num):
        tx_checksum = 0
        tx_buf = bytearray(64)

        len_pbuf = len(pbuf)

        if len_pbuf <= 64:
            tx_buf[0] = 0xAA  # Frame header
            tx_buf[1] = 0x55
            tx_buf[2] = len_pbuf + 5  # Frame length
            tx_buf[3] = num  # Frame number

            for i in range(len_pbuf):
                tx_buf[4 + i] = pbuf[i]

            cnt = 4 + len_pbuf
            for i in range(cnt):
                tx_checksum += tx_buf[i]
            tx_buf[cnt] = tx_checksum & 0xFF  # Ensure checksum is a byte

            # Calculate frame length
            cnt = len_pbuf + 5

            # Send data
            self.ser.write(tx_buf[:cnt])
            

    def cmd_vel_callback(self, msg):
        vel_data = bytearray(6)

        # Data conversion
        linear_x = int(msg['linear_x'] * 1000)
        linear_y = int(msg['linear_y'] * 1000)
        angular_z = int(msg['angular_z'] * 1000)

        vel_data[0:2] = struct.pack('>h', linear_x)  # Big-endian short
        vel_data[2:4] = struct.pack('>h', linear_y)  # Big-endian short
        vel_data[4:6] = struct.pack('>h', angular_z)  # Big-endian short

        # Send serial data
        self.send_serial_packet(vel_data,0x50)

    def joint_cmd_callback(self, msg):
            joint_data = bytearray(12)
            jointstate_list_ = [0.0] * 6  # Initialize joint state list
            # Data conversion
            for i in range(6):
                angle_value = int(msg['joint_pos'][i] * self.RADTOANGLE*self.SCALE_FACTOR)
                joint_data[2 * i:2 * i + 2] = struct.pack('>h', angle_value)  # Big-endian short
                jointstate_list_[i] = msg['joint_pos'][i] *self.RADTOANGLE

            # Send serial data
            self.send_serial_packet(joint_data, 0x60)
    def stop_motor(self):
        msg = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.cmd_vel_callback(msg=msg)
        print("电机停止")
    
if __name__ == "__main__":
    ser = serial.Serial("COM8",230400)
    motor = Motor(ser=ser)
    msg2 = {'linear_x': 1.0, 'linear_y': 0.0, 'angular_z': 0.5}  # Example data
    motor.cmd_vel_callback(msg2)
    motor.recv_callback()
    print(motor.pos_data_['angular_z'])
    msg = {'joint_pos': [0, 1.0, 1.0, 0.0, 0.0, 0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    time.sleep(5)
    msg1 = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}  # Example data
    motor.cmd_vel_callback(msg1)    
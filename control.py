import time
from constants import pwm_max,pwm_min
import struct
import serial
import math
import cv2
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
        derivative = error - self.previous_error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error
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
        
        
class StepMotor:
    # 对照表
    '''
        S_VER = 0      # 读取固件版本和对应的硬件版本
        S_RL = 1       # 读取读取相电阻和相电感
        S_PID = 2      # 读取PID参数
        S_VBUS = 3     # 读取总线电压
        S_CPHA = 5     # 读取相电流
        S_ENCL = 7     # 读取经过线性化校准后的编码器值
        S_TPOS = 8     # 读取电机目标位置角度
        S_VEL = 9      # 读取电机实时转速
        S_CPOS = 10    # 读取电机实时位置角度
        S_PERR = 11    # 读取电机位置误差角度
        S_FLAG = 13    # 读取使能/到位/堵转状态标志位
        S_Conf = 14    # 读取驱动参数
        S_State = 15   # 读取系统状态参数
        S_ORG = 16     # 读取正在回零/回零失败状态标志位
    '''
    def __init__(self,uart):
        self.uart = uart
        
    def Emm_V5_Read_Sys_Params(self,addr, s): # 读取驱动板参数
        i = 0
        cmd = bytearray(16)
        cmd[i] = addr
        i += 1 
        func_codes = {
            'S_VER': 0x1F,
            'S_RL': 0x20,
            'S_PID': 0x21,
            'S_VBUS': 0x24,
            'S_CPHA': 0x27,
            'S_ENCL': 0x31,
            'S_TPOS': 0x33,
            'S_VEL': 0x35,
            'S_CPOS': 0x36,
            'S_PERR': 0x37,
            'S_FLAG': 0x3A,
            'S_ORG': 0x3B,
            'S_Conf': 0x42,     # 读取驱动参数，功能码后面还需要加上一个辅助码0x6C
            'S_State': 0x43     # 读取系统状态参数，功能码后面还需要加上一个辅助码0x7A
        }
        if s in func_codes:
            cmd[i] = func_codes[s]
            i += 1
        cmd[i] = 0x6B
        i += 1
        self.uart.write(cmd[:i])
        
        
    def Emm_V5_Reset_CurPos_To_Zero(self, addr): # 将当前位置清零
        cmd = bytearray(4)
        cmd[0] =  addr                          # 地址
        cmd[1] =  0x0A                          # 功能码
        cmd[2] =  0x6D                          # 辅助码
        cmd[3] =  0x6B                          # 校验字节
        self.uart.write(cmd)
        
        
    def Emm_V5_Modify_Ctrl_Mode(self, addr, svF, ctrl_mode): # 调用函数修改控制模式
        cmd = bytearray(6)
        cmd[0] = addr          # 地址
        cmd[1] = 0x46          # 功能码
        cmd[2] = 0x69          # 辅助码
        cmd[3] = 0x01 if svF else 0x00  # 是否存储标志, 1为存储, 0为不存储
        cmd[4] = ctrl_mode     # 控制模式
        cmd[5] = 0x6B          # 校验字节
        self.uart.write(cmd)      
        
    def Emm_V5_En_Control(self, addr, state, snF): # 为地址电机使能，并启用多机同步
        cmd = bytearray(16)
        cmd[0] = addr               # 地址
        cmd[1] = 0xF3               # 功能码
        cmd[2] = 0xAB               # 辅助码
        cmd[3] = 0x01 if state else 0x00  # 使能状态，true为0x01，false为0x00
        cmd[4] = 0x01 if snF else 0x00    # 多机同步运动标志，true为0x01，false为0x00
        cmd[5] = 0x6B               # 校验字节
        self.uart.write(cmd[:6])
        
    def Emm_V5_Vel_Control(self, addr, dir, vel, acc, snF): # 地址电机，设置方向为CW，速度为1000RPM，加速度为50，无多机同步
        cmd = bytearray(16)
        cmd[0] = addr                  # 地址
        cmd[1] = 0xF6                  # 功能码
        cmd[2] = dir                   # 方向，0为CW，其余值为CCW
        cmd[3] = (vel >> 8) & 0xFF     # 速度(RPM)高8位字节
        cmd[4] = vel & 0xFF            # 速度(RPM)低8位字节
        cmd[5] = acc                   # 加速度，注意：0是直接启动
        cmd[6] = 0x01 if snF else 0x00 # 多机同步运动标志，true为0x01，false为0x00
        cmd[7] = 0x6B                  # 校验字节
        print(cmd[:8])
        self.uart.write(cmd[:8])
        
        
        
    def Emm_V5_Pos_Control(self, addr, dir, vel, acc, clk, raF, snF): # 地址电机，设置方向为CW，速度为1000RPM，加速度为50，脉冲数为2000，相对运动，无多机同步
        cmd = bytearray(16)
        cmd[0] = addr                      # 地址
        cmd[1] = 0xFD                      # 功能码
        cmd[2] = dir                       # 方向, 01 表示旋转方向为 CCW（00 表示 CW）
        cmd[3] = (vel >> 8) & 0xFF         # 速度(RPM)高8位字节
        cmd[4] = vel & 0xFF                # 速度(RPM)低8位字节 
        cmd[5] = acc                       # 加速度，注意：0是直接启动
        cmd[6] = (clk >> 24) & 0xFF        # 脉冲数高8位字节(bit24 - bit31)
        cmd[7] = (clk >> 16) & 0xFF        # 脉冲数(bit16 - bit23)
        cmd[8] = (clk >> 8) & 0xFF         # 脉冲数(bit8  - bit15)
        cmd[9] = clk & 0xFF                # 脉冲数低8位字节(bit0  - bit7)
        cmd[10] = 0x01 if raF else 0x00    # 相位/绝对标志，true为0x01，false为0x00
        cmd[11] = 0x01 if snF else 0x00    # 多机同步运动标志，true为0x01，false为0x00
        cmd[12] = 0x6B                     # 校验字节
        self.uart.write(cmd[:13])
        
    def Emm_V5_Stop_Now(self, addr, snF): # 地址电机，不启用多机同步，立即停止
        cmd = bytearray(5)
        cmd[0] = addr               # 地址
        cmd[1] = 0xFE               # 功能码
        cmd[2] = 0x98               # 辅助码
        cmd[3] = 0x01 if snF else 0x00  # 多机同步运动标志，true为0x01，false为0x00
        cmd[4] = 0x6B               # 校验字节
        self.uart.write(cmd)
    
    def Emm_V5_Receive_Data(self):
        i = 0
        rxCmd = bytearray(128)
        lTime = cTime = time.time()  # 使用time.time()来获取当前时间（以秒为单位）
        while True:
            if self.uart.any():
                if i < 128:
                    rxCmd[i] = self.uart.read(1)[0]
                    i += 1
                    lTime = time.time()  # 更新最后接收数据的时间
            else:
                cTime = time.time()  # 获取当前时间
                # 计算时间差（转换为毫秒）
                if (cTime - lTime) * 1000 > 100: 
                    # 将数据转换为十六进制字符串
                    hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                    hex_data = hex_data.strip('00 ')  # 去掉16进制字符串前后的无效0
                    if hex_data and hex_data[0] != '0':  # 如果首字符不是0，则在首字符前添加一个0
                        hex_data = '0' + hex_data
                    return hex_data, len(hex_data.replace(' ', '')) // 2  # 返回数据和数据长度
                
def pid_move(cap,motor,front_vel,element):
    pid_x = PIDController(kp=0.0002, ki=0.0, kd=0.0)
    pid_y = PIDController(kp=0.0002, ki=0.0, kd=0.0)
    while True:
            center = cap.detect_circle(element)
            cap.cv_imshow()
            if cv2.waitKey(100)  == ord('q'):
                motor.stop_motor()
                break
            if center:
                if abs(center[0]-352) <= 10 and abs(center[1]-159) <= 10 :
                    print(f"x_delt{center[0]-352},y_delt{center[1]-159}")
                    motor.stop_motor()
                    break
                print(f"x_delt{center[0]-352},y_delt{center[1]-159}")
                out_x = pid_x.compute(center[0], 352)
                out_y = pid_y.compute(center[1], 159)
                msg1 = {'linear_x': -out_x, 'linear_y': out_y, 'angular_z': 0.0}  # Example data
                motor.cmd_vel_callback(msg1)
            else:
                msg1 = {'linear_x': front_vel, 'linear_y': 0.0, 'angular_z': 0.0}  # Example data
                motor.cmd_vel_callback(msg1)
    time.sleep(2.0)
    return 1
                
if __name__ == "__main__":
    import cv2
    from identify import VideoCapture
    
    # msg2 = {'linear_x': 1.0, 'linear_y': 0.0, 'angular_z': 0.5}  # Example data
    # motor.cmd_vel_callback(msg2)
    # motor.recv_callback()
    # print(motor.pos_data_['angular_z'])
    # msg = {'joint_pos': [0, 1.0, 1.0, 0.0, 0.0, 0.0]}  # Example joint positions
    # motor.joint_cmd_callback(msg)
    # time.sleep(5)
    # msg1 = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}  # Example data
    # motor.cmd_vel_callback(msg1)
    ser1 = serial.Serial("/dev/ttyACM0",230400)
    ser = serial.Serial("/dev/ttyUSB0",115200)
    motor = Motor(ser=ser1)
    stepmotor = StepMotor(ser)   # 步进电机初始
    msg = {'joint_pos': [0, 0.05, 0.00, 0.0, 0.0, -1.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)   # 步进电机使能 1down
    time.sleep(2.0)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=3000, raF=1, snF=0)
    time.sleep(2.0)
    msg = {'joint_pos': [0, 0.05, 0.00, 0.0, 0.0, -0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=100, clk=3000, raF=1, snF=0)
    time.sleep(1.8)
    msg = {'joint_pos': [0, -1.0, 0.00, 0.0, 0.0, 0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    time.sleep(0.5)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=100, clk=1000, raF=1, snF=0)
    time.sleep(1.8)
    msg = {'joint_pos': [0, -1.0, 0.00, 0.0, 0.0, -1.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=100, clk=1000, raF=1, snF=0)
    time.sleep(1.8)
    msg = {'joint_pos': [0, 0.05, 0.00, 0.0, 0.0, 0.0]}
    motor.joint_cmd_callback(msg)
    #stepmotor.Emm_V5_Stop_Now(addr=0x01,snF=0)
    exit()
    motor = Motor(ser=ser)
    msg = {'joint_pos': [0, 0.05, 0.02, 0.0, 0.0, 0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    exit()
    cap=VideoCapture(1)
    #exit()
    list=[2,3,1]
    lasted_index = 0
    for i in range(len(list)):
        front_vel = 0.1
        if i == 0:
            lasted_list = list[i]
            
        else:
            if list[i]-lasted_list<0:
                front_vel = -0.1
            else:
                front_vel = 0.1
        pid_move(cap,motor,front_vel,list[i])
    cap.release()
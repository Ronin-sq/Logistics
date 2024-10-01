import time
from constants import pwm_max,pwm_min
from uart import Uart
import protocol
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
    
class Motor:           # 编码电机类对象
    def __init__(self,port,baudrate):
        self.uart = Uart(port,baudrate)
        
                
    def abs(self,data):
        if data<0:
            data = -data
        return data
    def control_stepping(self):
        print()
    
    def control_rudder(self,index,angle):
        send_data = protocol.rudder_move(index,angle)
        self.uart.send_data(send_data)
        print(f"控制舵机成功：{send_data}")
        
        
    def control_motor(self,index,motor_list,rpm_list):
        send_data = protocol.motor_move(index,motor_list,rpm_list)
        self.uart.send_data(send_data)
        print(f"发送成功:{send_data}")
        
        
    def flag_receive(self):
        flag = self.uart.receive_data()
        return flag
    
    
    def motor_stop(self):
        send_data = protocol.motor_close()
        self.uart.send_data(send_data)
        print("电机停转")
        
        
        
    def min_max(self, pwm):
        if self.abs(pwm) >=pwm_max:
            pwm = pwm_max
        if self.abs(pwm) <=pwm_min:
            pwm = 0
        return pwm
    
    def get_current_position(self):
        position = self.uart.receive_data()
        return position
    
    
if __name__ == "__main__":
    pid = PIDController(1,0,0)
    
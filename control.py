import time
from constants import pwm_max,pwm_min
from uart import Uart
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
    def __init__(self, index):
        self.index = index
        self.uart = Uart(port="dev",baudrate=9200)
        
    def abs(self,data):
        if data<0:
            data = -data
        return data
    
    def control_motor(self, pwm ):
        pwm = self.min_max(pwm)
        self.uart.send_data(pwm)
        print("发送成功")
        
        
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
    
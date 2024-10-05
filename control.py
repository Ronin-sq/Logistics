import time
from constants import pwm_max,pwm_min
from uart import Uart
import protocol
import protocol1
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
        self.motor = protocol1.Board()
        
                
    def abs(self,data):
        if data<0:
            data = -data
        return data
    def control_stepping(self):
        print()
    
    def control_rudder(self,index,angle):
        # send_data = protocol.rudder_move(index,angle)
        position = 500+(angle/360)*2000
        self.motor.pwm_servo_set_position(0.1,[[index,position]])
        print(f"控制舵机成功：{position}")
        
        
    def control_motor(self,motor_list,rpm_list):
        # send_data = protocol.motor_move(index,motor_list,rpm_list)
        
        self.motor.set_motor_speed([motor_list[0],rpm_list[0]],[motor_list[1],rpm_list[1],[motor_list[2],rpm_list[2]],[motor_list[3],rpm_list[3]]])
        print(f"发送成功")
        
        
    def flag_receive(self):
        flag = self.uart.receive_data()
        return flag
    
    
    def motor_stop(self):

        self.motor.set_motor_speed([[1,0],[2,0],[3,0],[4,0]])
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
    
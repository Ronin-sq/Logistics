# coding Ronin
# encoding = UTF-8
import time
import threading
import uart
import constants
from identify import VideoCapture,show_mission
from control import Motor
# 1代表夹爪，2代表转盘舵机序列号，3代表机械手底座舵机序号
rudder_gripper = 1
rudder_turntable = 2
rudder_arm = 3
# 初始化电机对象，电机对象内包含了uart初始化
motor = Motor(constants.port_1,constants.baudrate_1)
# 初始化两个相机对象
cap1 = VideoCapture(0)   # 识别二维码
cap2 = VideoCapture(1)   # 识别圆
while True:
    flag = motor.flag_receive()
    if flag:
        break
    # 接收到启动消息开始运动，接收侧还需要再设计
motor.control_motor(index=4, motor_list=[1,2,3,4],rpm_list=[1,1,1,1])
time.sleep(1.0)
motor.motor_stop()   # 移动出发车区
motor.control_motor(index=4, motor_list=[1,2,3,4],rpm_list=[1,1,1,1])
time.sleep(5.0)      # 移动到二维码区
motor.motor_stop()
list = cap1.QR_code()           # QR_code返回的格式是否为列表有待确认
identify_list = list           # 复制序列信息，防止线程交叉
thread = threading.Thread(target=show_mission,args=(list))      # 启动一个线程 展示任务信息
thread.start()
motor.control_motor(index=4, motor_list=[1,2,3,4],rpm_list=[1,1,1,1])
time.sleep(5.0)
motor.motor_stop()
for i,element in enumerate(identify_list):
    while True:
        try:
            xyr = cap2.find_material(element)
            break
        except:
            continue
    # 先降步进电机
    motor.control_stepping()       # 还未定义
    # 夹爪夹住物料
    motor.control_rudder(index=rudder_gripper,angle=90)
    # 升步进电机
    motor.control_stepping()
    # 转底座
    motor.control_rudder(index=rudder_arm,angle=90)
    
    
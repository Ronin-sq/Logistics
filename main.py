# coding Ronin
# encoding = UTF-8
import time
import serial
import threading
import uart
import constants
from identify import VideoCapture, show_mission, str_int
from control import Motor

# 1代表夹爪，2代表转盘舵机序列号，3代表机械手底座舵机序号

rudder_gripper = 1
rudder_turntable = 2
rudder_arm = 3
# 初始化电机对象，电机对象内包含了uart初始化
ser = serial.Serial(constants.port_1, constants.baudrate_1)  # 串口初始化
motor = Motor(ser=ser)  # 电机初始化

# 初始化两个相机对象
cap1 = VideoCapture(0)  # 识别二维码
# cap2 = VideoCapture(1)   # 识别圆
while True:
    if ser.in_waiting >= 7:
        data = ser.read(7)
        if len(data) == 7:  # 再次确认读取的数据长度
            byte = data[5]
            if byte == 0x01:
                print("获取到标准符，程序启动")
                break
        else:
            print("读取的数据不足7字节,尝试重新读取")
    # 接收到启动消息开始运动，接收侧还需要再设计
vel_msg = {"linear_x": 0.0, "linear_y": 0.5, "angular_z": 0.0}
motor.cmd_vel_callback(vel_msg)
time.sleep(1.0)
motor.stop_motor()  # 移动出发车区
vel_msg = {"linear_x": 1.0, "linear_y": 0.0, "angular_z": 0.0}
motor.cmd_vel_callback(vel_msg)
time.sleep(5.0)  # 移动到二维码区
motor.stop_motor()
while True:
    list = cap1.QR_code()  # QR_code返回的格式是否为列表有待确认
    list = str_int(list=list)
    cap1.cv_imshow()
    if len(list) > 0:
        print(type(list))
        break
identify_list = list  # 复制序列信息，防止线程交叉
thread = threading.Thread(target=show_mission, args=(list))  # 启动一个线程 展示任务信息
thread.start()
vel_msg = {"linear_x": 1.0, "linear_y": 0.0, "angular_z": 0.0}
motor.cmd_vel_callback(vel_msg)
time.sleep(5.0)
motor.stop_motor()
for i, element in enumerate(identify_list):
    while True:
        try:
            xyr = cap2.find_material(element)
            break
        except:
            continue
    # 先降步进电机
    motor.control_stepping()  # 还未定义
    # 夹爪夹住物料
    motor.control_rudder(index=rudder_gripper, angle=90)
    # 升步进电机
    motor.control_stepping()
    # 转底座
    motor.control_rudder(index=rudder_arm, angle=90)

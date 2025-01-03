# coding Ronin
# encoding = UTF-8
import time
import math
import serial
import threading
import uart
import constants
from constants import joint_msg
from identify import VideoCapture, show_mission, str_int
from control import Motor,StepMotor,pid_move
import cv2
def move_x(distance,vel):
    times = distance/(vel*1000)
    vel_msg = {"linear_x": vel, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(times)
    motor.stop_motor()
    
def move_y(distance,vel):
    times = distance/(vel*1000)
    vel_msg = {"linear_x": 0.0, "linear_y": vel, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(times)
    motor.stop_motor()
def move_z(angel,vel):
    angel_radians = math.radians(angel)
    times = angel_radians/vel
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": vel}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(times)
    motor.stop_motor()
def grip(motor,stepmotor,element):
    joint_msg['joint_pos'][5] = -1  # Example joint positions
    motor.joint_cmd_callback(joint_msg)
    print("open gripper")
    #exit()
    #stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)   # 步进电机使能 1down
    #time.sleep(2.0)
    print("down stepmotor")
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=5000, raF=1, snF=0)
    time.sleep(1.75)
    joint_msg['joint_pos'][5] = 1.0 # Example joint positions
    print("closed gripper")
    motor.joint_cmd_callback(joint_msg)
    time.sleep(1.0)
    print("up step")
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=3000, raF=1, snF=0)
    time.sleep(1.76)
    joint_msg['joint_pos'][1] = -1  # Example joint positions hypothesis is -1
    motor.joint_cmd_callback(joint_msg)
    if element==2:
        joint_msg['joint_pos'][2] = 0.2  # Example joint positions hypothesis is -1
        motor.joint_cmd_callback(joint_msg)
    if element==3:
        joint_msg['joint_pos'][2] = -0.17  # Example joint positions hypothesis is -1
        motor.joint_cmd_callback(joint_msg)
    if element==1:
        joint_msg['joint_pos'][2] = 0.02  # Example joint positions hypothesis is -1
        motor.joint_cmd_callback(joint_msg)
    time.sleep(1.0)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=100, raF=1, snF=0)
    time.sleep(1.5)#down
    print("down2")
    joint_msg['joint_pos'][5] = -1  # Example joint positions hypothesis is -1
    motor.joint_cmd_callback(joint_msg)
    stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=1000, raF=1, snF=0)
    time.sleep(1.2)#up
    joint_msg['joint_pos'][1] = 0.05  # Example joint positions hypothesis is -1
    motor.joint_cmd_callback(joint_msg)
# 1代表夹爪，2代表转盘舵机序列号，3代表机械手底座舵机序号
def main():
    global last_list
    rudder_gripper = 1
    rudder_turntable = 2
    rudder_arm = 3
    # 初始化电机对象，电机对象内包含了uart初始化
    ser = serial.Serial(constants.port_1, constants.baudrate_1)  # 串口初始化
    ser1 = serial.Serial(constants.port_2,constants.baudrate_2)
    motor = Motor(ser=ser)  # 电机初始化
    stepmotor = StepMotor(ser1)   # 步进电机初始化
    # 初始化两个相机对象
    cap1 = VideoCapture(0)  # 识别二维码
    cap2 = VideoCapture(1)   # 识别圆
    stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)   # 步进电机使能
    time.sleep(0.1)
    stepmotor.Emm_V5_Reset_CurPos_To_Zero(addr=0x01)
    msg = {'joint_pos': [0, 0.00, 0.0, 0.0, 0.0, 0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    while True:
        if ser.in_waiting >= 7:
            data = ser.read(7)
            if len(data) == 7:  # 再次确认读取的数据长度
                print(data)
                byte = data[5]
                if byte == 0x01:
                    print("获取到标准符，程序启动")
                    break
            else:
                print("读取的数据不足7字节,尝试重新读取")
        # 接收到启动消息开始运动，接收侧还需要再设计
    vel_msg = {"linear_x": 0.0, "linear_y": 0.1, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(1.0)
    motor.stop_motor()  # 移动出发车区
    vel_msg = {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(4.0)  # 移动到二维码区
    motor.stop_motor()
    time.sleep(3.0)
    #while True:
    #    list = cap1.QR_code()  # QR_code返回的格式是否为列表有待确认
    #   #list = str_int(list=list)
    #   cap1.cv_imshow()
    #   if len(list) > 0:
    #       print(type(list))
    #       break
    msg = {'joint_pos': [0, 0.05, 0.0, 0.0, 0.0, 0.0]}  # Example joint positions
    motor.joint_cmd_callback(msg)
    list = "231+132"
    list = str_int(list)
    identify_list = list  # 复制序列信息，防止线程交
    mission1 = identify_list[:3]
    mission2 = identify_list[3:6]
    print("code over")
    thread = threading.Thread(target=show_mission, args=(list, motor))  # 启动一个线程 展示任务信息
    thread.start()
    vel_msg = {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(3.35)
    motor.stop_motor()
    print("daodazhuaququ")
    count = 0 # 计数符
    for i, element in enumerate(identify_list):
        #print(f"element{element}")
        count += 1
        print(constants.color_config[element])
        frame = cap2.get_frame()
       
        while True:
            cv2.imshow("frame",frame)
            if cv2.waitKey(1)==ord("q"):
                break
            try:
                frame = cap2.get_frame()
                xyr = cap2.find_material(element,frame)
                #print(f"xyr{xyr}")
                if xyr:
                    print("识别成功，进入抓取放置物料环节")
                    #cv2.imwrite('666.jpg',frame)
                    break
            except:
                frame = cap2.get_frame()
                continue
        # grip
        grip(motor,stepmotor,element)
        print("grip successful")
        time.sleep(1.0)
        if count == 3:
            print("第一批识别完毕，退出识别")
            break
    vel_msg = {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.65)
    motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.12)
    motor.stop_motor()
    time.sleep(1.0)
    #qianwangjiagongqu
    vel_msg = {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(6.1)
    motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.12)
    motor.stop_motor()
    #exit()
    lasted_list = 0
    for i in range(len(mission1)):
        front_vel = 0.1
        if i == 0:
            lasted_list = mission1[i]
            
        else:
            if mission1[i]-lasted_list<0:
                front_vel = -0.1
            else:
                front_vel = 0.1
        flag = pid_move(cap2,motor,front_vel,mission1[i])
        print("zhaodaobaxin")
        if flag == 1:
            element = mission1[i]
            joint_msg['joint_pos'][1] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            if element==2:
                joint_msg['joint_pos'][2] = 0.2  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if element==3:
                joint_msg['joint_pos'][2] = -0.17  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if element==1:
                joint_msg['joint_pos'][2] = 0.02  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            time.sleep(1.0)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=100, raF=1, snF=0)
            time.sleep(1.5)#down
            print("down2")
            joint_msg['joint_pos'][5] = 1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=500, raF=1, snF=0)
            time.sleep(1.2)#up
            joint_msg['joint_pos'][1] = 0.05  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=6000, raF=1, snF=0)
            time.sleep(1.8)#down
            joint_msg['joint_pos'][5] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=5000, raF=1, snF=0)
            time.sleep(1.5)#down
            flag=0
            
    for i in range(len(mission1)):
        front_vel = 0.1
        
        if mission1[i]-lasted_list<0:
            front_vel = -0.1
        else:
            front_vel = 0.1
        flag = pid_move(cap2,motor,front_vel,mission1[i])
        print("zhaodaobaxin")
        if flag==1:
            joint_msg['joint_pos'][5] = -1  # Example joint positions
            motor.joint_cmd_callback(joint_msg)
            print("open gripper")
            #exit()
            #stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)   # 步进电机使能 1down
            #time.sleep(2.0)
            print("down stepmotor")
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=6000, raF=1, snF=0)
            time.sleep(1.75)
            joint_msg['joint_pos'][5] = 1.0 # Example joint positions
            print("closed gripper")
            motor.joint_cmd_callback(joint_msg)
            time.sleep(1.0)
            print("up step")
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=5000, raF=1, snF=0)
            time.sleep(1.76)
            joint_msg['joint_pos'][1] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            if mission1[i]==2:
                joint_msg['joint_pos'][2] = 0.2  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if mission1[i]==3:
                joint_msg['joint_pos'][2] = -0.17  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if mission1[i]==1:
                joint_msg['joint_pos'][2] = 0.02  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            time.sleep(1.0)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=100, raF=1, snF=0)
            time.sleep(1.5)#down
            print("down2")
            joint_msg['joint_pos'][5] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=1000, raF=1, snF=0)
            time.sleep(1.2)#up
            joint_msg['joint_pos'][1] = 0.05  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            time.sleep(1.0)
            flag = 0
    print("diercizhuaquwanbi")
    if last_list == 1:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(2.8)
        motor.stop_motor()
    if last_list == 2:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(3.0)
        motor.stop_motor()
    if last_list == 3:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(3.2)
        motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.15)
    motor.stop_motor()
    last_list = 0
    for i in range(len(mission1)):
        front_vel = -0.1
        if i == 0:
            lasted_list = mission1[i]
            
        else:
            if mission1[i]-lasted_list<0:
                front_vel = -0.1
            else:
                front_vel = 0.1
        flag = pid_move(cap2,motor,front_vel,mission1[i])
        print("zhaodaobaxin")
        if flag == 1:
            element = mission1[i]
            joint_msg['joint_pos'][1] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            if element==2:
                joint_msg['joint_pos'][2] = 0.2  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if element==3:
                joint_msg['joint_pos'][2] = -0.17  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            if element==1:
                joint_msg['joint_pos'][2] = 0.02  # Example joint positions hypothesis is -1
                motor.joint_cmd_callback(joint_msg)
            time.sleep(1.0)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=100, raF=1, snF=0)
            time.sleep(1.5)#down
            print("down2")
            joint_msg['joint_pos'][5] = 1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=1000, raF=1, snF=0)
            time.sleep(1.2)#up
            joint_msg['joint_pos'][1] = 0.05  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=1, vel=200, acc=200, clk=6000, raF=1, snF=0)
            time.sleep(1.8)#down
            joint_msg['joint_pos'][5] = -1  # Example joint positions hypothesis is -1
            motor.joint_cmd_callback(joint_msg)
            stepmotor.Emm_V5_Pos_Control(addr=0x01, dir=0, vel=200, acc=200, clk=5000, raF=1, snF=0)
            time.sleep(1.5)#down
            flag=0
        print("diercifangquchenggong")
            
    
if __name__ == "__main__":
    main()
    exit()
    import cv2
    ser = serial.Serial(constants.port_1, constants.baudrate_1)  # 串口初始化
    ser1 = serial.Serial(constants.port_2,constants.baudrate_2)
    motor = Motor(ser=ser)  # 电机初始化
    stepmotor = StepMotor(ser1)   # 步进电机初始
    msg = {'joint_pos': [0, 0.05, 0.0, 0.0, 0.0, 0.0]}
    motor.joint_cmd_callback(msg)
    motor.stop_motor()
    #exit()
    last_list =1
    if last_list == 1:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(3.4)
        motor.stop_motor()
    if last_list == 2:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(3.6)
        motor.stop_motor()
    if last_list == 3:
        vel_msg = {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0}
        motor.cmd_vel_callback(vel_msg)
        time.sleep(3.8)
        motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.15)
    motor.stop_motor()
    exit()
    vel_msg = {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.5)
    motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.15)
    motor.stop_motor()
    time.sleep(1.0)
    #qianwangjiagongqu
    vel_msg = {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(5.8)
    motor.stop_motor()
    vel_msg = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.8}
    motor.cmd_vel_callback(vel_msg)
    time.sleep(2.15)
    motor.stop_motor()
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
    exit()# Example joint positions
    motor.joint_cmd_callback(msg)
    stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)
    time.sleep(1.0)
    grip(motor,stepmotor,1)
    #grip(motor,stepmotor)
    exit()
    #msg = {'joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, -1]}  # Example joint positions
    stepmotor.Emm_V5_En_Control(addr=0x01, state=1,snF=0)   # 步进电机使能 1down
    time.sleep(2.0)
    stepmotor.Emm_V5_Vel_Control(addr=0x01, dir=0, vel=500, acc=100, snF=0)
    time.sleep(1.8)
    stepmotor.Emm_V5_Stop_Now(addr=0x01,snF=0)
    exit()
    #motor.joint_cmd_callback(msg)
    
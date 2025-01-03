import numpy as np
pwm_max = 7200
pwm_min = 300
flag = False
color_config = {
    1:[[[np.array([0, 43, 46]),np.array([12, 255, 255])],
       [np.array([155, 43, 46]),np.array([180, 255, 255])]]],  # red
    2:[np.array([30,30,30]),np.array([80,255,255])],    # green
    3:[np.array([100,43,46]),np.array([124,255,255])]  # blue
}
port_1 = "/dev/ttyACM0"
baudrate_1=230400
port_2 = "/dev/ttyUSB0"
baudrate_2=115200
# print(color_config[1])
joint_msg = {'joint_pos': [0, 0.05, 0.0, 0.0, 0.0, 0.0]}
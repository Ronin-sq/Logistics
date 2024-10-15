import numpy as np
pwm_max = 7200
pwm_min = 300
flag = False
color_config = {
    1:[np.array([0, 43, 46]),np.array([12, 255, 255])],  # yellow
    2:[np.array([35,43,46]),np.array([77,255,255])],    # green
    3:[np.array([100,43,46]),np.array([124,255,255])]  # blue
}
port_1 = "/dev/ttyACM0"
baudrate_1=230400
port_2 = "/dev/ttyS0"
baudrate_2=115200
# print(color_config[1])
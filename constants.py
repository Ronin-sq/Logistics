import numpy as np
pwm_max = 7200
pwm_min = 300
flag = False
color_config = {
    1:[np.array([26, 128, 46]),np.array([34, 255, 255])],  # yellow
    2:[np.array([35,43,46]),np.array([77,255,255])],    # green
    3:[np.array([100,43,46]),np.array([124,255,255])]  # blue
}
port_1 = "dev/"
baudrate_1=9600 
print(color_config[1])
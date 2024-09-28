# coding Ronin
# encoding = UTF-8
import uart
import constants
# 初始化串口串口对象
uart_1 = uart.Uart(constants.port_1, constants.baudrate_1)
# 初始化电机

flag = uart_1.receive_data()   # 获取启动信号
if flag == True:
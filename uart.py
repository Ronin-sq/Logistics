import serial
class Uart:
    def __init__(self,port,baudrate):
        try:
            self.uart = serial.Serial(port, baudrate)
        except serial.PortNotOpenError as e:
            print(f"{e}")
            
    def receive_data(self):
        while True:
            if self.uart.in_waiting:
                data = self.uart.readline().encode("UTF-8")
                print(f"receive data {data} successful!")
                break
        return data
    
    def send_data(self, data):
        if data:
            send_data = bytes(data, encoding = 'UTF-8')
            self.uart.write(send_data)
            print(f"send data:{send_data} successful!")
        else:
            print("receive data failed")
            
    def close(self):
        self.uart.close()
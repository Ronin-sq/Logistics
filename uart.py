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
            # send_data = bytes(data, encoding = 'UTF-8')
            self.uart.write(data)
            print(f"send data:{data} successful!")
        else:
            print("receive data failed")
            
    def close(self):
        self.uart.close()
        
if __name__ == "__main__":
    import struct
    byte = struct.pack("BBBBBBB",0xAA,0x55,0x07,0x00,0x01,0x00,0x07)
    # print(byte[4])
    if byte[4] == 0x01:
        print(f"flag = {byte[4]}")
    
import time
import socket
import numpy as np


class PcReadPlc():
    def __init__(self):
        self.IP_addres_PLC = '192.168.0.10'
        self.port_addres_PLC = 8501

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)
        time.sleep(1.0)

    #"Connect to PLC:"
    def socket_connect(self,host, port):
        try:
            self.soc.connect((host, port))
            print('Connected')
            return True
        except OSError:
            time.sleep(2)
            print('Connect fail will try reconect aftef 2s')
            return False

    #"Read bit. Ex: MR10 => deviceType: 'M', deviceNo: 100"
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    def read_bit(self,deviceType: str = 'R', deviceNo: str = '0', format: str = '', length: int = 1):
        if length > 1:
            device = 'RDS' + ' ' + deviceType + deviceNo + format + ' ' + str(length) + '\x0D'
        else:
            device = 'RD' + ' ' + deviceType + deviceNo + format + '\x0D' 
        dataformat = device.encode()
        self.soc.sendall(dataformat)
        dataRecv = self.soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataBit = dataRecvDec.split(' ')
        dataBit[len(dataBit)-1] = dataBit[len(dataBit)-1][0]
        return np.int_(dataBit)


    #"Read register:"
    # deviceType: 'D', deviceNo: 100,
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    def read_register(self,deviceType: str = 'DM', deviceNo: str = '0', format: str = '', length: int = 1):
        if length > 1:
            device = 'RDS' + ' ' + deviceType + deviceNo + format + ' ' + str(length) + '\x0D'
        else:
            device = 'RD' + ' ' + deviceType + deviceNo + format + '\x0D' 
        dataformat = device.encode()
        self.soc.sendall(dataformat)
        dataRecv = self.soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataReg = dataRecvDec.split(' ')
        dataReg[len(dataReg)-1] = dataReg[len(dataReg)-1][:-2]
        return np.int_(dataReg)

def main(args=None):
    pc_read_plc = PcReadPlc()
    while True:
        data = pc_read_plc.read_register(deviceType='DM',deviceNo='1', format='.U',length=4)
        # print(len(data))
        for i in data:
            print(i)

if __name__ == '__main__':
    main()

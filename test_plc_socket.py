import socket
import time


def socket_connect(host, port):
    try:
        soc.connect((host, port))
        print("connect OK")
        return True
    except OSError:
        print("connect error")
        time.sleep(2)
        return False

IP_addres_PLC = '192.168.1.1'
port_addres_PLC = 8501
soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_is_connected = False
while _is_connected == False:
    _is_connected = socket_connect(IP_addres_PLC, port_addres_PLC)

#"------------Read device-------------"
# deviceType: 'DM'
# deviceNo: 100
# format: .U: Unsigned 16-bit DEC
#         .S: Signed 16-bit DEC
#         .D: Unsigned 32-bit DEC
#         .L: Signed 32-bit DEC
#         .H: 16-bit HEX
# length: 3
def read_device(deviceType: str = 'DM', deviceNo: int = 0, format: str = '', length: int = 1):
    try:
        if length > 1:
            device = 'RDS' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(length) + '\x0D'
        else:
            device = 'RD' + ' ' + deviceType + str(deviceNo) + format + '\x0D' 
        dataformat = device.encode()
        soc.sendall(dataformat)
        dataRecv = soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataResp = dataRecvDec.split(' ')
        dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
        for i in range (0,len(dataResp)):
            dataResp[i] = int(dataResp[i])
        return dataResp
    except Exception as e:
        print(e)
        return False
    

#"------------Write device-------------"
# deviceType: 'DM'
# deviceNo: 100
# format: .U: Unsigned 16-bit DEC
#         .S: Signed 16-bit DEC
#         .D: Unsigned 32-bit DEC
#         .L: Signed 32-bit DEC
#         .H: 16-bit HEX
# length: 3
# data: [1,2,3]
def write_device( deviceType: str = 'R', deviceNo: int = 0, format: str = '', length: int = 1, data: list = []):
    try:
        if length != len(data):
            print('write data error (length of data not correct!)')
            return False
        
        if length > 1:
            device = 'WRS' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(length)
            for i in data:
                device += (' ' + str(i))
            device += '\x0D'
        else:
            device = 'WR' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(data[0]) + '\x0D'
        
        dataformat = device.encode()
        soc.sendall(dataformat)
        dataRecv = soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataResp = dataRecvDec.split(' ')
        dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
        if (dataResp[0]== 'OK'):
            return True
        return False
    except Exception as e:
        print(e)
        return False

print(write_device('MR',1,'.U',3,[1,0,1]))
print(read_device('MR',1,'.U',3))
print(read_device('CM',700,'.U',7))


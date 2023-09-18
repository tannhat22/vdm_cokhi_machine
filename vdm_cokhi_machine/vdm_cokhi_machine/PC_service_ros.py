import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import os

from std_msgs.msg import Bool, String
from vdm_cokhi_machine_msgs.msg import StateMachinesStamped
from vdm_cokhi_machine_msgs.srv import GetAllMachineName, GetMachineData, ResetMachine

class PcService(Node):
    def __init__(self):
        super().__init__('PC_read_PLC')

        self.IP_addres_PLC = '192.168.0.11'
        self.port_addres_PLC = 8501

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)
        time.sleep(1.0)

        # Database path:
        self.database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.create_table_group_machines_db(self.tableName)
        self.machine_info = self.get_all_machine_name_db()

        # Ros Services:
        self.getAllMachineName_srv = self.create_service(GetAllMachineName, 'get_all_machine_name', self.get_all_machine_name_cb)
        self.getMachineData_srv = self.create_service(GetMachineData, 'get_machine_data', self.get_machine_data_cb)
        self.resetMachine_srv = self.create_service(ResetMachine, 'reset_machine', self.reset_machine_cb)

        # Ros pub, sub:
        self.pub_update_machines_table = self.create_publisher(Bool, '/update_machines_table', 10)

        self.bool_true = Bool()
        self.bool_true.data = True

        self.bool_false = Bool()
        self.bool_false.data = False
        
        # Address all device:
        self.clock_red_res = ['CM',700,'.U']

        self.password_read_res = ['DM',1000,'.U',1]
        self.password_write_res = ['DM',1000,'.U',1]

        self.reset_machine_bit = ['MR',1000,'.U',1]
        self.reset_machine_res = ['DM',1000,'.U',1]

        self.data_machine_length = 40
        self.data_machines_res = ['DM',1000,'.U',self.data_machine_length * self.machine_info['quantity']]
        self.separate_machine = 10
        self.data_machine_res_structure = {
            'signalLight': [1,0],
            'noload': [2,1],
            'underload': [2,3],
            'valueSetting': [3,5],
            'timeReachSpeed': [1,8],
            'totalDays': [1,9],
            'noloadHistory': [30,10],
            'underloadHistory': [30,40],
            'years': [30,70],
            'months': [30,100],
            'days': [30,130],
        }

        self.get_logger().info("is running!!!!!!!!!!")


    #"Connect to PLC:"
    def socket_connect(self,host, port):
        try:
            self.soc.connect((host, port))
            self.get_logger().info("is connected to PLC success")
            return True
        except OSError:
            self.get_logger().info("can't connect to PLC, will auto reconneting after 2s")
            time.sleep(2)
            return False

    def create_table_group_machines_db(self, tableName):
        try:
            conn = sqlite3.connect(self.database_path)
            print("Opened database successfully")
            cur = conn.cursor()
            cur.execute('CREATE TABLE IF NOT EXISTS ' + tableName +
            ''' (ID INT PRIMARY KEY     NOT NULL,
                 NAME           TEXT    NOT NULL);''')
        except Exception as e:
            print(Exception)
        return

    # Lấy dữ liệu tên của tất cả các máy từ database
    def get_all_machine_name_db(self):
        try:
            conn = sqlite3.connect(self.database_path)
            cur = conn.cursor()
            cur.execute("SELECT * from " + self.tableName)
            num = 0
            result = {
                'quantity': 0,
                'machineName': []
            }
            for row in cur:
                result['machineName'].append(row[1]) 
                num += 1
            result['quantity'] = num

            return result
        except Exception as e:
            print(Exception)

    def create_machine_db(self, id, days):
        return
    
    def update_machine_db(self, id, days):
        return

    def delete_machine_db(self, id):
        return

    def get_all_machine_name_cb(self, request: GetAllMachineName.Request, respone: GetAllMachineName.Response):
        if request.get_allname:
            infoNames = self.get_all_machine_name_db()
            respone.machines_quantity = infoNames['quantity']
            respone.machines_name = infoNames['machineName']
            return respone

    # Lấy dữ liệu của một máy dựa trên yêu cầu ID và số ngày cần lấy
    def get_machine_data_cb(self, request: GetMachineData.Request, respone: GetMachineData.Response):
        # Lấy toàn bộ dữ liệu của máy theo ID từ PLC
        startRes = self.data_machines_res[1] + (self.data_machine_length + self.separate_machine)*(request.id_machine - 1)
        data = self.read_device(self.data_machines_res[0],
                                startRes,
                                self.data_machines_res[2],
                                self.data_machine_length)
        
        #Lấy tổng số ngày đã lưu và lọc ngày dựa trên yêu cầu
        total_days = data[self.data_machine_res_structure['totalDays'][1]]
        daysFilter = 0
        if total_days > request.days:
            daysFilter = total_days - request.days

        dataDates = [
            data[(self.data_machine_res_structure['days'][1] + daysFilter):(self.data_machine_res_structure['days'][1] + total_days)],
            data[(self.data_machine_res_structure['months'][1] + daysFilter):(self.data_machine_res_structure['months'][1] + total_days)],
            data[(self.data_machine_res_structure['years'][1] + + daysFilter):(self.data_machine_res_structure['years'][1] + total_days)],
        ]

        # Chuẩn hóa dữ liệu cho client
        datesResp = []
        for i in range(0,total_days):
            datesResp.append(
                str(dataDates[0][i]) + '/' + str(dataDates[1][i]) + '/' + '20' +str(dataDates[2][i])
            )
        noloadResp = data[(self.data_machine_res_structure['noloadHistory'][1] + daysFilter):(self.data_machine_res_structure['noloadHistory'][1] + total_days)]
        underloadResp = data[(self.data_machine_res_structure['underloadHistory'][1] + daysFilter):(self.data_machine_res_structure['underloadHistory'][1] + total_days)]

        respone.dates = datesResp
        respone.noload_time = noloadResp
        respone.underload_time = underloadResp
        return respone

    def reset_machine_cb(self, request: ResetMachine.Request, respone: ResetMachine.Response):
        if self.check_password(request.password):
            if self.write_device(self.reset_machine_res[0],
                                self.reset_machine_res[1],
                                self.reset_machine_res[2],
                                self.reset_machine_res[3],
                                [request.id_machine]):
                if self.write_device(self.password_write_res[0],
                                    self.password_write_res[1],
                                    self.password_write_res[2],
                                    self.password_write_res[3],
                                    [request.password]):
                    if self.write_device(self.reset_machine_bit[0],
                                        self.reset_machine_bit[1],
                                        self.reset_machine_bit[2],
                                        self.reset_machine_bit[3],
                                        [1]):
                        respone.success = True
                        return respone
        respone.success = False
        return respone

    def check_password(self, passwordCheck):
        password = self.read_device(self.password_read_res[0],
                                    self.password_read_res[1],
                                    self.password_read_res[2],
                                    self.password_read_res[3])[0]
        if password == passwordCheck:
            return True
        return False
    
    #"------------Read device-------------"
    # deviceType: 'DM'
    # deviceNo: 100
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    # length: 3
    def read_device(self,deviceType: str = 'DM', deviceNo: int = 0, format: str = '', length: int = 1):
        if length > 1:
            device = 'RDS' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(length) + '\x0D'
        else:
            device = 'RD' + ' ' + deviceType + str(deviceNo) + format + '\x0D' 
        dataformat = device.encode()
        self.soc.sendall(dataformat)
        dataRecv = self.soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataResp = dataRecvDec.split(' ')
        dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
        for i in range (0,len(dataResp)):
            dataResp[i] = int(dataResp[i])
        return dataResp
    

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
    def write_device(self, deviceType: str = 'R', deviceNo: int = 0, format: str = '', length: int = 1, data: list = []):
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
        self.soc.sendall(dataformat)
        dataRecv = self.soc.recv(1024)
        dataRecvDec = dataRecv.decode()
        dataResp = dataRecvDec.split(' ')
        dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
        
        if (dataResp[0] + dataResp[1] == 'OK'):
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    pc_service = PcService()
    rclpy.spin(pc_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

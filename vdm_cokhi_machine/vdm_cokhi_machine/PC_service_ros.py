import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import math

from std_msgs.msg import Bool
from vdm_cokhi_machine_msgs.srv import GetAllMachineName, GetMachineData, ResetMachine, CreateMachine, UpdateMachine, DeleteMachine

class PcService(Node):
    def __init__(self):
        super().__init__('PC_service_ros')

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
        self.conn = sqlite3.connect(self.database_path)
        self.cur = self.conn.cursor()
        self.create_table_group_machines_db(self.tableName)
        self.machine_info = self.get_all_machine_name_db()

        # Ros Services:
        self.getAllMachineName_srv = self.create_service(GetAllMachineName, 'get_all_machine_name', self.get_all_machine_name_cb)
        self.getMachineData_srv = self.create_service(GetMachineData, 'get_machine_data', self.get_machine_data_cb)
        self.resetMachine_srv = self.create_service(ResetMachine, 'reset_machine', self.reset_machine_cb)
        self.createMachine_srv = self.create_service(CreateMachine, 'create_machine', self.create_machine_cb)
        self.updateMachine_srv = self.create_service(UpdateMachine, 'update_machine', self.update_machine_cb)
        self.deleteMachine_srv = self.create_service(DeleteMachine, 'delete_machine', self.delete_machine_cb)

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

        # self.dataMachines_res = ['DM',1000,'.U',self.dataMachine_length * self.machine_info['quantity']]
        # self.dataMachine_length = 7
        # self.separateMachine = 10
        # self.dataMachine_res_structure = {
        #     'signalLight': [1,0],
        #     'noload': [1,1],
        #     'underload': [1,2],
        #     'valueSetting': [3,3],
        #     'timeReachSpeed': [1,6],
        # }

        self.dataMachineHistory_length = 151
        self.dataMachineHistory_res = ['EM',1000,'.U',self.dataMachineHistory_length]
        self.separateMachineHistory = 10
        self.dataMachineHistory_res_structure = {
            'totalDays': [1,0],
            'noload': [30,1],
            'underload': [30,31],
            'years': [30,61],
            'months': [30,91],
            'days': [30,121],
        }

        self.status = {
            'success': 'All success',
            'notFound': 'Data is not found!',
            'nameInvalid': 'Machine name Invalid!',
            'nameInuse': 'Machine name already in use!',
            'passErr': 'Password is Incorrect!',
            'dbErr': 'Connection with database error!',
            'socketErr': 'Connection between Raspberry and PLC error!',
            'fatalErr': 'Something is wrong, please check all system!'
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
            self.cur.execute('CREATE TABLE IF NOT EXISTS ' + tableName +
            ''' (ID INTEGER PRIMARY KEY     NOT NULL,
                 NAME           TEXT    NOT NULL);''')
        except Exception as e:
            print(Exception)
        return

    # Lấy dữ liệu tên của tất cả các máy từ database
    def get_all_machine_name_db(self):
        try:
            self.cur.execute("SELECT * from " + self.tableName)
            num = 0
            result = {
                'quantity': 0,
                'machineName': [],
                'idMachines': []
            }
            for row in self.cur:
                result['idMachines'].append(row[0])
                result['machineName'].append(row[1]) 
                num += 1
            result['quantity'] = num
        
            return result
        except Exception as e:
            print(e)
            return False

    # Tạo thêm máy mới vào database
    def create_machine_db(self, name):
        try:
            self.cur.execute("INSERT INTO " + self.tableName + " (NAME) VALUES (?)", (name,))
            
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False
    
    # Sửa machine trong database:
    def update_machine_db(self, id, newName):
        try:
            self.cur.execute("UPDATE " + self.tableName + " SET NAME = ? WHERE ID = ?", (newName, id))
            
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False

    # Xóa machine trong database
    def delete_machine_db(self, id):
        try:
            self.cur.execute("DELETE FROM " + self.tableName + " WHERE ID = ?", (id,))
            
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False

    # Kiểm tra tên machine có tồn tại trong database:
    def checkNameIsExists(self,name):
        try:
            self.cur.execute("SELECT ID FROM " + self.tableName + " WHERE name = ?",(name,))
            data = self.cur.fetchone()
            if data is None:
                return False
            else:
                return True
        except Exception as e:
            print(e)
            return False

    def get_all_machine_name_cb(self, request: GetAllMachineName.Request, response: GetAllMachineName.Response):
        if request.get_allname:
            self.machine_info = self.get_all_machine_name_db()
            if not self.machine_info:
                response.success = False
                response.status = self.status['dbErr']
                return response

            response.success = True
            response.status = self.status['success']
            response.machines_quantity = self.machine_info['quantity']
            response.id_machines = self.machine_info['idMachines']
            response.machines_name = self.machine_info['machineName']
            return response

    # Lấy dữ liệu của một máy dựa trên yêu cầu ID và số ngày cần lấy
    def get_machine_data_cb(self, request: GetMachineData.Request, response: GetMachineData.Response):
        # Lấy toàn bộ dữ liệu của máy theo ID từ PLC
        if not self.machine_info:
            response.success = False
            response.status = self.status['dbErr']
            return response

        try:
            indexMachine = self.machine_info['idMachines'].index(request.id_machine)
        except Exception as e:
            print(e)
            response.success = False
            response.status = self.status['notFound']
            return response
        
        startRes = self.dataMachineHistory_res[1] + (self.dataMachineHistory_length + self.separateMachineHistory)*(indexMachine)
        data = self.read_device(self.dataMachineHistory_res[0],
                                startRes,
                                self.dataMachineHistory_res[2],
                                self.dataMachineHistory_length)
        
        if not data:
            response.success = False
            response.status = self.status['socketErr']
            return response
        
        #Lấy tổng số ngày đã lưu và lọc ngày dựa trên yêu cầu
        total_days = data[self.dataMachineHistory_res_structure['totalDays'][1]]
        daysFilter = 0
        if total_days > request.days:
            daysFilter = total_days - request.days

        dataDates = [
            data[(self.dataMachineHistory_res_structure['days'][1] + daysFilter):(self.dataMachineHistory_res_structure['days'][1] + total_days)],
            data[(self.dataMachineHistory_res_structure['months'][1] + daysFilter):(self.dataMachineHistory_res_structure['months'][1] + total_days)],
            data[(self.dataMachineHistory_res_structure['years'][1] + daysFilter):(self.dataMachineHistory_res_structure['years'][1] + total_days)],
        ]
        dataNoload = data[(self.dataMachineHistory_res_structure['noload'][1] + daysFilter):(self.dataMachineHistory_res_structure['noload'][1] + total_days)]
        dataUnderload = data[(self.dataMachineHistory_res_structure['underload'][1] + daysFilter):(self.dataMachineHistory_res_structure['underload'][1] + total_days)]
        
        # Chuẩn hóa dữ liệu cho client
        datesResp = []
        noloadResp = []
        underloadResp = []
        for i in range(0,total_days):
            datesResp.append(
                str(dataDates[0][i]) + '/' + str(dataDates[1][i]) + '/' + '20' +str(dataDates[2][i])
            )
            noload = dataNoload[i]
            underload = dataUnderload[i]
            noloadResp.append(noload)
            underloadResp.append(underload)

        response.success = True
        response.status = self.status['success']
        response.dates = datesResp
        response.noload = noloadResp
        response.underload = underloadResp
        return response

    def reset_machine_cb(self, request: ResetMachine.Request, response: ResetMachine.Response):
        if self.check_password(request.password):
            indexMachine = self.machine_info['idMachines'].index(request.id_machine)
            if (self.write_device(self.reset_machine_res[0],
                                self.reset_machine_res[1],
                                self.reset_machine_res[2],
                                self.reset_machine_res[3],
                                [indexMachine]) and
                self.write_device(self.password_write_res[0],
                                self.password_write_res[1],
                                self.password_write_res[2],
                                self.password_write_res[3],
                                [request.password]) and
                self.write_device(self.reset_machine_bit[0],
                                self.reset_machine_bit[1],
                                self.reset_machine_bit[2],
                                self.reset_machine_bit[3],
                                [1])):
                response.success = True
                return response
            
            response.success = False
            response.status = self.status['socketErr']

        response.success = False
        response.status = self.status['passErr']
        return response

    def create_machine_cb(self, request: CreateMachine.Request, response: CreateMachine.Response):
        if self.check_password(request.password):
            if request.name == '':
                response.success = False
                response.status = self.status['nameInvalid']

            elif self.checkNameIsExists(request.name):
                response.success = False
                response.status = self.status['nameInuse']

            elif not self.create_machine_db(request.name):
                # self.pub_update_machines_table.publish(self.bool_true)
                response.success = False
                response.status = self.status['dbErr']
            
            else:
                self.pub_update_machines_table.publish(self.bool_true)
                response.success = True
                response.status = self.status['success']

            return response               

        response.success = False
        response.status = self.status['passErr']
        return response

    def update_machine_cb(self, request: UpdateMachine.Request, response: UpdateMachine.Response):
        if self.check_password(request.password):            
            if request.new_name == '':
                response.success = False
                response.status = self.status['nameInvalid']

            elif self.checkNameIsExists(request.new_name):
                response.success = False
                response.status = self.status['nameInuse']

            elif not self.update_machine_db(request.id_machine, request.new_name):
                response.success = False
                response.status = self.status['dbErr']
            
            else:
                self.pub_update_machines_table.publish(self.bool_true)
                response.success = True
                response.status = self.status['success']

            return response

        response.success = False
        response.status = self.status['passErr']
        return response

    def delete_machine_cb(self, request: DeleteMachine.Request, response: DeleteMachine.Response):
        if self.check_password(request.password):
            if self.delete_machine_db(request.id_machine):
                self.pub_update_machines_table.publish(self.bool_true)
                response.success = True
                response.status = self.status['success']
            else:
                response.success = False
                response.status = self.status['dbErr']
            return response

        response.success = False
        response.status = self.status['passErr']
        return response

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
        try:
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
    def write_device(self, deviceType: str = 'R', deviceNo: int = 0, format: str = '', length: int = 1, data: list = []):
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
            self.soc.sendall(dataformat)
            dataRecv = self.soc.recv(1024)
            dataRecvDec = dataRecv.decode()
            dataResp = dataRecvDec.split(' ')
            dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
            
            if (dataResp[0] + dataResp[1] == 'OK'):
                return True
            return False
        except Exception as e:
            print(e)
            return False        

def main(args=None):
    rclpy.init(args=args)
    pc_service = PcService()
    rclpy.spin(pc_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc_service.conn.close()
    pc_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

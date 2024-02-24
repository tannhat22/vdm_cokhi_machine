import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import datetime
from collections import Counter

from vdm_cokhi_machine_msgs.msg import OverralMachine, StateMachine, StateMachinesStamped
from vdm_cokhi_machine_msgs.srv import GetAllMachineName, GetMachineData, ResetMachine
from vdm_cokhi_machine_msgs.srv import CreateMachine, UpdateMachine, DeleteMachine


class PlcService(Node):
    def __init__(self):
        super().__init__('PLC_service_ros')
        # Cấu hình các thông số quan trọng:
        self.IP_addres_PLC = '192.168.1.1'
        self.port_addres_PLC = 8501
        self.maximumDates = 365

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)
        time.sleep(1.0)

        # Database path:
        self.database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'      
        # self.database_path = '/home/raspberry/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.conn = sqlite3.connect(self.database_path)
        self.cur = self.conn.cursor()
        self.create_table_group_machines_db(self.tableName)
        self.machine_info = self.get_machines_inform_db()
        self.types_info = self.array_type_to_dict(self.machine_info['machineType'])

        # Ros Services:
        self.getAllMachineName_srv = self.create_service(GetAllMachineName, 'get_all_machine_name', self.get_all_machine_name_cb)
        self.getMachineData_srv = self.create_service(GetMachineData, 'get_machine_data', self.get_machine_data_cb)
        self.resetMachine_srv = self.create_service(ResetMachine, 'reset_machine', self.reset_machine_cb)
        self.createMachine_srv = self.create_service(CreateMachine, 'create_machine', self.create_machine_cb)
        self.updateMachine_srv = self.create_service(UpdateMachine, 'update_machine', self.update_machine_cb)
        self.deleteMachine_srv = self.create_service(DeleteMachine, 'delete_machine', self.delete_machine_cb)

        # Ros pub, sub:
        self.pub_state_machine = self.create_publisher(StateMachinesStamped, '/state_machines', 10)


        #------ Address all device -------:
        ## System data:
        ## Clock resgister:
            # year(0-99):       CM700
            # month(1-12):      CM701
            # day(1-31):        CM702
            # hour(0-23):       CM703
            # minute(0-59):     CM704
            # second(0-59):     CM705
            # day-of-week(0-6): CM706 (0: sunday, 1: monday, 2: tuesday, 3: wednesday, 4: thursday, 5: friday, 6: saturday)    
        self.clock_res = ['CM',700,'.U',7]

        self.save_data_bit = ['MR',202,'.U',1]

        self.password = '10064'
        self.password_write_res = ['DM',500,'.U',1]

        self.reset_bit = ['MR',200,'.U',1]
        self.reset_machine_bit = ['MR',1000,'.U',1]
        self.max_bit_reset = 15
        # self.reset_change_bit = ['MR',1100, '.U',1]
        self.reset_machine_res = ['DM',0,'.U',1]
        self.reset_separate = 0

        ## Realtime data:
        self.dataMachine_length = 4
        self.separateMachine = 6
        self.dataMachines_res = ['DM',1014,'.U',(self.dataMachine_length + self.separateMachine) * self.machine_info['quantity']]
        self.dataMachine_res_structure = {
            'signalLight': [1,0],
            'noload': [1,1],
            'underload': [1,2],
            'offtime': [1,3],
            # 'valueSetting': [3,4],
            # 'timeReachSpeed': [1,7],
        }

        # Status for response services:
        self.status = {
            'success': 'Thành công',
            'notFound': 'Dữ liệu không tìm thấy!',
            'nameInvalid': 'Tên máy không hợp lệ!',
            'nameInuse': 'Tên máy đã được sử dụng!',
            'typeInvalid': 'Loại máy không hợp lệ!',
            'passErr': 'Sai mật khẩu!',
            'resetErr': 'Reset máy lỗi!',
            'dbErr': 'Lỗi cơ sở dữ liệu!',
            'socketErr': 'Lỗi kết nối Raspberry và PLC!',
            'fatalErr': 'Something is wrong, please check all system!'
        }

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
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

    # Tạo bảng database nếu chưa có:
    def create_table_group_machines_db(self, tableName):
        try:
            self.cur.execute('CREATE TABLE IF NOT EXISTS ' + tableName +
            ''' (ID INTEGER PRIMARY KEY     NOT NULL,
                 NAME           TEXT    NOT NULL,
                 TYPE           TEXT    NOT NULL,
                 PLC            TEXT    NOT NULL,
                 ADDRESS     INTEGER    NOT NULL);''')
            
            self.cur.execute("SELECT * from " + self.tableName)
            rows = self.cur.fetchall()
            if len(rows) > 0:
                for row in rows:
                    self.create_table_machine_history_db(row[1])
        except Exception as e:
            print(Exception)
        return
    
    # Tạo bảng database history cho máy được cài đặt nếu chưa có:
    def create_table_machine_history_db(self, tableName):
        try:
            self.cur.execute('CREATE TABLE IF NOT EXISTS ' + tableName +
            ''' (ID INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                 DATE TIMESTAMP NOT NULL,
                 NOLOAD INTEGER NOT NULL,
                 UNDERLOAD INTEGER NOT NULL,
                 OFFTIME INTEGER NOT NULL);''')
        except Exception as e:
            print(Exception)
        return

    # Lấy dữ liệu tên của tất cả các máy từ database
    def get_machines_inform_db(self):
        try:
            self.cur.execute("SELECT * from " + self.tableName)
            rows = self.cur.fetchall()
            # num = 0
            result = {
                'quantity': 0,
                'idMachines': [],
                'machineName': [],
                'machineType': []
            }
            for row in rows:
                result['quantity'] += 1
                result['idMachines'].append(row[0])
                result['machineName'].append(row[1])
                result['machineType'].append(row[2])
                # num += 1
            # result['quantity'] = num
            self.types_info = self.array_type_to_dict(result['machineType'])
            return result
        except Exception as e:
            print(e)
            return False
    
    def get_machine_name_db(self, id):
        try:
            self.cur.execute("SELECT NAME FROM " + self.tableName + " WHERE ID = ?",(id,))
            machineName = self.cur.fetchone()[0]
            return machineName
        except Exception as e:
            print(e)
            self.get_logger().info(f'Exeption: {e}')
            return False

    # Lấy dữ liệu theo ngày của một máy từ database
    def get_history_machine_db(self, tableName):
        try:
            # self.get_logger().info(f'machine name: {tableName}')
            self.cur.execute("SELECT * from " + tableName)
            rows = self.cur.fetchall()
            result = {
                'totalDays': 0,
                'dates': [],
                'noload': [],
                'underload': [],
                'offtime': []
            }
            for row in rows:
                result['totalDays'] +=1
                result['dates'].append(row[1])
                # result['dates'].append(row[1].date().strftime("%m/%d/%Y"))
                result['noload'].append(row[2]) 
                result['underload'].append(row[3])
                result['offtime'].append(row[4])


            return result
        except Exception as e:
            self.get_logger().info(f'{e}')
            return False

    # Tạo thêm máy mới vào database
    def add_machine_db(self, name, type):
        try:
            self.cur.execute("INSERT INTO " + self.tableName + " (NAME, TYPE) VALUES (?, ?)", (name, type))
            self.create_table_machine_history_db(name)
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False
    
    # Thêm dữ liệu ngày mới vào bảng database:
    def add_history_data_db(self, tableName, date, noLoad, underLoad, offtime):
        try:
            self.cur.execute("SELECT * from " + tableName)
            totalDates = len(self.cur.fetchall())
            if totalDates >= self.maximumDates:
                self.cur.execute("DELETE FROM " + tableName + " WHERE ID = 1")
                self.cur.execute("CREATE TABLE momenttable AS SELECT * FROM " + tableName)
                self.cur.execute("DELETE FROM " + tableName)
                self.cur.execute("DELETE FROM sqlite_sequence WHERE name='" + tableName + "'")
                self.cur.execute("INSERT INTO " + tableName + " (DATE, NOLOAD, UNDERLOAD, OFFTIME) SELECT DATE, NOLOAD, UNDERLOAD, OFFTIME FROM momenttable")
                self.delete_table("momenttable")
                self.conn.commit()
            
            self.cur.execute("INSERT INTO " + tableName + " (DATE, NOLOAD, UNDERLOAD, OFFTIME) VALUES (?, ?, ?, ?)", (date, noLoad, underLoad, offtime))
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False

    # Sửa tên bảng trong database:
    def update_table_name(self, tableOldName, tableNewName):
        try:
            self.cur.execute("ALTER TABLE " + tableOldName + " RENAME TO " + tableNewName)
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False

    # Sửa machine trong database:
    def update_machine_db(self, id, newName, newType):
        try:
            machineName = self.get_machine_name_db(id)
            if self.update_table_name(machineName, newName):
                self.cur.execute("UPDATE " + self.tableName + " SET NAME = ?, TYPE = ? WHERE ID = ?", (newName, newType, id))
                self.conn.commit()
                return True
            return False
        except Exception as e:
            print(e)
            return False

    # Xóa bảng database:
    def delete_table(self, tableName):
        try:
            self.cur.execute("DROP TABLE IF EXISTS " + tableName)
        except Exception as e:
            print(e)
            return False

    # Xóa machine trong database
    def delete_machine_db(self, id):
        try:
            machineName = self.get_machine_name_db(id)
            if self.delete_table(machineName):
                self.cur.execute("DELETE FROM " + self.tableName + " WHERE ID = ?", (id,))
                self.conn.commit()
                return True
            return False
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

    # Chuyển mảng 'loại máy' của các máy về thành dict và với giá trị là số lượng máy mỗi loại
    def array_type_to_dict(self, type_arr):
        count_dict = Counter(type_arr)
        result_dict = dict(count_dict)
        return result_dict

    def get_all_machine_name_cb(self, request: GetAllMachineName.Request, response: GetAllMachineName.Response):
        if request.get_allname:
            self.machine_info = self.get_machines_inform_db()
            if not self.machine_info:
                response.success = False
                response.status = self.status['dbErr']
                return response

            response.success = True
            response.status = self.status['success']
            response.machines_quantity = self.machine_info['quantity']
            response.id_machines = self.machine_info['idMachines']
            response.machines_name = self.machine_info['machineName']
            response.machines_type = self.machine_info['machineType']
            return response

    # Lấy dữ liệu của một máy dựa trên yêu cầu ID và số ngày cần lấy
    def get_machine_data_cb(self, request: GetMachineData.Request, response: GetMachineData.Response):
        if not self.machine_info:
            response.success = False
            response.status = self.status['dbErr']
            return response
        
        # Lấy dữ liệu ngày từ database:
        machineName = self.get_machine_name_db(request.id_machine)
        dataHistory = self.get_history_machine_db(machineName)
        if not dataHistory: 
            return response

        # Lấy tổng số ngày đã lưu và lọc ngày dựa trên yêu cầu
        i = 0
        if (dataHistory['totalDays'] > request.days):
            i = dataHistory['totalDays'] - request.days
        
        response.success = True
        response.status = self.status['success']
        response.dates = dataHistory['dates'][i:]
        response.noload = dataHistory['noload'][i:]
        response.underload = dataHistory['underload'][i:]
        response.offtime = dataHistory['offtime'][i:]

        return response

    def reset_machine_cb(self, request: ResetMachine.Request, response: ResetMachine.Response):
        self.get_logger().info(f'Reset machine: {request.id_machine}')
        # response.success = True
        # return response 
        if self.check_password(request.password):
            indexMachine = self.machine_info['idMachines'].index(request.id_machine)

            if (self.write_device(self.reset_machine_res[0],
                                self.reset_machine_res[1],
                                self.reset_machine_res[2],
                                self.reset_machine_res[3],
                                [request.id_machine]) and
                self.write_device(self.password_write_res[0],
                                self.password_write_res[1],
                                self.password_write_res[2],
                                self.password_write_res[3],
                                [request.password]) and
                self.write_device(self.reset_bit[0],
                                self.reset_bit[1],
                                self.reset_bit[2],
                                self.reset_bit[3],
                                [1])):
                a = 0
                resetOK = False
                while not resetOK: 
                    resetOK = not self.read_device(self.reset_bit[0],
                                               self.reset_bit[1],
                                               self.reset_bit[2],
                                               self.reset_bit[3])[0]
                    if a >= 1200:
                        response.status = self.status['resetErr']
                        break
                    a += 1

                response.success = resetOK
            
            else:
                response.success = False
                response.status = self.status['socketErr']

        else:
            response.success = False
            response.status = self.status['passErr']

        return response

    def create_machine_cb(self, request: CreateMachine.Request, response: CreateMachine.Response):
        if self.check_password(request.password):
            if request.name == '':
                response.success = False
                response.status = self.status['nameInvalid']

            elif request.type == '':
                response.success = False
                response.status = self.status['typeInvalid']

            elif self.checkNameIsExists(request.name.upper()):
                response.success = False
                response.status = self.status['nameInuse']

            elif not self.add_machine_db(request.name.upper(), request.type):
                response.success = False
                response.status = self.status['dbErr']
            
            else:
                self.update_machineInfo()
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

            elif request.new_type == '':
                response.success = False
                response.status = self.status['typeInvalid']

            elif self.checkNameIsExists(request.new_name.upper()):
                response.success = False
                response.status = self.status['nameInuse']

            elif not self.update_machine_db(request.id_machine, request.new_name.upper(), request.new_type):
                response.success = False
                response.status = self.status['dbErr']
            
            else:
                self.update_machineInfo()
                response.success = True
                response.status = self.status['success']

            return response

        response.success = False
        response.status = self.status['passErr']
        return response

    def delete_machine_cb(self, request: DeleteMachine.Request, response: DeleteMachine.Response):
        if self.check_password(request.password):
            if self.delete_machine_db(request.id_machine):
                self.update_machineInfo()
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
        if passwordCheck == self.password:
            return True
        return False

    def update_machineInfo(self):
        self.machine_info = self.get_machines_inform_db()
        self.dataMachines_res = ['DM',self.dataMachines_res[1],'.U',(self.dataMachine_length + self.separateMachine) * self.machine_info['quantity']]
        return
    
    def handleSaveData(self,data):
        dataClock = self.read_device(self.clock_res[0],
                                     self.clock_res[1],
                                     self.clock_res[2],
                                     self.clock_res[3])

        date = datetime.datetime(2000 + dataClock[0],dataClock[1],dataClock[2],dataClock[3],dataClock[4],dataClock[5])

        for i in range(0,self.machine_info['quantity']):
            j = i * (self.dataMachine_length + self.separateMachine)
            name = self.machine_info['machineName'][i]
            noload = data[j + self.dataMachine_res_structure['noload'][1]]
            underload = data[j + self.dataMachine_res_structure['underload'][1]]
            offtime = data[j + self.dataMachine_res_structure['offtime'][1]]
            if not self.add_history_data_db(name,date,noload,underload,offtime):
                self.get_logger().info("ERROR: Save data of day error!!!")
                return False
        
        self.get_logger().info("Save data of day success!")
        return True
    

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
            dataRecv = self.soc.recv(4096)
            dataRecvDec = dataRecv.decode()
            dataResp = dataRecvDec.split(' ')
            self.get_logger().warn(f'len data: {len(dataResp)}')
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
            
            if (dataResp[0]== 'OK'):
                return True
            return False
        except Exception as e:
            print(e)
            return False
    
    
    def timer_callback(self):
        if not self.machine_info: return

        saveDataBit = self.read_device(self.save_data_bit[0],
                                      self.save_data_bit[1],
                                      self.save_data_bit[2],
                                      self.save_data_bit[3])[0]

        dataMachines = self.read_device(self.dataMachines_res[0],
                                        self.dataMachines_res[1],
                                        self.dataMachines_res[2],
                                        self.dataMachines_res[3])
        
        if saveDataBit:
            self.handleSaveData(dataMachines)
            self.write_device(self.save_data_bit[0],
                              self.save_data_bit[1],
                              self.save_data_bit[2],
                              self.save_data_bit[3],[0])
        
        state_machines = []
        overral_machines_dict = {}
        for i in range(0,self.machine_info['quantity']):
            j = i * (self.dataMachine_length + self.separateMachine)
            machineState = StateMachine()
            machineState.name = self.machine_info['machineName'][i]
            machineState.type = self.machine_info['machineType'][i]
            machineState.signal_light = dataMachines[j + self.dataMachine_res_structure['signalLight'][1]]
            machineState.noload = dataMachines[j + self.dataMachine_res_structure['noload'][1]]
            machineState.underload = dataMachines[j + self.dataMachine_res_structure['underload'][1]]
            machineState.offtime = dataMachines[j + self.dataMachine_res_structure['offtime'][1]]
            # machineState.value_setting.min = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1]]
            # machineState.value_setting.max = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 1]
            # machineState.value_setting.current = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 2]
            # machineState.time_reachspeed = dataMachines[j + self.dataMachine_res_structure['timeReachSpeed'][1]]
            state_machines.append(machineState)

            if machineState.type in overral_machines_dict:
                overral_machines_dict[machineState.type][0] += 1
                overral_machines_dict[machineState.type][1] += machineState.noload
                overral_machines_dict[machineState.type][2] += machineState.underload
                overral_machines_dict[machineState.type][3] += machineState.offtime
            else:
                overral_machines_dict[machineState.type] = [1, machineState.noload, machineState.underload, machineState.offtime]

        # Tính số giờ theo từng loại máy
        overral_machines = []
        for type in overral_machines_dict:
            machineOverral = OverralMachine()
            machineOverral.type = type
            machineOverral.quantity = overral_machines_dict[type][0]
            machineOverral.noload = overral_machines_dict[type][1]
            machineOverral.underload = overral_machines_dict[type][2]
            machineOverral.offtime = overral_machines_dict[type][3]
            overral_machines.append(machineOverral)

        msg = StateMachinesStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.machines_quantity = self.machine_info['quantity']
        msg.types_quantity = len(overral_machines_dict)
        msg.id_machines = self.machine_info['idMachines']
        msg.state_machines = state_machines
        msg.overral_machines = overral_machines
        self.pub_state_machine.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    plc_service = PlcService()
    rclpy.spin(plc_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plc_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

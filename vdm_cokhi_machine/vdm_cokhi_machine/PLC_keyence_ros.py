import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import datetime

from std_msgs.msg import Empty
from vdm_cokhi_machine_msgs.msg import MachineState, MachineStateArray
from vdm_cokhi_machine_msgs.srv import ResetMachinePLC


class PlcService(Node):
    def __init__(self):
        super().__init__('PLC_keyence_ros')
        # Params:
        # Cấu hình các thông số quan trọng:
        self.declare_parameter('PLC_model','KV-5500')
        self.declare_parameter('PLC_IP_address','192.168.1.1')
        self.declare_parameter('PLC_Port_address',8501)
        self.declare_parameter('maximum_dates',365)
        # self.declare_parameter('reset_service','/reset_machine_plc_kv')


        self.IP_addres_PLC = self.get_parameter('PLC_IP_address').value
        self.port_addres_PLC = self.get_parameter('PLC_Port_address').value
        self.maximumDates = self.get_parameter('maximum_dates').value
        self.plc_model = self.get_parameter('PLC_model').value
        self.reset_service_name = "/reset_machine_" + self.plc_model


        self.get_logger().info(f"PLC model: {self.plc_model}")
        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)
        # time.sleep(1.0)

        # Database path:
        # self.database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'    
        self.database_path = '/home/raspberry/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.conn = sqlite3.connect(self.database_path)
        self.cur = self.conn.cursor()
        self.machines_info = self.get_machines_inform_db()

        # Ros Services:
        self.resetMachine_srv = self.create_service(ResetMachinePLC, self.reset_service_name, self.reset_machine_cb)

        # Ros pub, sub:
        # Publishers:
        self.pub_state_machine = self.create_publisher(MachineStateArray, '/state_machine_plc', 10)

        # Subcribers:
        self.sub_update_database = self.create_subscription(Empty,'/update_database',self.update_machineInfo,10)


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
        # self.reset_machine_bit = ['MR',1000,'.U',1]
        # self.max_bit_reset = 15
        # self.reset_change_bit = ['MR',1100, '.U',1]
        self.reset_machine_res = ['DM',0,'.U',1]
        # self.reset_separate = 0

        ## Realtime data:
        self.dataMachine_length = 4
        self.separateMachine = 6
        if len(self.machines_info['PLC_address']) > 0:
            self.isPLCRun = True
            self.dataMachines_res = ['DM',1014,'.U',(self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
        else:
            self.isPLCRun = False
            self.dataMachines_res = ['DM',1014,'.U',1]
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

        timer_period = 0.5
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

    # Lấy dữ liệu tên của tất cả các máy từ database
    def get_machines_inform_db(self):
        try:
            self.cur.execute("SELECT * from " + self.tableName)
            rows = self.cur.fetchall()
            sorted_rows = sorted(rows, key=lambda x: x[4])
            # num = 0
            result = {
                'quantity': 0,
                'idMachines': [],
                'machineName': [],
                'machineType': [],
                'PLC_address': [],
            }
            for row in sorted_rows:
                if row[3] == self.plc_model:
                    result['quantity'] += 1
                    result['idMachines'].append(row[0])
                    result['machineName'].append(row[1])
                    result['machineType'].append(row[2])
                    result['PLC_address'].append(row[4])

            return result
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


    def reset_machine_cb(self, request: ResetMachinePLC.Request, response: ResetMachinePLC.Response):
        if not self.isPLCRun:
            response.success = False
            response.status = self.status['resetErr']
            return response
        self.get_logger().info(f'Reset machine: {request.name}, '
                            f'PLC model: {self.plc_model}, PLC address: {request.plc_address}')
        # self.get_logger().info(f'Receiv req machine: {request.name}')

        if self.check_password(request.password):
            if (self.write_device(self.reset_machine_res[0],
                                self.reset_machine_res[1],
                                self.reset_machine_res[2],
                                self.reset_machine_res[3],
                                [request.plc_address]) and
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
                        response.success = False
                        response.status = self.status['resetErr']
                        return response
                    a += 1

                response.success = resetOK
            
            else:
                response.success = False
                response.status = self.status['socketErr']
        else:
            response.success = False
            response.status = self.status['passErr']

        return response

    def check_password(self, passwordCheck):
        if passwordCheck == self.password:
            return True
        return False

    def update_machineInfo(self, msg: Empty):
        self.machines_info = self.get_machines_inform_db()
        if len(self.machines_info['PLC_address']) > 0:
            self.isPLCRun = True
            self.dataMachines_res = ['DM',self.dataMachines_res[1],'.U',
                                     (self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
        else:
            self.isPLCRun = False
        # self.dataMachines_res = ['DM',self.dataMachines_res[1],'.U',(self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
        return
    
    def handleSaveData(self,data):
        dataClock = self.read_device(self.clock_res[0],
                                     self.clock_res[1],
                                     self.clock_res[2],
                                     self.clock_res[3])

        date = datetime.datetime(2000 + dataClock[0],dataClock[1],dataClock[2],dataClock[3],dataClock[4],dataClock[5])

        for i in range(0,self.machines_info['quantity']):
            j = (self.machines_info['PLC_address'][i] - 1) * (self.dataMachine_length + self.separateMachine)
            name = self.machines_info['machineName'][i]
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
            # self.get_logger().warn(f'len data: {len(dataResp)}')
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
        if (not self.machines_info
            or not self.isPLCRun):
            return

        saveDataBit = self.read_device(self.save_data_bit[0],
                                      self.save_data_bit[1],
                                      self.save_data_bit[2],
                                      self.save_data_bit[3])[0]

        dataMachines = self.read_device(self.dataMachines_res[0],
                                        self.dataMachines_res[1],
                                        self.dataMachines_res[2],
                                        self.dataMachines_res[3])
        
        if saveDataBit:
            self.get_logger().info("Signal save data trigger!")
            self.handleSaveData(dataMachines)
            self.write_device(self.save_data_bit[0],
                              self.save_data_bit[1],
                              self.save_data_bit[2],
                              self.save_data_bit[3],[0])
        
        state_machines = []
        for i in range(0,self.machines_info['quantity']):
            j = (self.machines_info['PLC_address'][i] - 1) * (self.dataMachine_length + self.separateMachine)
            # self.get_logger().info(f"Thong so j: {j}")
            machineState = MachineState()
            machineState.name = self.machines_info['machineName'][i]
            machineState.type = self.machines_info['machineType'][i]
            machineState.signal_light = dataMachines[j + self.dataMachine_res_structure['signalLight'][1]]
            machineState.noload = dataMachines[j + self.dataMachine_res_structure['noload'][1]]
            machineState.underload = dataMachines[j + self.dataMachine_res_structure['underload'][1]]
            machineState.offtime = dataMachines[j + self.dataMachine_res_structure['offtime'][1]]
            # machineState.value_setting.min = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1]]
            # machineState.value_setting.max = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 1]
            # machineState.value_setting.current = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 2]
            # machineState.time_reachspeed = dataMachines[j + self.dataMachine_res_structure['timeReachSpeed'][1]]
            state_machines.append(machineState)

        msg = MachineStateArray()
        msg.state_machines = state_machines
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

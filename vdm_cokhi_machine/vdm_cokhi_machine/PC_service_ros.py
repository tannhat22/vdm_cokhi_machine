import rclpy
from rclpy.node import Node
import sqlite3
from collections import Counter

from vdm_cokhi_machine_msgs.msg import OverralMachine, MachineState, MachineStateArray, MachinesStateStamped
from vdm_cokhi_machine_msgs.srv import GetAllMachineName, GetMachineData, ResetMachine, ResetMachinePLC
from vdm_cokhi_machine_msgs.srv import CreateMachine, UpdateMachine, DeleteMachine


class State:
    def __init__(self, ID: int = 0, state: MachineState = None):
        self.ID = ID
        self.state = state

class PlcService(Node):
    def __init__(self):
        super().__init__('PC_service_ros')
        # Cấu hình các thông số quan trọng:
        self.maximumDates = 365
        self.plc_keyence = 'KV-5500'
        self.plc_mitsu = 'FX3U'
        self.machines = {}

        # Database path:
        self.database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'  
        # self.database_path = '/home/raspberry/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.conn = sqlite3.connect(self.database_path)
        self.cur = self.conn.cursor()
        self.create_table_group_machines_db(self.tableName)
        self.machines_info = self.get_machines_inform_db()
        if self.machines_info:
            for i in range(self.machines_info['quantity']):
                self.machines[self.machines_info['machineName'][i]] = State(ID=self.machines_info['idMachines'][i])

        # Ros Services
        # Server:
        self.getAllMachineName_srv = self.create_service(GetAllMachineName, 'get_all_machine_name', self.get_all_machine_name_cb)
        self.getMachineData_srv = self.create_service(GetMachineData, 'get_machine_data', self.get_machine_data_cb)
        self.resetMachine_srv = self.create_service(ResetMachine, 'reset_machine', self.reset_machine_cb)
        self.createMachine_srv = self.create_service(CreateMachine, 'create_machine', self.create_machine_cb)
        self.updateMachine_srv = self.create_service(UpdateMachine, 'update_machine', self.update_machine_cb)
        self.deleteMachine_srv = self.create_service(DeleteMachine, 'delete_machine', self.delete_machine_cb)

        # Client:
        self.keyence_client = self.create_client(ResetMachinePLC, 'reset_machine_plc_kv')
        self.mitsu_client = self.create_client(ResetMachinePLC, 'reset_machine_plc_fx')
        # while not self.keyence_client.wait_for_service(timeout_sec=10.0):
        #     self.get_logger().info('Keyence service not available, waiting again...')
        # while not self.mitsu_client.wait_for_service(timeout_sec=10.0):
        #     self.get_logger().info('FX3U service not available, waiting again...')


        # Ros pub, sub:
        # Publishers:
        self.pub_state_machines = self.create_publisher(MachinesStateStamped, '/state_machines', 10)

        # Subcribers:
        self.sub_state_machine = self.create_subscription(MachineStateArray, '/state_machine_plc', self.state_machine_cb, 10)

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
        self.password = '10064'

        # Status for response services:
        self.status = {
            'success': 'Thành công',
            'notFound': 'Dữ liệu không tìm thấy!',
            'nameInvalid': 'Tên máy không hợp lệ!',
            'nameInuse': 'Tên máy đã được sử dụng!',
            'addressInuse': 'Địa chỉ PLC này đã được sử dụng!',
            'typeInvalid': 'Loại máy không hợp lệ!',
            'PLCInvalid': 'Model PLC không hợp lệ!',
            'passErr': 'Sai mật khẩu!',
            'resetErr': 'Reset máy lỗi!',
            'dbErr': 'Lỗi cơ sở dữ liệu!',
            'socketErr': 'Lỗi kết nối Raspberry và PLC!',
            'fatalErr': 'Something is wrong, please check all system!'
        }

        # timer_period = 0.2
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("is running!!!!!!!!!!")


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
                'machineType': [],
                'PLC_model': [],
                'PLC_address': [],
            }
            for row in rows:
                result['quantity'] += 1
                result['idMachines'].append(row[0])
                result['machineName'].append(row[1])
                result['machineType'].append(row[2])
                result['PLC_model'].append(row[3])
                result['PLC_address'].append(row[4])
                # num += 1
            # result['quantity'] = num
            # self.types_info = self.array_type_to_dict(result['machineType'])
            return result
        except Exception as e:
            print(e)
            return False
    
    # Lấy tên của machine trong bảng dựa vào thông số đầu vào là ID 
    def get_machine_name_db(self, id):
        try:
            self.cur.execute("SELECT NAME FROM " + self.tableName + " WHERE ID = ?",(id,))
            machineName = self.cur.fetchone()[0]
            return machineName
        except Exception as e:
            print(e)
            self.get_logger().info(f'Exeption: {e}')
            return False
    
    # Lấy thông tin tên, loại, plc, địa chỉ dựa vào thông số đầu vào là ID
    def get_machine_inform_db(self, id):
        try:
            self.cur.execute("SELECT * FROM " + self.tableName + " WHERE ID = ?",(id,))
            row = self.cur.fetchone()
            return {
                "ID:", row[0],
                "name:", row[1],
                "type:", row[2],
                "plc:", row[3],
                "address:", row[4],
            }
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
    def add_machine_db(self, name, type, plc, address):
        try:
            self.cur.execute("INSERT INTO " + self.tableName + " (NAME, TYPE, PLC, ADDRESS) VALUES (?, ?, ?, ?)", (name, type, plc, address))
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
    def update_machine_db(self, id, newName, newType, newPLC, newAddress):
        try:
            machineName = self.get_machine_name_db(id)
            if self.update_table_name(machineName, newName):
                self.cur.execute("UPDATE " + self.tableName + " SET NAME = ?, TYPE = ?, PLC = ?, ADDRESS = ? WHERE ID = ?",
                                 (newName, newType, newPLC, newAddress, id))
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
    def checkNameIsExists(self, name):
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
        
    # Kiểm tra địa chỉ machine trong cùng một PLC có tồn tại trong database:
    def checkAddressIsExists(self, plc, address):
        try:
            self.cur.execute("SELECT ID FROM " + self.tableName + " WHERE PLC = ? AND ADDRESS = ?",(plc, address))
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
            # self.machine_info = self.get_machines_inform_db()
            machine_info = self.get_machines_inform_db()
            if not machine_info:
                response.success = False
                response.status = self.status['dbErr']
                return response

            response.success = True
            response.status = self.status['success']
            response.machines_quantity = machine_info['quantity']
            response.id_machines = machine_info['idMachines']
            response.machines_name = machine_info['machineName']
            response.machines_type = machine_info['machineType']
            response.plc_model = machine_info['PLC_model']
            response.plc_address = machine_info['PLC_address']
            return response

    # Lấy dữ liệu của một máy dựa trên yêu cầu ID và số ngày cần lấy
    def get_machine_data_cb(self, request: GetMachineData.Request, response: GetMachineData.Response):
        # if not self.machine_info:
        #     response.success = False
        #     response.status = self.status['dbErr']
        #     return response
        
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
        machine = self.get_machine_inform_db(request.id_machine)
        self.get_logger().info(f'Reset machine: {machine["name"]}, '
                               f'PLC model: {machine["plc"]}, PLC address: {machine["address"]}')

        if self.check_password(request.password):
            result = self.send_request(machine,request.password)
            if result.success:
                response.success = True
            else:
                response.success = False
                response.status = result.status
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

            elif request.plc_model == '':
                response.success = False
                response.status = self.status['PLCInvalid']

            elif self.checkNameIsExists(request.name.upper()):
                response.success = False
                response.status = self.status['nameInuse']

            elif self.checkAddressIsExists(request.plc_model, request.plc_address):
                response.success = False
                response.status = self.status['addressInuse']    

            elif not self.add_machine_db(request.name.upper(), request.type,
                                         request.plc_model, request.plc_address):
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

            elif request.new_plc_model == '':
                response.success = False
                response.status = self.status['PLCInvalid']

            elif self.checkNameIsExists(request.new_name.upper()):
                response.success = False
                response.status = self.status['nameInuse']

            elif self.checkAddressIsExists(request.new_plc_model, request.new_plc_address):
                response.success = False
                response.status = self.status['addressInuse']    

            elif not self.update_machine_db(request.id_machine, request.new_name.upper(),
                                            request.new_type, request.new_plc_model, request.new_plc_address):
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
        self.machines_info = self.get_machines_inform_db()
        if self.machines_info:
            for i in range(self.machines_info['quantity']):
                if self.machines_info['machineName'][i] not in self.machines:
                    self.machines[self.machines_info['machineName'][i]] = State(ID=self.machines_info['idMachines'][i])
        return
    
    def send_request(self, machine, password):
        reqPLC = ResetMachinePLC.Request()
        reqPLC.name = machine['name']
        reqPLC.plc_address = machine['address']
        reqPLC.password = password
        if machine['plc'] == self.plc_keyence:
            self.future = self.keyence_client.call_async(reqPLC)
        elif machine['plc'] == self.plc_mitsu:
            self.future = self.mitsu_client.call_async(reqPLC)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

    def state_machine_cb(self, msg: MachineStateArray):
        for state in msg.state_machines:
            if state.name in self.machines:
                self.machines[state.name].state = state
            else:
                self.get_logger().warn(f'Detect machine is not sync: {state.name}')

        self.timer_callback()

    def timer_callback(self):
        # if not self.machine_info: return
        state_machines = []
        machineID = []
        overral_machines_dict = {}
        for machine in self.machines:
            machineID.append(self.machines[machine].ID)
            machineState = self.machines[machine].state
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

        msg = MachinesStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.machines_quantity = len(self.machines)
        msg.types_quantity = len(overral_machines_dict)
        msg.id_machines = machineID
        msg.state_machines = state_machines
        msg.overral_machines = overral_machines
        self.pub_state_machines.publish(msg)


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

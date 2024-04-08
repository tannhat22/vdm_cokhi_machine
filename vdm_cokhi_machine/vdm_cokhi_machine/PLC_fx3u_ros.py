import math
import rclpy
from rclpy.node import Node
import sqlite3
import datetime
import os
from .mcprotocol.type1e import Type1E

from std_msgs.msg import Empty
from vdm_cokhi_machine_msgs.msg import MachineState, MachineStateArray
from vdm_cokhi_machine_msgs.srv import ResetMachinePLC

class State:
    def __init__(self, ID: int = 0, signalLight: int = 4):
        self.ID = ID
        self.signalLight = signalLight

class PlcService(Node):
    def __init__(self):
        super().__init__('PLC_mitsu_ros')
        # Params
        # Cấu hình các thông số quan trọng:
        self.declare_parameter('PLC_model','FX3U')
        self.declare_parameter('PLC_IP_address','192.168.1.250')
        self.declare_parameter('PLC_Port_address',8000)
        self.declare_parameter('maximum_dates',365)
        # self.declare_parameter('reset_service','/reset_machine_plc_fx')

        self.IP_addres_PLC = self.get_parameter('PLC_IP_address').value
        self.port_addres_PLC = self.get_parameter('PLC_Port_address').value
        self.maximumDates = self.get_parameter('maximum_dates').value
        self.plc_model = self.get_parameter('PLC_model').value
        # self.reset_service_name = self.get_parameter('reset_service').value
        self.reset_service_name = "/reset_machine_" + self.plc_model

        
        self.get_logger().info(f"PLC model: {self.plc_model}")
        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")


        self.pyPLC = Type1E('F')
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        # Database path:
        self.database_path = os.path.join(os.path.expanduser("~"),
                             'ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db') 
        # self.database_path = '/home/raspberry/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.conn = sqlite3.connect(self.database_path)
        self.cur = self.conn.cursor()
        self.machines_info = self.get_machines_inform_db()
        self.machines = {}
        if self.machines_info:
            for i in range(self.machines_info['quantity']):
                self.machines[self.machines_info['machineName'][i]] = State(ID=self.machines_info['idMachines'][i])

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
            # year(0-99):       D700
            # month(1-12):      D701
            # day(1-31):        D702
            # hour(0-23):       D703
            # minute(0-59):     D704
            # second(0-59):     D705
            # day-of-week(0-6): D706 (0: sunday, 1: monday, 2: tuesday, 3: wednesday, 4: thursday, 5: friday, 6: saturday)    
        self.clock_res = ['D700',7]

        self.save_data_bit = ['M0',1]

        self.password = '10064'
        self.password_write_res = ['D500',1]

        self.reset_bit = ['M2000',1]
        # self.reset_machine_bit = ['MR',1000,'.U',1]
        # self.max_bit_reset = 15
        # self.reset_change_bit = ['MR',1100, '.U',1]
        self.reset_machine_res = ['D510',1]
        # self.reset_separate = 0

        ## Realtime data:
        self.dataMachine_length = 4
        self.separateMachine = 6
        if len(self.machines_info['PLC_address']) > 0:
            self.isPLCRun = True
            self.dataMachines_res = ['D1500',(self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
        else:
            self.isPLCRun = False
            self.dataMachines_res = ['D1500',1]
        # self.dataMachines_res = ['D1500',(self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
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
        # Thời gian của các ca:
        self.dayShift = [datetime.time(hour=6, minute=0, second=0),
                         datetime.time(hour=17, minute=59, second=59)]
        self.nightShift = [datetime.time(hour=18, minute=0, second=0),
                         datetime.time(hour=5, minute=59, second=59)]
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("is running!!!!!!!!!!")


    #"Connect to PLC:"
    # def socket_connect(self,host, port):
    #     try:
    #         self.soc.connect((host, port))
    #         self.get_logger().info("is connected to PLC success")
    #         return True
    #     except OSError:
    #         self.get_logger().info("can't connect to PLC, will auto reconneting after 2s")
    #         time.sleep(2)
    #         return False

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
    def add_history_data_db(self, tableName, date, shift, noLoad, underLoad, offtime):
        try:
            # self.cur.execute("SELECT * from " + tableName)
            # totalDates = len(self.cur.fetchall())
            self.cur.execute("SELECT COUNT(*) from " + tableName)
            totalDates = self.cur.fetchone()[0]

            if totalDates >= self.maximumDates:
                self.cur.execute("DELETE FROM " + tableName + " WHERE ROWID = (SELECT MIN(ROWID) FROM " + tableName + ")")
                # self.cur.execute("DELETE FROM " + tableName + " WHERE ID = 1")
                # self.cur.execute("CREATE TABLE momenttable AS SELECT * FROM " + tableName)
                # self.cur.execute("DELETE FROM " + tableName)
                # self.cur.execute("DELETE FROM sqlite_sequence WHERE name='" + tableName + "'")
                # self.cur.execute("INSERT INTO " + tableName + " (DATE, SHIFT, NOLOAD, UNDERLOAD, OFFTIME) SELECT DATE, SHIFT, NOLOAD, UNDERLOAD, OFFTIME FROM momenttable")
                # self.delete_table("momenttable")
                self.conn.commit()
            
            self.cur.execute("INSERT INTO " + tableName + " (DATE, SHIFT, NOLOAD, UNDERLOAD, OFFTIME) VALUES (?, ?, ?, ?)", (date, shift, noLoad, underLoad, offtime))
            self.conn.commit()
            return True
        except Exception as e:
            print(e)
            return False

    # Thêm dữ liệu logs mới vào bảng database:
    def add_logs_data_db(self, tableName, clock: datetime.datetime, state):
        stateDes = ""
        if state == MachineState.MACHINE_OFF:
            stateDes = "Tắt máy"
        elif state == MachineState.MACHINE_NOLOAD:
            stateDes = "Chạy không tải"
        elif state == MachineState.MACHINE_UNDERLOAD:
            stateDes = "Chạy có tải"
        elif state == MachineState.MACHINE_OVERLOAD:
            stateDes = "Quá tải"

        try:
            self.cur.execute("SELECT COUNT(*) from " + tableName)
            totalRow = self.cur.fetchone()[0]
            if totalRow > 30000:
                self.cur.execute("DELETE FROM " + tableName + " WHERE ROWID = (SELECT MIN(ROWID) FROM " + tableName + ")")
                self.conn.commit()
            
            date = clock.strftime("%d/%m/%Y")
            time = clock.strftime("%H:%M:%S")
            self.cur.execute("INSERT INTO " + tableName + " (DATE, TIME, STATE) VALUES (?, ?, ?)", (date, time, stateDes))
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
            self.pyPLC.batchwrite_wordunits(self.reset_machine_res[0],
                                                [request.plc_address])
            self.pyPLC.batchwrite_wordunits(self.password_write_res[0],
                                                [int(request.password)])
            self.pyPLC.batchwrite_bitunits(self.reset_bit[0],[1])
            a = 0
            resetOK = False
            while not resetOK:
                resetOK = not self.pyPLC.batchread_bitunits(self.reset_bit[0],
                                                            self.reset_bit[1])[0]
                if a >= 1200:
                    response.success = False
                    response.status = self.status['resetErr']
                    return response
                a += 1

            response.success = resetOK

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
            self.dataMachines_res = [self.dataMachines_res[0],
                                     (self.dataMachine_length + self.separateMachine) * self.machines_info['PLC_address'][-1]]
        else:
            self.isPLCRun = False
        return
    
    def handleSaveData(self, data, shift, clockRes):
        if shift == MachineStateArray.DAY_SHIFT:
            date = datetime.datetime(2000 + clockRes[0],clockRes[1],clockRes[2],6,0,0)
            shiftDes = "CN"
        elif shift == MachineStateArray.NIGHT_SHIFT:
            date = datetime.datetime(2000 + clockRes[0],clockRes[1],clockRes[2],18,0,0) - datetime.timedelta(days=1)
            shiftDes = "CD"

        for i in range(0,self.machines_info['quantity']):
            j = (self.machines_info['PLC_address'][i] - 1) * (self.dataMachine_length + self.separateMachine)
            name = self.machines_info['machineName'][i]
            noload = data[j + self.dataMachine_res_structure['noload'][1]]
            underload = data[j + self.dataMachine_res_structure['underload'][1]]
            offtime = data[j + self.dataMachine_res_structure['offtime'][1]]
            if not self.add_history_data_db(name,date,shiftDes,noload,underload,offtime):
                self.get_logger().info("ERROR: Save data of day error!!!")
                return False
        
        self.get_logger().info("Save data of day success!")
        return True
    
    
    def timer_callback(self):
        if (not self.machines_info
            or not self.isPLCRun):
            return

        saveDataBit = self.pyPLC.batchread_bitunits(self.save_data_bit[0],
                                                    self.save_data_bit[1])[0]
        
        dataClock = self.pyPLC.batchread_wordunits(self.clock_res[0],
                                                   self.clock_res[1])

        realTimePLc = datetime.datetime(2000 + dataClock[0],dataClock[1],dataClock[2],
                                             dataClock[3],dataClock[4],dataClock[5])
        

        # CN-06:00:00-17:59:59, CD-18:00:00:-05:59:59
        if self.dayShift[0] <= realTimePLc.time() <= self.dayShift[1]:
            shiftNow = MachineStateArray.DAY_SHIFT
        else:
            shiftNow = MachineStateArray.NIGHT_SHIFT

        dataMachines = []
        count = math.ceil(self.dataMachines_res[1] / 64)
        remainRes = self.dataMachines_res[1]
        for i in range(count):
            headDevice = f"{self.dataMachines_res[0][0]}{int(self.dataMachines_res[0][1:]) + 64*i}"
            if remainRes >= 64:
                dataMachines += self.pyPLC.batchread_wordunits(headDevice, 64)
            else:
                dataMachines += self.pyPLC.batchread_wordunits(headDevice, remainRes)
            remainRes -= 64
        
        if saveDataBit:
            self.get_logger().info("Signal save data trigger!")
            self.handleSaveData(data=dataMachines, shift=shiftNow, clockRes=dataClock)
            self.pyPLC.batchwrite_bitunits(self.save_data_bit[0],[0])
        
        state_machines = []
        for i in range(0,self.machines_info['quantity']):
            j = (self.machines_info['PLC_address'][i] - 1) * (self.dataMachine_length + self.separateMachine)
            machineState = MachineState()
            machineState.name = self.machines_info['machineName'][i]
            machineState.type = self.machines_info['machineType'][i]
            machineState.signal_light = dataMachines[j + self.dataMachine_res_structure['signalLight'][1]]
            machineState.noload = dataMachines[j + self.dataMachine_res_structure['noload'][1]]
            machineState.underload = dataMachines[j + self.dataMachine_res_structure['underload'][1]]
            machineState.offtime = dataMachines[j + self.dataMachine_res_structure['offtime'][1]]
            state_machines.append(machineState)

            if machineState.signal_light != self.machines[machineState.name].signalLight:
                self.machines[machineState.name].signalLight = machineState.signal_light
                self.get_logger().info(f"{machineState.name}: has new logging!!!")
                self.add_logs_data_db(machineState.name + '_logs',
                                      realTimePLc, machineState.signal_light)

        msg = MachineStateArray()
        msg.shift = shiftNow
        msg.state_machines = state_machines
        self.pub_state_machine.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    plc_service = PlcService()
    rclpy.spin(plc_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plc_service.conn.close()
    plc_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import math

from std_msgs.msg import Bool
from vdm_cokhi_machine_msgs.msg import StateMachine, StateMachinesStamped

class PcReadPlc(Node):
    def __init__(self):
        super().__init__('PC_read_PLC')
        self.IP_addres_PLC = '192.168.0.10'
        self.port_addres_PLC = 8501

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)
        time.sleep(1.0)

        # Database path:
        self.database_path = '/home/tannhat/ros2_ws/src/vdm_cokhi_machine/vdm_cokhi_machine/database/machine.db'
        self.tableName = 'MACHINES'
        self.machine_info = self.get_all_machine_name_db()

        # Ros pub, sub:
        self.pub_state_machine = self.create_publisher(StateMachinesStamped, '/state_machines', 10)

        self.sub_update_machines_table = self.create_subscription(Bool, '/update_machines_table',self.update_machines_table_cb, 10)

        # self.bool_true = Bool()
        # self.bool_true.data = True

        # self.bool_false = Bool()
        # self.bool_false.data = False
        
        self.dataMachine_length = 7
        self.dataMachines_res = ['DM',1000,'.U',self.dataMachine_length * self.machine_info['quantity']]
        self.separateMachine = 10
        self.dataMachine_res_structure = {
            'signalLight': [1,0],
            'noload': [1,1],
            'underload': [1,2],
            'valueSetting': [3,3],
            'timeReachSpeed': [1,6],
        }

        self.dataMachineHistory_length = 151
        self.dataMachineHistory_res = ['EM',1000,'.U',self.dataMachineHistory_length]
        self.separateMachineHistory = 10
        self.dataMachineHistory_res_structure = {
            'totalDays': [1,0],
            'noloadHistory': [30,1],
            'underloadHistory': [30,31],
            'years': [30,61],
            'months': [30,91],
            'days': [30,121],
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

    def get_all_machine_name_db(self):
        try:
            conn = sqlite3.connect(self.database_path)
            cur = conn.cursor()
            cur.execute("SELECT * from " + self.tableName)
            num = 0
            result = {
                'quantity': 0,
                'machineName': [],
                'idMachines': []
            }
            for row in cur:
                result['idMachines'].append(row[0])
                result['machineName'].append(row[1]) 
                num += 1
            result['quantity'] = num
        
            conn.close()
            return result
        except Exception as e:
            print(e)
            return False

    def update_machines_table_cb(self, msg: Bool):
        if msg.data:
            self.machine_info = self.get_all_machine_name_db()
        return


    #"Read bit. Ex: MR10 => deviceType: 'M', deviceNo: 100"
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    # def read_bit(self,deviceType: str = 'R', deviceNo: str = '0', format: str = '', length: int = 1):
    #     if length > 1:
    #         device = 'RDS' + ' ' + deviceType + deviceNo + format + ' ' + str(length) + '\x0D'
    #     else:
    #         device = 'RD' + ' ' + deviceType + deviceNo + format + '\x0D' 
    #     dataformat = device.encode()
    #     self.soc.sendall(dataformat)
    #     dataRecv = self.soc.recv(1024)
    #     dataRecvDec = dataRecv.decode()
    #     dataBit = dataRecvDec.split(' ')
    #     dataBit[len(dataBit)-1] = dataBit[len(dataBit)-1][0]
    #     for i in range (0,len(dataBit)):
    #         dataBit[i] = int(dataBit[i])
    #     return dataBit


    #"Read register:"
    # deviceType: 'D', deviceNo: 100,
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    # def read_register(self,deviceType: str = 'DM', deviceNo: str = '0', format: str = '', length: int = 1):
    #     if length > 1:
    #         device = 'RDS' + ' ' + deviceType + deviceNo + format + ' ' + str(length) + '\x0D'
    #     else:
    #         device = 'RD' + ' ' + deviceType + deviceNo + format + '\x0D' 
    #     dataformat = device.encode()
    #     self.soc.sendall(dataformat)
    #     dataRecv = self.soc.recv(1024)
    #     dataRecvDec = dataRecv.decode()
    #     dataReg = dataRecvDec.split(' ')
    #     dataReg[len(dataReg)-1] = dataReg[len(dataReg)-1][:-2]
    #     for i in range (0,len(dataReg)):
    #         dataReg[i] = int(dataReg[i])
    #     return dataReg
    
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
    
    
    def timer_callback(self):
        if not self.machine_info: return
        
        dataMachines = self.read_device(self.dataMachines_res[0],
                                        self.dataMachines_res[1],
                                        self.dataMachines_res[2],
                                        self.dataMachines_res[3])
        
        state_machines = []
        for i in range(0,self.machine_info['quantity']):
            j = i * (self.dataMachine_length + self.separateMachine)
            machineState = StateMachine()
            machineState.name = self.machine_info['machineName'][i]
            machineState.signal_light = dataMachines[j + self.dataMachine_res_structure['signalLight'][1]]
            machineState.noload = (dataMachines[j + self.dataMachine_res_structure['noload'][1]])
            machineState.underload = (dataMachines[j + self.dataMachine_res_structure['underload'][1]])
            machineState.value_setting.min = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1]]
            machineState.value_setting.max = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 1]
            machineState.value_setting.current = dataMachines[j + self.dataMachine_res_structure['valueSetting'][1] + 2]
            machineState.time_reachspeed = dataMachines[j + self.dataMachine_res_structure['timeReachSpeed'][1]]
            state_machines.append(machineState)
        
        msg = StateMachinesStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.machines_quantity = self.machine_info['quantity']
        msg.id_machines = self.machine_info['idMachines']
        msg.state_machines = state_machines
        self.pub_state_machine.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pc_read_plc = PcReadPlc()
    rclpy.spin(pc_read_plc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc_read_plc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import time
import socket
import rclpy
from rclpy.node import Node
import sqlite3
import os

# from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
# from std_msgs.msg import Bool, String
from vdm_cokhi_machine_msgs.msg import StateMachineStamped

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
        self.tableName = 'MACHINE'
        self.create_table_db(self.tableName)
        self.machine_info = self.get_machine_name()

        # Ros pub, sub:
        self.pub_state_machine = self.create_publisher(StateMachineStamped, '/state_machine', 10)

        # self.bool_true = Bool()
        # self.bool_true.data = True

        # self.bool_false = Bool()
        # self.bool_false.data = False
        
        self.signalLight_bit_start = '1'
        self.signalLight_bit_length = self.machine_info['quantity'] * 3
        self.time_res_start = '1'
        self.time_res_length = self.machine_info['quantity'] * 2

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

    def create_table_db(self, tableName):
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
    
    def get_machine_name(self):
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
        for i in range (0,len(dataBit)):
            dataBit[i] = int(dataBit[i])
        return dataBit


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
        for i in range (0,len(dataReg)):
            dataReg[i] = int(dataReg[i])
        return dataReg
    
    #Decode bytes:
    def decode_bytes(self,bytes_data):
        byte_val = bytes(bytes_data)
        data_formDEC = int.from_bytes(byte_val,'big',signed=True)
        return data_formDEC
    
    #M200-206
    def timer_callback(self):
        data_signalLight = self.read_bit('MR',self.signalLight_bit_start,'',self.signalLight_bit_length)
        data_time = self.read_register('DM',self.time_res_start,'',self.time_res_length)


        msg = StateMachineStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state_machine.number_machine = self.machine_info['quantity']
        msg.state_machine.machine_name = self.machine_info['machineName']
        msg.state_machine.time_noload = data_time[:self.machine_info['quantity']]
        msg.state_machine.time_load = data_time[self.machine_info['quantity']:]
        msg.state_machine.signal_light = data_signalLight
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

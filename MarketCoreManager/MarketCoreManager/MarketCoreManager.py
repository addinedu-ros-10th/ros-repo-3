import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from pinky_msgs.msg import StateControl
from pinky_msgs.msg import RobotState

from .tcp_socket import TCPSocket
from .config import *
import time
import threading
import queue
import struct

class MarketCoreManager(Node):
    def __init__(self):
        super().__init__('MarketCoreManager')

        self.ongui_recv_queue = queue.Queue(maxsize=1)
        self.admingui_recv_queue = queue.Queue(maxsize=1)

        self.ongui_socket = TCPSocket(MARKETCOREMANAGER_IP, ONGUI_PORT)
        self.admingui_socket = TCPSocket(MARKETCOREMANAGER_IP, ADMINGUI_PORT)
        self.thread_list = []

        self.ongui_socket_start()
        self.admingui_socket_start()
        ongui_callback_thread = threading.Thread(target=self.ongui_recv_callback, daemon=True)
        admin_callback_thread = threading.Thread(target=self.admingui_recv_callback, daemon=True)
        self.thread_list.append(ongui_callback_thread)
        self.thread_list.append(admin_callback_thread)
        
        self.robot_status_subscriber = self.create_subscription(
            RobotState,
            '/pinky31/robot_state',
            self.pinky31_callback,
            10
        )
        self.robot_status_subscriber = self.create_subscription(
            RobotState,
            '/pinky32/robot_state',
            self.pinky32_callback,
            10
        )
        self.robot_state_control_publisher = self.create_publisher(
            StateControl,
            '/state_control',
            10
        )
        
        for t in self.thread_list:
            t.start()

        self.get_logger().info(f"INIT COMPLETE")

    def ongui_socket_start(self) -> None:
        self.ongui_socket.init_socket()

        connection_thread = threading.Thread(target=self.ongui_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)
        
        recv_thread = threading.Thread(target=self.ongui_socket.recv_data, args=(self.ongui_recv_queue,), daemon=True)
        self.thread_list.append(recv_thread)

    def admingui_socket_start(self) -> None:
        self.admingui_socket.init_socket()

        connection_thread = threading.Thread(target=self.admingui_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.admingui_socket.recv_data, args=(self.admingui_recv_queue,), daemon=True)
        self.thread_list.append(recv_thread)

    def ongui_recv_callback(self):
        while True:
            if self.ongui_recv_queue.full():
                data = self.ongui_recv_queue.get_nowait()
                self.ongui_data_check(data)
            else:
                pass
            time.sleep(0.1)

    def ongui_data_check(self, data):
        if (int(data[0:4]) != 1):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM ONGUI")
            return
        
        if (int(data[8:12]) == ONGUI_REQ1_ID):
            unpacked_data = struct.unpack(ONGUI_REQ1, data[12:])
            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV1, ONGUI_ID, 1, ONGUI_RECV1_ID, True)
            self.ongui_socket.send_data(send_data)
            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV2, ONGUI_ID, 65, ONGUI_RECV2_ID, 1)
            self.ongui_socket.send_data(send_data)

        elif (int(data[8:12]) == ONGUI_REQ2_ID):
            unpacked_data = struct.unpack(ONGUI_REQ2, data[12:])
            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV3, ONGUI_ID,  1, ONGUI_RECV3_ID, True)
            self.ongui_socket.send_data(send_data)

        else:
            self.get_logger().info(f"WRONG DATA FUNCTION CODE FROM ONGUI")
         
    def admingui_recv_callback(self):
        while True:
            if self.admingui_recv_queue.full():
                data = self.admingui_recv_queue.get_nowait()
                self.admingui_data_check(data)
            else:
                pass
            time.sleep(0.1)
    
    def admingui_data_check(self, data):
        if (int(data[0:4]) != 1):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM ADMINGUI")
            return
                
        if (int(data[8:12]) == ADMINGUI_REQ1_ID):
            unpacked_data = struct.unpack(ONGUI_REQ1, data[12:])
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV1, ADMINGUI_ID, 1, ADMINGUI_RECV1_ID, True)
            self.admingui_socket.send_data(send_data)
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV2, ADMINGUI_ID, 65, ADMINGUI_RECV2_ID, 1)
            self.admingui_socket.send_data(send_data)

        elif (int(data[8:12]) == ADMINGUI_REQ2_ID):
            unpacked_data = struct.unpack(ONGUI_REQ2, data[12:])
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV3, ADMINGUI_ID,  1, ADMINGUI_RECV3_ID, True)
            self.admingui_socket.send_data(send_data)

        else:
            self.get_logger().info(f"WRONG DATA FUNCTION CODE FROM ONGUI")
         
    def pinky31_callback(self, msg):
        cart_id = msg.cid
        user_id = msg.id
        state_id = msg.stid
        battery = msg.bat
        posx = msg.posx
        posy = msg.posy

        send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV3, ADMINGUI_ID, 32, ADMINGUI_RECV3_ID, *[cart_id, user_id, state_id, battery, posx, posy])
        self.admingui_socket.send_data(send_data)

    def pinky32_callback(self, msg):
        cart_id = msg.cid
        user_id = msg.id
        state_id = msg.stid
        battery = msg.bat
        posx = msg.posx
        posy = msg.posy

        send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV3, ADMINGUI_ID, 32, ADMINGUI_RECV3_ID, *[cart_id, user_id, state_id, battery, posx, posy])
        self.admingui_socket.send_data(send_data)

    def __del__(self):
        for t in self.thread_list:
            t.join()
        

def main(args=None):
    rclpy.init(args=args)
    node = MarketCoreManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
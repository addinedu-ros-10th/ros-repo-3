import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from pinky_msgs.msg import StateControl
from pinky_msgs.msg import RobotState

from .tcp_socket import TCPSocket
from .config import *
from .state_machine import *

import time
import threading
import queue
import struct


class CartStateManager(Node):
    def __init__(self):
        super().__init__('CartStateManager')

        self.state_machine = StateMachine(STATE_TABLE, 0)
        self.user_id = -1

        self.perception_manager_queue = queue.Queue(maxsize=1)
        self.interface_manager_queue = queue.Queue(maxsize=1)

        self.perception_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, PERCEPTIONMANAGER_PORT)
        self.interface_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, INTERFACEMANAGER_PORT)
        
        self.thread_list = []
        self.perception_manager_socket_start()
        self.interface_manager_socket_start()

        for t in self.thread_list:
            t.start()

    def perception_manager_socket_start(self):
        self.perception_manager_socket.init_socket()

        connection_thread = threading.Thread(target=self.perception_manager_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.perception_manager_socket.recv_data, args=(self.perception_manager_queue,), daemon=True)
        self.thread_list.append(recv_thread)

    def interface_manager_socket_start(self):
        self.interface_manager_socket.init_socket()

        connection_thread = threading.Thread(target=self.interface_manager_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.interface_manager_socket.recv_data, args=(self.interface_manager_queue,), daemon=True)
        self.thread_list.append(recv_thread)

        recv_callback_thread = threading.Thread(target=self.interface_manager_recv_callback, daemon=True)
        self.thread_list.append(recv_callback_thread)

    def perception_manager_recv_callback(self):
        while True:
            if self.perception_manager_queue.full():
                data = self.perception_manager_queue.get_nowait()
                print(data)
                self.perception_manager_data_check(data)
            else:
                pass
            time.sleep(0.1)

    def interface_manager_recv_callback(self):
        while True:
            if self.interface_manager_queue.full():
                data = self.interface_manager_queue.get_nowait()
                print(data)
                self.interface_manager_data_check(data)
            else:
                pass
            time.sleep(0.1)

    def perception_manager_data_check(self, data):
        if (struct.unpack("<i",data[0:4])[0] != CARTSTATEMANAGER_ID):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM PERCEPTIONMANAGER")
            return

    def interface_manager_data_check(self, data):
        if (struct.unpack("<i",data[0:4])[0] != CARTSTATEMANAGER_ID):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM INTERFACEMANAGER")
            return
        
        if (struct.unpack("<i",data[8:12])[0] == INTERFACEMANAGER_FUNCTION1_ID):
            self.user_id = struct.unpack(INTERFACEMANAGER_FUNCTION_1, data[12:])[0]
            print(self.user_id)
        
        elif (struct.unpack("<i",data[8:12])[0] == INTERFACEMANAGER_FUNCTION2_ID):
            data = struct.unpack(INTERFACEMANAGER_FUNCTION_2, data[12:])[0]
            if data[0] == 0:
                pass
            elif data[1] == 1:
                pass
            else:
                self.get_logger().info(f"WRONG COM CODE FROM INTERFACEMANAGER")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CartStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        



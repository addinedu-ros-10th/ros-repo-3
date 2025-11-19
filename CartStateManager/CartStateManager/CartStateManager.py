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
import time
import threading
import queue
import struct

class CartStateManager(Node):
    def __init__(self):
        super().__init__('CartStateManager')

        self.perception_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, PERCEPTIONMANAGER_PORT)
        self.interface_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, INTERFACEMANAGER_PORT)

        self.perception_manager_socket_start()
        self.interface_manager_socket_start()


    def perception_manager_socket_start():
        pass
    def interface_manager_socket_start():
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

        



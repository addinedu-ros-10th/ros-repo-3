import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from pinky_msgs.msg import RobotState, PoseOrder, HumanPos, ItemReq
from pinky_msgs.srv import Usercheck

from .tcp_socket import TCPSocket
from .config import *
from .state_machine import *

import math
import time
import threading
import queue
import struct
import os

class CartStateManager(Node):
    def __init__(self):
        super().__init__('CartStateManager')

        robot_domain_id = os.environ.get('ROS_DOMAIN_ID')
        self.robot_id = int(robot_domain_id) - 31
        self.state_machine = StateMachine(STATE_TABLE, 0)
        self.user_id = -1
        self.state_code_timer = 0.1
        self.robot_status_timer = 5

        self.battery_percentage = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.destin_posx = 0.0
        self.destin_posy = 0.0
        self.destin_theta = 0.0
        self.user_id = 0

        self.perception_manager_queue = queue.Queue(maxsize=1)
        self.interface_manager_queue = queue.Queue(maxsize=1)

        self.perception_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, PERCEPTIONMANAGER_PORT)
        self.interface_manager_socket = TCPSocket(CARTSTATEMANAGER_IP, INTERFACEMANAGER_PORT)
        
        self.thread_list = []
        self.perception_manager_socket_start()
        self.interface_manager_socket_start()

        
        self.robot_status_publisher  = self.create_publisher(
            RobotState,
            'robot_state',
            10
        )

        self.human_pose_publisher = self.create_publisher(
            HumanPos,
            'human_pos',
            10
        )

        self.item_destination_req_publisher = self.create_publisher(
            ItemReq,
            'item_req',
            10
        )

        self.user_check_client = self.create_client(
            Usercheck,
            'user_check'
        )

        self.battery_status_subscriber = self.create_subscription(
            Float32,
            'battery/present',
            self.battery_status_callback,
            10
        )

        self.filtered_odom_subscriber = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.filtered_odom_callback,
            10
        )

        self.position_order_subscriber = self.create_subscription(
            PoseOrder,
            'pose_order',
            self.position_order_callback,
            10
        )

        self.state_event_subscriber = self.create_subscription(
            String,
            'state_event',
            self.state_event_callback,
            10
        )

        for t in self.thread_list:
            t.start()
        self.perception_manager_sending_timer = self.create_timer(self.state_code_timer, self.perception_manager_sending)
        self.interface_manager_sending_timer = self.create_timer(self.state_code_timer, self.interface_manager_sending)
        self.robot_status_sending_timer = self.create_timer(self.robot_status_timer, self.robot_status_sending)

    def user_check_client_callback(self, msg : ItemReq):
        if msg.cid != self.robot_id:
            return
        else:
            if (msg.item):
                self.get_logger().info("USER CHECKED")
            else:
                self.user_id = -1
                self.get_logger().error("USER ID NOT IN ID")            

    def state_event_callback(self, msg : String):
        state_event = msg.data
        self.state_machine.recv_event("DEFAULT")

    def position_order_callback(self, msg : PoseOrder):
        
        print(self.robot_id)
        if (self.robot_id == msg.cid):
            self.state_machine.recv_event("DEFAULT")
            self.destin_posx = msg.posx
            self.destin_posy = msg.posy
            self.destin_theta = msg.theta
            print(self.destin_posx, self.destin_posy, self.destin_theta)

    def filtered_odom_callback(self, msg : Odometry):
        self.posx, self.posy = msg.pose.pose.position.x, msg.pose.pose.position.y

    def battery_status_callback(self, msg : Float32):
        self.state_machine.recv_event(msg.data)
        self.battery_percentage = msg.data

    def robot_status_sending(self):
        msg = RobotState()

        current_state = self.state_machine.get_state()
        msg.cid = self.robot_id
        msg.id = self.user_id
        msg.stid = current_state.state_id
        msg.bat = self.battery_percentage
        msg.posx = self.posx
        msg.posy = self.posy

        self.robot_status_publisher.publish(msg)

    def interface_manager_sending(self):
        current_state = self.state_machine.get_state()
        if (self.interface_manager_socket.connected == True):
            try:
                self.interface_manager_socket.send_data(struct.pack("<i",current_state.state_id))
            except:
                pass

    def perception_manager_sending(self):
        current_state = self.state_machine.get_state()
        if (self.perception_manager_socket.connected == True):
            try:
                self.perception_manager_socket.send_data(struct.pack("<i",current_state.state_id))
            except:
                pass

    def perception_manager_socket_start(self):
        self.perception_manager_socket.init_socket()

        connection_thread = threading.Thread(target=self.perception_manager_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.perception_manager_socket.recv_data, args=(self.perception_manager_queue,), daemon=True)
        self.thread_list.append(recv_thread)

        recv_callback_thread = threading.Thread(target=self.perception_manager_recv_callback, daemon=True)
        self.thread_list.append(recv_callback_thread)

        health_callback_thread= threading.Thread(target=self.perception_manager_socket.client_health_check, daemon=True)
        self.thread_list.append(health_callback_thread)

    def interface_manager_socket_start(self):
        self.interface_manager_socket.init_socket()

        connection_thread = threading.Thread(target=self.interface_manager_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.interface_manager_socket.recv_data, args=(self.interface_manager_queue,), daemon=True)
        self.thread_list.append(recv_thread)

        recv_callback_thread = threading.Thread(target=self.interface_manager_recv_callback, daemon=True)
        self.thread_list.append(recv_callback_thread)

        health_callback_thread= threading.Thread(target=self.interface_manager_socket.client_health_check, daemon=True)
        self.thread_list.append(health_callback_thread)


    def perception_manager_recv_callback(self):
        while True:
            if self.perception_manager_queue.full():
                data = self.perception_manager_queue.get_nowait()
                #!@
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
        
        if (struct.unpack("<i",data[8:12])[0] == PERCEPTIONMANAGER_FUNCTION1_ID):
            user_checked = struct.unpack(PERCEPTIONMANAGER_FUNCTION_1, data[12:])[0]
            if user_checked:
                self.state_machine.recv_event("DEFAULT")


        if (struct.unpack("<i",data[8:12])[0] == PERCEPTIONMANAGER_FUNCTION2_ID):
            cam_data = struct.unpack(PERCEPTIONMANAGER_FUNCTION_2, data[12:])
            msg = HumanPos()

            msg.camid = cam_data[2]
            msg.posx = cam_data[3]
            msg.posy = cam_data[4]
            msg.distance = cam_data[5]

            self.human_pose_publisher.publish(msg)

    def interface_manager_data_check(self, data):
        if (struct.unpack("<i",data[0:4])[0] != CARTSTATEMANAGER_ID):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM INTERFACEMANAGER")
            return
        
        if (struct.unpack("<i",data[8:12])[0] == INTERFACEMANAGER_FUNCTION1_ID):
            user_id = struct.unpack(INTERFACEMANAGER_FUNCTION_1, data[12:])[0]

            self.user_check_client.wait_for_service()

            req = Usercheck.Request()
            req.req.cid = self.robot_id
            req.req.user_id = user_id
            response = self.user_check_client.call_async(req)
            rclpy.spin_until_future_complete(self, response)

            if (response.result()):
                self.user_id = user_id
            else:
                self.user_id = -1

            send_data = struct.pack(INTERFACECOMMON+CARTSTATEMANAGER_FUNCTION_1, 
                                    INTERFACEMANAGER_ID, 1, CARTSTATEMANAGER_FUNCTION1_ID, True)
            self.interface_manager_socket.send_data(send_data)
        
        elif (struct.unpack("<i",data[8:12])[0] == INTERFACEMANAGER_FUNCTION2_ID):
            data = struct.unpack(INTERFACEMANAGER_FUNCTION_2, data[12:])
            if data[0] == 1:
                print(f"Item ID : {data[1]}")

                msg = ItemReq()
                msg.cid = self.robot_id
                msg.item = data[1]

                self.item_destination_req_publisher.publish(msg)

            elif data[0] == 2:
                print("MODE SELECTION")
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

        



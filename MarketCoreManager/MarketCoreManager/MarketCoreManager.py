import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from pinky_msgs.msg import RobotState, PoseOrder, ItemReq

from .tcp_socket import TCPSocket
from .config import *
import time
import threading
import queue
import struct
import mysql.connector
from .awsconfig import *


class MarketCoreManager(Node):
    def __init__(self):
        super().__init__('MarketCoreManager')

        self.dbms = mysql.connector.connect(
            host = AWSHOST,
            port = AWSPORT,
            user = AWSUSER,
            password = AWSPASSWORD,
            database = AWSDATABASE
        )
        self.cursor = self.dbms.cursor(buffered=True)

        self.ongui_recv_queue = queue.Queue(maxsize=1)
        self.admingui_recv_queue = queue.Queue(maxsize=1)
        self.robot_status_queue = queue.Queue(maxsize=100)

        self.ongui_socket = TCPSocket(MARKETCOREMANAGER_IP, ONGUI_PORT)
        self.admingui_socket = TCPSocket(MARKETCOREMANAGER_IP, ADMINGUI_PORT)
        self.thread_list = []

        self.task_list = []
        self.total_position_list = []
        self.pinky31_item_list = []
        self.pinky32_item_list = []

        self.robot_status_period = 0.5
        self.task_checker_period = 3
        self.robot_command_period = 0.1

        self.ongui_socket_start()
        self.admingui_socket_start()
        
        self.robot_status_subscriber = self.create_subscription(
            RobotState,
            'robot_state',
            self.pinky31_callback,
            10
        )
        self.robot_status_subscriber = self.create_subscription(
            RobotState,
            'pinky32/robot_state',
            self.pinky32_callback,
            10
        )
        self.robot_position_order_publisher = self.create_publisher(
            PoseOrder,
            'pose_order',
            10
        )
        
        self.item_req_subscriber = self.create_subscription(
            ItemReq,
            'item_req',
            self.item_req_callback,
            10
        )


        self.user_check_req_subscriber = self.create_subscription(
            ItemReq,
            'user_check',
            self.user_check_req_callback,
            10
        )

        self.user_check_resp_publisher = self.create_publisher(
            ItemReq,
            'user_resp',
            10
        )


        self.robot_status_timer = self.create_timer(self.robot_status_period, self.robot_status_callback)
        self.task_checker = self.create_timer(self.task_checker_period, self.task_checker_callback)
        self.robot_command_timer = self.create_timer(self.robot_command_period, self.robot_command_callback)


        for t in self.thread_list:
            t.start()

        self.get_logger().info(f"INIT COMPLETE")

    def user_check_req_callback(self, msg):   
        sql = "SELECT id FROM user_info"
        self.cursor.execute(sql)
        result = self.cursor.fetchall()
        data = False
        for i in result:
            if (i[0] == msg.item):
                data = True
                break
        
        pub_msg = ItemReq
        pub_msg.cid = msg.cid
        if (data):
            pub_msg.item = 1
        else:
            pub_msg.item = 0
        self.user_check_resp_publisher.publish(pub_msg)

    def item_req_callback(self, msg):
        item_id = msg.item
        
        pub_msg = PoseOrder()

        sql = "SELECT location.coordinate_x, location.coordinate_y, location.coordinate_theta " \
        "FROM location, item_list " \
        f"WHERE item_list.iid = {item_id} " \
        "AND item_list.location_id = location.id"
        self.cursor.execute(sql)
        data = self.cursor.fetchall()
        print(data)
        if (len(data) > 0):
            pub_msg.cid = msg.cid
            pub_msg.posx = data[0][0]
            pub_msg.posy = data[0][1]
            pub_msg.theta = data[0][2]
            self.robot_position_order_publisher.publish(pub_msg)
        else:
            pass

    def robot_command_callback(self):
        sql = "SELECT * FROM "

    def robot_status_callback(self):
        if (not self.robot_status_queue.empty()):
            if self.admingui_socket.connected == True:
                self.admingui_socket.send_data(self.robot_status_queue.get_nowait())

    def task_checker_callback(self):
        sql = "SELECT * FROM order_list"
        self.cursor.execute(sql)
        result = self.cursor.fetchall()
        for i in result:
            if (i[2] == 0):
                order_id = i[0]
                order_list = i[4]
                self.task_assign(order_id, order_list)
                break
    
    def task_assign(self, order_id, order_list):
        sql = "SELECT * FROM robot_state"
        self.cursor.execute(sql)
        result = self.cursor.fetchall()
        robot_id = -1
        for i in result:
            if (i[1] == 2):
                if (i[0] == 0):
                    self.pinky31_item_list = list(map(int, order_list[1:-1].split(", ")))
                    robot_id = i[0]
                    break
                elif (i[0] == 1):
                    self.pinky32_item_list = list(map(int, order_list[1:-1].split(", ")))
                    robot_id = i[0]
                    break
                else:
                    self.get_logger().info(f"CART ID IS NOT VERIFIED!")  
            else:
                pass

        if (not (robot_id < 0)): 
            sql = f"INSERT task (order_id, task_status_id, robot_id) VALUES {order_id}, {1}, {robot_id}"
            self.cursor.execute(sql)
            self.dbms.commit()

            sql = f"UPDATE order_list SET order_status_id = 1 WHERE order_id = {order_id}"
            self.cursor.execute(sql)
            self.dbms.commit()
        else:
            self.get_logger().info(f"NONE OF CART IS IDLE!")      

    def ongui_socket_start(self) -> None:
        self.ongui_socket.init_socket()

        connection_thread = threading.Thread(target=self.ongui_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)
        
        recv_thread = threading.Thread(target=self.ongui_socket.recv_data, args=(self.ongui_recv_queue,), daemon=True)
        self.thread_list.append(recv_thread)

        ongui_callback_thread = threading.Thread(target=self.ongui_recv_callback, daemon=True)
        self.thread_list.append(ongui_callback_thread)

        health_callback_thread= threading.Thread(target=self.ongui_socket.client_health_check, daemon=True)
        self.thread_list.append(health_callback_thread)


    def admingui_socket_start(self) -> None:
        self.admingui_socket.init_socket()

        connection_thread = threading.Thread(target=self.admingui_socket.connect_socket, daemon=True)
        self.thread_list.append(connection_thread)

        recv_thread = threading.Thread(target=self.admingui_socket.recv_data, args=(self.admingui_recv_queue,), daemon=True)
        self.thread_list.append(recv_thread)

        admin_callback_thread = threading.Thread(target=self.admingui_recv_callback, daemon=True)
        self.thread_list.append(admin_callback_thread)

        health_callback_thread= threading.Thread(target=self.admingui_socket.client_health_check, daemon=True)
        self.thread_list.append(health_callback_thread)

    def ongui_recv_callback(self):
        while True:
            if self.ongui_recv_queue.full():
                data = self.ongui_recv_queue.get_nowait()
                #!@
                print(data)
                self.ongui_data_check(data)
            else:
                pass
            time.sleep(0.1)

    def ongui_data_check(self, data):
        if (struct.unpack("<i",data[0:4])[0] != MARKETCOREMANAGER_ID):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM ONGUI")
            return
        
        if (struct.unpack("<i",data[8:12])[0] == ONGUI_REQ1_ID):
            self.user_id = struct.unpack(ONGUI_REQ1, data[12:])[0]
            
            sql = "SELECT id FROM user_info"
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            data = False
            for i in result:
                if (i[0] == self.user_id):
                    data = True
                    break
            
            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV1, ONGUI_ID, 1, ONGUI_RECV1_ID, data)
            self.ongui_socket.send_data(send_data)

            sql = "SELECT quantity FROM item_list"
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            data = []
            for i in result:
                data.append(i[0])
            
            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV2, ONGUI_ID, 68, ONGUI_RECV2_ID, self.user_id,  *data)
            self.ongui_socket.send_data(send_data)

        elif (struct.unpack("<i",data[8:12])[0] == ONGUI_REQ2_ID):
            item_list = list(struct.unpack(ONGUI_REQ2, data[12:]))

            sql = "INSERT INTO order_list (user_id, order_status_id, order_item) VALUES (%s, %s, %s)"
            self.cursor.execute(sql, (self.user_id, 0, str(item_list[1:-1])))
            self.dbms.commit()

            send_data = struct.pack(INTERFACECOMMON+ONGUI_RECV3, ONGUI_ID,  1, ONGUI_RECV3_ID, True)
            self.ongui_socket.send_data(send_data)

        else:
            self.get_logger().info(f"WRONG DATA FUNCTION CODE FROM ONGUI")
        
    def admingui_recv_callback(self):
        while True:
            if self.admingui_recv_queue.full():
                data = self.admingui_recv_queue.get_nowait()
                print(data)
                self.admingui_data_check(data)
            else:
                pass
            time.sleep(0.1)
    
    def admingui_data_check(self, data):
        if (struct.unpack("<i",data[0:4])[0] != MARKETCOREMANAGER_ID):
            self.get_logger().info(f"WRONG DATA DESTINATION FROM ADMINGUI")
            return
                
        if (struct.unpack("<i",data[8:12])[0] == ADMINGUI_REQ1_ID):
            admin_id = struct.unpack(ONGUI_REQ1, data[12:])

            data = False
            if (admin_id[0] == 1):
                data = True
            
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV1, ADMINGUI_ID, 1, ADMINGUI_RECV1_ID, data)
            self.admingui_socket.send_data(send_data)

            sql = "SELECT quantity FROM item_list"
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            data = []
            for i in result:
                data.append(i[0])
            
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV2, ADMINGUI_ID, 68, ADMINGUI_RECV2_ID, 1, *data)
            self.admingui_socket.send_data(send_data)

        elif (struct.unpack("<i",data[8:12])[0] == ADMINGUI_REQ2_ID):
            updated_item_list = struct.unpack(ONGUI_REQ2, data[12:])
            
            for idx, i in enumerate(updated_item_list):
                sql = f"UPDATE item_list set quantity = {i[0]} where item_id = {idx}"
                self.cursor.execute(sql)
                self.dbms.commit()

            sql = "SELECT quantity FROM item_list"
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            data = []
            for i in result:
                data.append(i[0])
            
            send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV2, ADMINGUI_ID,  1, ADMINGUI_RECV2_ID, 1, *data)
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
        if self.robot_status_queue.full():
            self.robot_status_queue.get_nowait()
        self.robot_status_queue.put_nowait(send_data)
        
        sql = f"SELECT robot_id FROM robot_state WHERE robot_id = {cart_id}"
        self.cursor.execute(sql)
        data = self.cursor.fetchall()

        if (len(data) > 0):
                sql = "UPDATE robot_state SET " \
                "robot_status_id = %s, " \
                "battery_state_id = %s, " \
                "error_status_id = 0, " \
                "connection_state_id = 1, " \
                "coordinate_x = %s, " \
                "coordinate_y = %s " \
                "WHERE robot_id = %s"

                self.cursor.execute(sql, (user_id, battery, posx, posy, cart_id))
                self.dbms.commit()
        else:
            sql = "INSERT INTO robot_state (robot_id, robot_status_id, battery_state_id, error_status_id, connection_state_id, coordinate_x, coordinate_y)" \
            " VALUES (%s, %s, %s, 0, 1, %s, %s)"
            self.cursor.execute(sql, (cart_id, state_id, battery, posx, posy))
            self.dbms.commit()

    def pinky32_callback(self, msg):
        cart_id = msg.cid
        user_id = msg.id
        state_id = msg.stid
        battery = msg.bat
        posx = msg.posx
        posy = msg.posy

        send_data = struct.pack(INTERFACECOMMON+ADMINGUI_RECV3, ADMINGUI_ID, 32, ADMINGUI_RECV3_ID, *[cart_id, user_id, state_id, battery, posx, posy])
        self.robot_status_queue.put_nowait(send_data)

    def __del__(self):
        self.dbms.close()
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

        
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.msg import SpeedLimit


import math
import time

from geometry_msgs.msg import Twist, TransformStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float32, String
from pinky_msgs.msg import Encoder, RobotState, PoseOrder, HumanPos

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from collections import deque

from .waypoint_dict import *
from .dynamixel_driver import DynamixelDriver
from .function_list import *
from .led import *
from .config import * 

ANGLE_TOLERANCE = 0.01
DISTANCE_TOLERANCE = 0.01

class ActuatorManager(Node):
    def __init__(self):
        super().__init__('ActuatorManager')

        self.led = LED()
        self.driver = DynamixelDriver(SERIAL_PORT_NAME, BAUDRATE, DYNAMIXEL_IDS)

        self.distance = 0
        # PID parameters
        self.angle_kp = 1.0
        self.angle_ki = 0.0
        self.angle_kd = 0.2

        self.dist_kp = 0.8
        self.dist_ki = 0.0
        self.dist_kd = 0.1

        # PID internal variables
        self.prev_angle_error = 0.0
        self.angle_integral = 0.0

        self.prev_dist_error = 0.0
        self.dist_integral = 0.0

        self.prev_time = time.time()

        if not self.driver.begin():
            self.get_logger().error("Failed to open serial port! Shutting down.")
            return
        
        if not self.driver.initialize_motors():
            self.get_logger().error("Failed to initialize motors! Shutting down.")
            self.driver.terminate()
            return
        
        self.get_logger().info("Waiting for motors to be ready...")
        time.sleep(1.0)


        self.get_logger().info("3. Setting initial RPM to zero...")
        if not self.driver.set_double_rpm(0, 0):
            self.get_logger().error("Failed to set initial RPM! Shutting down.")
            self.driver.terminate()
            return

        self.get_logger().info("4. Reading initial encoder values...")
        _, _, self.last_encoder_l, self.last_encoder_r = self.driver.get_feedback()
        if self.last_encoder_l is None:
            self.get_logger().error("Failed to read initial encoder position! Shutting down.")
            self.driver.terminate()
            return

        self.get_logger().info(f"Initial Encoder read: L={self.last_encoder_l}, R={self.last_encoder_r}. Controller is responsive.")
        

        # --- 상태 변수 ---
        self.current_state = 0                 # IDLE / NAVIGATING
        self.current_goal_handle = None        # Nav2 goal handle
        self.current_robot_pose = None         # /amcl_pose 에서 받은 현재 포즈
        self.last_goal_command = None          # 마지막 주행 명령 (디버깅용)

        # 재계획 관련
        self.current_path_ids = []             # 현재 Nav2에 보낸 waypoint ID 순서
        self.current_goal_target = None        # 최종 목표 waypoint ID
        self.replan_attempts = 0               # 자동 재계획 시도 횟수

        # 진행 상황 모니터링(막혔는지 판단용)
        self.last_progress_time = None
        self.last_progress_distance = None
        self.stuck_timeout_sec = 8.0           # 이 시간 동안 거의 못 움직이면 막힌 것으로 간주
        self.stuck_distance_threshold = 0.05   # 이만큼 이상 가까워지지 않으면 "진행 없음"으로 간주

        # 목표 근처 판정(맵이 작아서 10cm 이내로 꽉 채우는 느낌)
        self.goal_near_distance = 0.10   # 10cm 안쪽이면 거의 도착했다고 봄

        # add 이후 첫 shelf 에서만 10cm 추가 전진하기 위한 플래그
        self.post_add_shelf_forward = False

        # --- 4. Nav2 액션 클라이언트 (2개) ---
        self._action_single_goal_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )
        self._action_path_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses'
        )

        # --- 5. /cmd_vel 발행기 ---
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.speed_limit_publisehr = self.create_publisher(
            SpeedLimit,
            'speed_limit',
            10
        )

        self.state_event_publisher = self.create_publisher(
            String,
            'state_event',
            10
        )

        # --- 6. /amcl_pose 구독 (현재 위치 확인) ---
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.robot_status_subscriber  = self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_callback,
            10
        )

        self.position_order_subscriber = self.create_subscription(
            PoseOrder,
            'pose_order',
            self.position_order_callback,
            10
        )

        self.human_dir_subscriber = self.create_subscription(
            HumanPos,
            'human_position',
            self.human_position_callback,
            10
        )


        # --- 8. 진행 상황 체크용 타이머 (1초마다) ---
        self.state_action_timer = self.create_timer(0.02, self.state_action_callback)
        self.speed_limit_timer = self.create_timer(0.02, self.speed_limit_callback)









        self.joint_pub = self.create_publisher(
            JointState,
            JOINT_PUB_TOPIC_NAME,
            10
        )
        
        self.encoder_pub = self.create_publisher(
            Encoder,
            "encoder",
            10
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            TWIST_SUB_TOPIC_NAME,
            self.twist_callback,
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.is_initialized = True
        self.get_logger().info('Pinky.')

    def state_action_callback(self):
        if (self.current_state in [ONLINE_DRIVE, OFFLINE_DRIVE, RETRUN_CHRG]):
            self.drive_order()
        elif (self.current_state == ONLINE_PICKUP):
            self.picking_order()
        elif (self.current_state == OFFLINE_FOLLOW):
            self.follow_order()
        elif (self.current_state in [ONLINE_PACKING, OFFLINE_PACKING]):
            self.packing_order()
        elif (self.current_state in [TASK_STBY, ONLINE_STBY, OFFLINE_STBY, ONLINE_END, OFFLINE_END]):
            self.standby_order()
        else:
            pass

    def speed_limit_callback(self):
        if (self.current_state != OFFLINE_DRIVE):
            return
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' 
        msg.percentage = True
        
        percentage = 100 * (1- abs(self.distance)/3)


        msg.speed_limit = percentage # Maximum allowed speed in m/s

        self.speed_limit_publisehr.publish(msg)

    def human_position_callback(self, msg):
        self.cam_id = msg.camid
        human_x = msg.posx
        human_y = msg.posy
        self.distance = msg.distance

        self.relative_angle = calculate_cam_angle(human_x, human_y, self.distance, self.cam_id)

    def drive_order(self):
        if (len(self.current_path_ids) > 0):
            return
        
        current_wpid = find_nearest_waypoint(self.posx, self.posy)
        destination_wpid = find_nearest_waypoint(self.destination_x, self.destination_y)
        if destination_wpid == 21 or destination_wpid == 20:
            self.ending_sqeuence = True
        else:
            self.ending_sqeuence = False
        
        path_ids = find_shortest_path(current_wpid, destination_wpid)

        if path_ids:
            self.get_logger().info(f"경로 찾음: {' -> '.join(path_ids)}")

            self.send_waypoint_path(path_ids, goal_id = destination_wpid, final_theta = self.destination_theta)
        else:
            self.get_logger().error(f"경로를 찾을 수 없습니다: {current_wpid} -> {destination_wpid}")
            self.send_state_event("STBY")

    def picking_order(self):
        self.led.fill((255, 0, 0))
        i = 100
        while (i > 0):
            self.pid_control(0, i)
            i -= 1
        self.led.clear()
        
        result = True
        while result:
            result = self.pid_control(self.theta - math.pi, 0)
            if (result == False):
                self.send_state_event("STBY")
                break
        
    def follow_order(self):
        self.led.fill((0, 255, 0))
        self.pid_control(self.relative_angle, self.distance)

        
    def pid_control(self, relative_angle, distance):
        """
        Callback function for PID control based on:
            relative_angle: relative bearing to target (rad)
            distance: relative distance to target (m)
        """
        
        # Time
        now = time.time()
        dt = max(now - self.prev_time, 1e-6)  # avoid division by zero
        self.prev_time = now

        # ------------------------
        # Angle PID (angular.z)
        # ------------------------
        angle_error = relative_angle
        self.angle_integral += angle_error * dt
        angle_derivative = (angle_error - self.prev_angle_error) / dt
        self.prev_angle_error = angle_error

        w = (self.angle_kp * angle_error +
             self.angle_ki * self.angle_integral +
             self.angle_kd * angle_derivative)

        # ------------------------
        # Distance PID (linear.x)
        # ------------------------
        dist_error = distance
        self.dist_integral += dist_error * dt
        dist_derivative = (dist_error - self.prev_dist_error) / dt
        self.prev_dist_error = dist_error

        v = (self.dist_kp * dist_error +
             self.dist_ki * self.dist_integral +
             self.dist_kd * dist_derivative)


        # ------------------------
        # Output command
        # ------------------------
        if (angle_error < ANGLE_TOLERANCE and dist_error < DISTANCE_TOLERANCE):
            return False
        else:
            cmd = Twist()
            cmd.linear.x = max(min(v, 1.0), -1.0)       # clamp velocity
            cmd.angular.z = max(min(w, 1.5), -1.5)      # clamp rotation
            if self.cam_id == 1:
                self.cmd_vel_publisher.publish(cmd)
            else:
                cmd.linear.x = -cmd.linear.x
                self.cmd_vel_publisher.publish(cmd)
            self.get_logger().info(
                f"PID -> v: {cmd.linear.x:.2f}, w: {cmd.angular.z:.2f}, angle_err: {angle_error:.2f}, dist_err: {dist_error:.2f}"
            )
            return True
        

    def packing_order(self):
        self.led.rainbow(iterations=3)
        self.send_state_event("END")

    def standby_order(self):
        # self.get_logger().info("pause 명령 수신: Nav2 주행 취소 시도")
        self.cancel_navigation()
            # self.get_logger().warn("취소할 Nav2 주행이 없습니다.")

    def cancel_navigation(self):
        if self.current_goal_handle is None:
            return False

        try:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info(">>> Nav2 주행 취소 요청 전송")
        except Exception as e:
            self.get_logger().error(f"Nav2 취소 중 오류: {e}")
            return False

        self.current_path_ids = []
        self.current_goal_target = None
        return True

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.posx, self.posy = msg.pose.pose.position.x, msg.pose.pose.position.y
        _, _, self.theta = euler_from_quaternion(msg.pose.pose.orientation)

    def robot_state_callback(self, msg):
        self.robot_id = msg.cid
        self.user_id = msg.id
        self.current_state = msg.stid
        self.battery = msg.bat

    def position_order_callback(self, msg : PoseOrder):
        if (self.robot_id == msg.cid):
            self.destination_x = msg.posx
            self.destination_y = msg.posy
            self.destination_theta = msg.theta
        else:
            self.get_logger().error(f"NOT FOR ME")

    def send_waypoint_path(self, path_ids, goal_id=None, final_theta = None, from_replan=False):
        if not self._action_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 액션 서버(/navigate_through_poses)를 찾을 수 없습니다.")
            self.send_state_event("STBY")
            return

        self.current_path_ids = list(path_ids)
        if goal_id is not None:
            self.current_goal_target = goal_id

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for i, wp_id in enumerate(path_ids):
            x, y = xy_from_wp(wp_id)
            if x is None:
                continue

            theta = None

            # 다음 waypoint가 있다면: 현재 → 다음으로 가는 방향
            if i < len(path_ids) - 1:
                nx, ny = xy_from_wp(path_ids[i + 1])
                if nx is not None:
                    theta = math.atan2(ny - y, nx - x)
            # 마지막 waypoint인데, 앞에 하나라도 있으면: 이전 → 현재 방향
            elif i > 0:
                if final_theta is not None:
                    theta = final_theta
                else:
                    theta = 0.0


            goal_msg.poses.append(create_pose_stamped(x, y, theta_rad=theta))

        if not goal_msg.poses:
            self.get_logger().error("경유지 목록이 비어있습니다.")
            self.send_state_event("STBY")
            return

        tag = "재계획 경로" if from_replan else "원래 경로"
        self.get_logger().info(
            f"Nav2에 {len(goal_msg.poses)}개 경유지 전송 ({tag}: {' -> '.join(path_ids)})"
        )

        self.reset_progress_monitor()
        send_goal_future = self._action_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_handle_callback)

    def goal_handle_callback(self, future):
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error("목표 핸들을 받지 못했습니다.")
            self.send_state_event("STBY")
            return

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다 (Goal rejected)")
            self.send_state_event("STBY")
            return

        self.get_logger().info("목표가 수락되었습니다 (Goal accepted)")
        self.current_goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f, gh=goal_handle: self.get_result_callback(f, gh)
        )
    
    def get_result_callback(self, future, goal_handle):
        if goal_handle is not self.current_goal_handle:
            self.get_logger().info("[OLD Nav2 RESULT] 이전 goal에 대한 결과 수신을 무시합니다.")
            return

        status = future.result().status

        self.get_logger().info(
            f"[Nav2 RESULT RAW] status={status}, result_msg={future.result().result}"
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표 지점 도착 성공!")
            if (self.ending_sqeuence):
                if (self.current_state == 255):
                    self.send_state_event("CHARGE")
                    self.ending_sqeuence = False
                else:
                    self.send_state_event("PACKING")
            else:
                self.send_state_event("STBY")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("목표가 취소되었습니다 (STATUS_CANCELED)")
            self.send_state_event("STBY")
        else:
            self.get_logger().error(f"목표 지점 도착 실패! (Status: {status})")
            self.send_state_event("STBY")

        
        self.current_goal_handle = None

    def send_state_event(self, data):
        pub_msg = String()
        pub_msg.data = data
        self.state_event_publisher.publish(pub_msg)












    def twist_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        v_l = linear_x - (angular_z * WHEEL_BASE / 2.0)
        v_r = linear_x + (angular_z * WHEEL_BASE / 2.0)

        wheel_rads_l = v_l / WHEEL_RAD
        wheel_rads_r = v_r / WHEEL_RAD

        rpm_l = wheel_rads_l * 60.0 / (2 * math.pi)
        rpm_r = -wheel_rads_r * 60.0 / (2 * math.pi)

        max_val = max(abs(rpm_l), abs(rpm_r))
        MAX_RPM = 100.0
        if max_val > MAX_RPM:
            scale = MAX_RPM / max_val
            rpm_l *= scale
            rpm_r *= scale

        if not self.driver.set_double_rpm(rpm_l, rpm_r):
            self.get_logger().warn("Failed to send motor command.")

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        feedback = self.driver.get_feedback()
        if feedback[0] is None:
            self.get_logger().warn("Failed to read motor data. Skipping update cycle.")
            return
        rpm_l, rpm_r, encoder_l, encoder_r = feedback

        delta_l = encoder_l - self.last_encoder_l
        delta_r = -(encoder_r - self.last_encoder_r)
        
        self.last_encoder_l = encoder_l
        self.last_encoder_r = encoder_r

        self._publish_encoder(current_time, rpm_l, rpm_r, encoder_l, encoder_r)
        self._publish_joint_states(current_time, rpm_l, rpm_r)

        self.last_time = current_time
        
    def _publish_encoder(self, current_time, rpm_l, rpm_r, encoder_l, encoder_r):
        encoder_msg = Encoder()
        
        encoder_msg.header.stamp = current_time.to_msg()
        encoder_msg.rpm_l = float(rpm_l)
        encoder_msg.rpm_r = float(rpm_r)
        encoder_msg.pos_raw_l = float(encoder_l)
        encoder_msg.pos_raw_r = float(encoder_r)

        self.encoder_pub.publish(encoder_msg)

    def _publish_joint_states(self, current_time, rpm_l, rpm_r):
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R]
        
        pos_l_rad = (self.last_encoder_l / PULSE_PER_ROT) * (2 * math.pi)
        pos_r_rad = (self.last_encoder_r / PULSE_PER_ROT) * (2 * math.pi)
        joint_msg.position = [pos_l_rad, pos_r_rad]
        joint_msg.velocity = [rpm_l * RPM2RAD, rpm_r * RPM2RAD]

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ActuatorManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



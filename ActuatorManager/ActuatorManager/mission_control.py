import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from collections import deque

from .waypoint_dict import *

class MissionControlNode(Node):

    def __init__(self):
        super().__init__('mission_control_node')

        # --- 상태 변수 ---
        self.current_state = "IDLE"            # IDLE / NAVIGATING
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 6. /amcl_pose 구독 (현재 위치 확인) ---
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # --- 7. /nav_command 구독 (텍스트 명령) ---
        self.subscription = self.create_subscription(
            String,
            '/nav_command',
            self.command_callback,
            10
        )

        # --- 8. 진행 상황 체크용 타이머 (1초마다) ---
        self.stuck_timer = self.create_timer(1.0, self.check_stuck)

        self.get_logger().info("--- 미션 컨트롤 노드 준비 완료 (v19 - yaw 개선) ---")
        self.get_logger().info("예시 명령:")
        self.get_logger().info("  '10'         (현재 위치 -> 10번 자동 경로)")
        self.get_logger().info("  '1,6'        (1번 -> 6번 자동 경로)")
        self.get_logger().info("  'here,12'    (현재 위치 기준 -> 12번)")
        self.get_logger().info("  'pause'      (현재 Nav2 주행 취소)")
        self.get_logger().info("  'add'        (Nav2 끄고 10cm 전진 + 3초 대기 + 180도 회전)")
        self.get_logger().info("  '1 shelf'    (현재 위치 -> 1번 진열대 앞에서 정지)")
        self.get_logger().info("  'shelf 10'   (형식은 '10 shelf', 'shelf10' 모두 가능)")

    # ------------------------------------------------------------------
    #  AMCL 콜백 & 유틸리티
    # ------------------------------------------------------------------
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_robot_pose = msg.pose.pose

    def _xy_from_wp(self, wp_id):
        """waypoints 항목에서 x,y만 안전하게 추출"""
        if wp_id not in WAYPOINTS:
            return None, None
        data = WAYPOINTS[wp_id]
        if len(data) >= 2:
            return float(data[0]), float(data[1])
        return None, None

    def find_nearest_waypoint(self):
        """현재 위치 기준으로 가장 가까운 waypoint ID 반환"""
        if self.current_robot_pose is None:
            self.get_logger().error("로봇의 현재 위치(/amcl_pose)를 알 수 없습니다!")
            return None

        robot_x = self.current_robot_pose.position.x
        robot_y = self.current_robot_pose.position.y

        min_dist = float('inf')
        nearest_wp = None
        for wp_id in WAYPOINTS.keys():
            wp_x, wp_y = self._xy_from_wp(wp_id)
            if wp_x is None:
                continue
            dist = math.hypot(robot_x - wp_x, robot_y - wp_y)
            if dist < min_dist:
                min_dist = dist
                nearest_wp = wp_id

        if nearest_wp is not None:
            self.get_logger().info(
                f"[NEAREST_WP] robot=({robot_x:.2f},{robot_y:.2f}), "
                f"nearest={nearest_wp} (dist={min_dist:.2f})"
            )
        return nearest_wp

    def find_shortest_path(self, start, end, blocked_nodes=None):
        """BFS로 waypoint_graph 기반 최단 경로 찾기 (blocked_nodes 제외)"""
        if start not in WAYPOINT_GRAPH or end not in WAYPOINT_GRAPH:
            return None

        blocked = set(blocked_nodes) if blocked_nodes else set()
        blocked.discard(start)
        blocked.discard(end)

        queue = deque([[start, [start]]])
        visited = set()

        while queue:
            node, path = queue.popleft()
            if node in visited or node in blocked:
                continue
            visited.add(node)

            if node == end:
                return path

            for neighbor in WAYPOINT_GRAPH[node]:
                if neighbor not in visited and neighbor not in blocked:
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append([neighbor, new_path])

        return None

    def distance_to_goal(self):
        """현재 로봇 위치와 최종 목표 waypoint 사이의 거리"""
        if self.current_robot_pose is None:
            return None
        if self.current_goal_target is None:
            return None
        if self.current_goal_target not in WAYPOINTS:
            return None

        gx, gy = self._xy_from_wp(self.current_goal_target)
        rx = self.current_robot_pose.position.x
        ry = self.current_robot_pose.position.y
        return math.hypot(rx - gx, ry - gy)

    def reset_progress_monitor(self):
        """새 목표가 설정될 때 진행 상황 모니터 초기화"""
        self.last_progress_time = self.get_clock().now()
        self.last_progress_distance = self.distance_to_goal()

    # ------------------------------------------------------------------
    #  진행 멈춤 감지 타이머 콜백
    # ------------------------------------------------------------------
    def check_stuck(self):
        if self.current_state != "NAVIGATING":
            self.last_progress_time = None
            self.last_progress_distance = None
            return

        dist = self.distance_to_goal()
        if dist is None:
            return

        now = self.get_clock().now()

        if self.last_progress_time is None or self.last_progress_distance is None:
            self.last_progress_time = now
            self.last_progress_distance = dist
            return

        # 목표 근처면(10cm 이내) 막혔다고 보지 않음
        if dist < self.goal_near_distance:
            self.last_progress_time = now
            self.last_progress_distance = dist
            return

        elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        progress = self.last_progress_distance - dist  # 양수면 목표에 가까워짐

        self.get_logger().info(
            f"[PROGRESS DEBUG] goal={self.current_goal_target}, "
            f"dist={dist:.3f}, last_dist={self.last_progress_distance:.3f}, "
            f"elapsed={elapsed:.1f}s, progress={progress:.3f}"
        )

        if progress > self.stuck_distance_threshold:
            self.get_logger().info("[PROGRESS DEBUG] 충분한 전진 감지 → baseline 업데이트")
            self.last_progress_time = now
            self.last_progress_distance = dist
            return

        if elapsed > self.stuck_timeout_sec and self.current_path_ids:
            self.get_logger().warn(
                f"최근 {elapsed:.1f}초 동안 목표({self.current_goal_target})와의 거리가 "
                f"{progress:.3f}m밖에 줄지 않았습니다. (막힘 감지, 자동 재계획은 나중에 다시 조정 가능)"
            )
            # 나중에 replan_from_current_position() 부활 가능
            self.last_progress_time = now
            self.last_progress_distance = dist

    # ------------------------------------------------------------------
    #  명령 콜백
    # ------------------------------------------------------------------
    def command_callback(self, msg: String):
        raw = msg.data.strip()
        command = raw.lower().replace(" ", "")

        # add 이후 첫 shelf 에만 10cm 전진하고 싶으므로,
        # add & shelf 외의 어떤 명령이 오면 플래그 리셋
        if ("shelf" not in command) and (command != "add"):
            self.post_add_shelf_forward = False

        # 1) pause = Nav2 주행 취소
        if command == "pause":
            self.pause_navigation()
            return

        # 2) add = Nav2 취소 + 10cm 전진 + 3초 대기 + 제자리 180도 회전
        if command == "add":
            self.handle_add_command()
            return

        # 3) shelf 명령 (예: "1 shelf", "shelf10")
        if "shelf" in command:
            self.handle_shelf_command(raw)  # raw 그대로 넘겨서 로그 예쁘게
            return

        # 4) 그 외는 모두 Nav2 주행 명령
        self.last_goal_command = command
        self.current_state = "NAVIGATING"
        self.replan_attempts = 0

        # 4-1) "start,end" 형식 (예: "1,6" 또는 "here,6")
        if "," in command:
            try:
                start, end = command.split(",", 1)

                if start == "here":
                    start = self.find_nearest_waypoint()
                if start is None:
                    self.get_logger().error("경로 시작점을 찾을 수 없습니다.")
                    self.current_state = "IDLE"
                    return

                self.current_goal_target = end
                self.get_logger().info(f"경로 탐색 명령: {start} -> {end}")
                path_ids = self.find_shortest_path(start, end)
                if path_ids:
                    self.get_logger().info(f"경로 찾음: {' -> '.join(path_ids)}")
                    self.send_waypoint_path(path_ids, goal_id=end)
                else:
                    self.get_logger().error(f"경로를 찾을 수 없습니다: {start} -> {end}")
                    self.current_state = "IDLE"

            except Exception as e:
                self.get_logger().error(f"경로 명령 처리 실패: {e}")
                self.current_state = "IDLE"

        # 4-2) 단일 waypoint 번호 (예: "10")
        elif command in WAYPOINTS:
            start = self.find_nearest_waypoint()
            if start is None:
                self.current_state = "IDLE"
                return

            end = command
            self.current_goal_target = end

            if start == end:
                self.get_logger().info(
                    f"이미 가장 가까운 지점({start})에 있습니다. 단일 목표로 전송합니다."
                )
                x, y = self._xy_from_wp(end)
                self.current_path_ids = [end]
                self.send_single_goal(x, y)
            else:
                self.get_logger().info(
                    f"단일 목표 수신: 현재 위치({start}) -> {end}"
                )
                path_ids = self.find_shortest_path(start, end)
                if path_ids:
                    self.get_logger().info(f"경로 찾음: {' -> '.join(path_ids)}")
                    self.send_waypoint_path(path_ids, goal_id=end)
                else:
                    self.get_logger().error(
                        f"경로를 찾을 수 없습니다: {start} -> {end}"
                    )
                    self.current_state = "IDLE"
        else:
            self.get_logger().warn(f"알 수 없는 명령입니다: {raw}")

    # ------------------------------------------------------------------
    #  새 shelf 명령 처리 (진열대 → 전용 waypoint + 방향)
    # ------------------------------------------------------------------
    def handle_shelf_command(self, raw_command: str):
        """
        shelf 명령:
        - 기본: 현재 위치 기준 가장 가까운 waypoint -> 진열대 앞 waypoint -> 해당 각도 바라보고 정지
        - 단, 바로 직전에 add 를 실행했다면:
          Nav2 없이 먼저 10cm 전진한 뒤, 그 위치를 기준으로 경로 계획
        """
        cmd = raw_command.lower().replace(" ", "")
        if "shelf" not in cmd:
            self.get_logger().error(f"shelf 명령 파싱 실패: {raw_command}")
            return

        shelf_id = cmd.replace("shelf", "")
        if not shelf_id.isdigit():
            self.get_logger().error(f"shelf 명령 파싱 실패: {raw_command}")
            return

        # 진열대 번호 1~18만 허용
        if shelf_id not in SHELF_TO_WP:
            self.get_logger().warn(
                f"{shelf_id}번은 정의된 진열대가 아닙니다 (1~18만 허용)."
            )
            return

        # --- add 이후 첫 shelf 이면: Nav2 없이 10cm 전진 ---
        if self.post_add_shelf_forward:
            self.get_logger().info(
                "[SHELF] add 이후 첫 shelf 명령 감지 → Nav2 없이 먼저 10cm 전진 후 경로 계획"
            )
            # 0.10 m/s 로 1초 → 10cm
            self._publish_twist_for(linear_x=0.10, angular_z=0.0, duration=1.0)
            # AMCL 위치가 약간 안정될 시간을 줌
            time.sleep(0.5)
            # 한 번 사용했으니 플래그 해제
            self.post_add_shelf_forward = False

        target_wp = SHELF_TO_WP[shelf_id]

        start = self.find_nearest_waypoint()
        if start is None:
            self.get_logger().error("현재 위치 기준 시작 waypoint를 찾을 수 없습니다.")
            return

        self.current_goal_target = target_wp    # 거리 계산은 waypoint 기준
        self.current_state = "NAVIGATING"
        self.replan_attempts = 0
        self.last_goal_command = f"shelf{shelf_id}"

        self.get_logger().info(
            f"'shelf {shelf_id}' 명령 수신: 현재({start}) -> "
            f"진열대 {shelf_id} 앞 waypoint {target_wp} 경로 탐색"
        )

        path_ids = self.find_shortest_path(start, target_wp)
        if not path_ids:
            self.get_logger().error(
                f"shelf 경로를 찾을 수 없습니다: {start} -> {target_wp}"
            )
            self.current_state = "IDLE"
            return

        self.get_logger().info(
            f"shelf {shelf_id} 경로: {' -> '.join(path_ids)} "
            f"(마지막: waypoint {target_wp})"
        )
        # 위치는 target_wp, 방향은 shelf_id 기준으로 보정 + 중간 waypoint는 경로 방향 yaw
        self.send_shelf_path(path_ids, goal_wp_id=target_wp, shelf_id=shelf_id)

    def handle_add_command(self):
        """
        'add' 명령:
        1) Nav2 주행 취소
        2) 앞으로 10cm 이동
        3) 3초 정지
        4) 제자리에서 180도 회전
        5) 다음에 오는 첫 shelf 명령에서 추가로 10cm 전진 후 Nav2 경로 주행
        """
        self.get_logger().info(
            "'add' 명령 수신: Nav2 취소 → 10cm 전진 → 3초 대기 → 제자리 180도 회전"
        )

        # Nav2 주행 취소 (있으면)
        self.cancel_navigation()

        # 내부 상태 깔끔하게 초기화
        self.current_state = "IDLE"
        self.current_path_ids = []
        self.current_goal_target = None
        self.replan_attempts = 0
        self.last_goal_command = "add"

        # 앞으로 10cm 이동 (0.10 m/s로 1초)
        self.get_logger().info("[ADD] 앞으로 10cm 이동")
        self._publish_twist_for(linear_x=0.10, angular_z=0.0, duration=1.0)

        # 3초 정지
        self.get_logger().info("[ADD] 3초 대기")
        time.sleep(3.0)
        self.cmd_vel_publisher.publish(Twist())  # 한번 더 정지 신호

        # 제자리에서 180도 회전 (0.5 rad/s로 pi/0.5초)
        self.get_logger().info("[ADD] 제자리 180도 회전")
        turn_speed = 0.5               # rad/s
        turn_duration = math.pi / turn_speed
        self._publish_twist_for(linear_x=0.0, angular_z=turn_speed, duration=turn_duration)

        # 다음 shelf 명령에서 10cm 추가 전진하도록 플래그 세팅
        self.post_add_shelf_forward = True

        self.get_logger().info("[ADD] 동작 완료, 다음 go/shelf 명령 대기")

    # ------------------------------------------------------------------
    #  Nav2 취소 / Pause
    # ------------------------------------------------------------------
    def pause_navigation(self):
        self.get_logger().info("pause 명령 수신: Nav2 주행 취소 시도")
        if not self.cancel_navigation():
            self.get_logger().warn("취소할 Nav2 주행이 없습니다.")

    def cancel_navigation(self):
        if self.current_goal_handle is None:
            self.current_state = "IDLE"
            return False

        try:
            self.current_goal_handle.cancel_goal_async()
            self.get_logger().info(">>> Nav2 주행 취소 요청 전송")
        except Exception as e:
            self.get_logger().error(f"Nav2 취소 중 오류: {e}")
            return False

        self.current_state = "IDLE"
        self.current_path_ids = []
        self.current_goal_target = None
        self.replan_attempts = 0
        return True

    def _publish_twist_for(self, linear_x=0.0, angular_z=0.0, duration=0.0):
        """
        지정된 시간(duration 초) 동안 주어진 속도(linear_x, angular_z)로
        /cmd_vel 을 계속 발행하는 간단한 헬퍼.
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        end_time = time.time() + duration
        period = 0.05  # 20Hz

        while time.time() < end_time and rclpy.ok():
            self.cmd_vel_publisher.publish(twist)
            time.sleep(period)

        # 정지
        stop = Twist()
        self.cmd_vel_publisher.publish(stop)

    # ------------------------------------------------------------------
    #  Nav2 목표 전송 / 콜백
    # ------------------------------------------------------------------
    def send_single_goal(self, x, y, theta_rad=None):
        if not self._action_single_goal_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 액션 서버(/navigate_to_pose)를 찾을 수 없습니다.")
            self.current_state = "IDLE"
            return

        # theta를 안 넘겼으면, 현재 위치 → 목표까지 방향으로 yaw 계산
        if theta_rad is None and self.current_robot_pose is not None:
            rx = self.current_robot_pose.position.x
            ry = self.current_robot_pose.position.y
            theta_rad = math.atan2(y - ry, x - rx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, theta_rad)

        self.get_logger().info(
            f"Nav2에 단일 목표 전송: x={x:.2f}, y={y:.2f}, "
            f"theta={'auto' if theta_rad is None else f'{theta_rad:.2f}'} rad"
        )

        self.reset_progress_monitor()
        send_goal_future = self._action_single_goal_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_handle_callback)

    def send_waypoint_path(self, path_ids, goal_id=None, from_replan=False):
        if not self._action_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 액션 서버(/navigate_through_poses)를 찾을 수 없습니다.")
            self.current_state = "IDLE"
            return

        self.current_path_ids = list(path_ids)
        if goal_id is not None:
            self.current_goal_target = goal_id

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for i, wp_id in enumerate(path_ids):
            x, y = self._xy_from_wp(wp_id)
            if x is None:
                continue

            theta = None

            # 다음 waypoint가 있다면: 현재 → 다음으로 가는 방향
            if i < len(path_ids) - 1:
                nx, ny = self._xy_from_wp(path_ids[i + 1])
                if nx is not None:
                    theta = math.atan2(ny - y, nx - x)
            # 마지막 waypoint인데, 앞에 하나라도 있으면: 이전 → 현재 방향
            elif i > 0:
                px, py = self._xy_from_wp(path_ids[i - 1])
                if px is not None:
                    theta = math.atan2(y - py, x - px)

            goal_msg.poses.append(self.create_pose_stamped(x, y, theta_rad=theta))

        if not goal_msg.poses:
            self.get_logger().error("경유지 목록이 비어있습니다.")
            self.current_state = "IDLE"
            return

        tag = "재계획 경로" if from_replan else "원래 경로"
        self.get_logger().info(
            f"Nav2에 {len(goal_msg.poses)}개 경유지 전송 ({tag}: {' -> '.join(path_ids)})"
        )

        self.reset_progress_monitor()
        send_goal_future = self._action_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_handle_callback)

    def send_shelf_path(self, path_ids, goal_wp_id, shelf_id):
        """shelf 명령 전용: 마지막 포즈만 shelf_facing_theta 방향으로 (중간은 경로 방향 yaw)"""
        if not self._action_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 액션 서버(/navigate_through_poses)를 찾을 수 없습니다.")
            self.current_state = "IDLE"
            return

        self.current_path_ids = list(path_ids)
        self.current_goal_target = goal_wp_id  # 거리 계산용

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for i, wp_id in enumerate(path_ids):
            x, y = self._xy_from_wp(wp_id)
            if x is None:
                continue

            # 기본은 경로 방향 yaw
            theta = None
            if i < len(path_ids) - 1:
                nx, ny = self._xy_from_wp(path_ids[i + 1])
                if nx is not None:
                    theta = math.atan2(ny - y, nx - x)
            elif i > 0:
                px, py = self._xy_from_wp(path_ids[i - 1])
                if px is not None:
                    theta = math.atan2(y - py, x - px)

            # 마지막 포즈는 shelf_facing_theta로 override
            if i == len(path_ids) - 1:
                theta = SHELF_FACING_THETA.get(str(shelf_id), 0.0)

            goal_msg.poses.append(self.create_pose_stamped(x, y, theta_rad=theta))

        if not goal_msg.poses:
            self.get_logger().error("shelf 경유지 목록이 비어있습니다.")
            self.current_state = "IDLE"
            return

        self.get_logger().info(
            f"Nav2에 {len(goal_msg.poses)}개 경유지 전송 "
            f"(shelf {shelf_id}: {' -> '.join(path_ids)}, 목표 waypoint={goal_wp_id})"
        )

        self.reset_progress_monitor()
        send_goal_future = self._action_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_handle_callback)

    def goal_handle_callback(self, future):
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error("목표 핸들을 받지 못했습니다.")
            self.current_state = "IDLE"
            return

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다 (Goal rejected)")
            self.current_state = "IDLE"
            return

        self.get_logger().info("목표가 수락되었습니다 (Goal accepted)")
        self.current_goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f, gh=goal_handle: self.get_result_callback(f, gh)
        )

    def get_result_callback(self, future, goal_handle):
        if goal_handle is not self.current_goal_handle:
            self.get_logger().info("[OLD Nav2 RESULT] 이전 goal에 대한 결과 수신 – 무시합니다.")
            return

        status = future.result().status

        self.get_logger().info(
            f"[Nav2 RESULT RAW] status={status}, result_msg={future.result().result}"
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표 지점 도착 성공!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("목표가 취소되었습니다 (STATUS_CANCELED)")
        else:
            self.get_logger().error(f"목표 지점 도착 실패! (Status: {status})")

        self.current_state = "IDLE"
        self.current_goal_handle = None
        self.last_progress_time = None
        self.last_progress_distance = None

    # ------------------------------------------------------------------
    #  PoseStamped 생성 헬퍼
    # ------------------------------------------------------------------
    def create_pose_stamped(self, x, y, theta_rad=None):
        """
        PoseStamped 생성
        - theta_rad가 주어지면: 해당 각도로 쿼터니언 생성
        - theta_rad가 None이면: 0(rad)로 둠
        (실제 사용에서는 send_* 쪽에서 웬만하면 방향을 계산해서 넘겨줌)
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        if theta_rad is None:
            theta_rad = 0.0

        pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        pose.pose.orientation.w = math.cos(theta_rad / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

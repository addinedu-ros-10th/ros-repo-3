import math
from collections import deque
from geometry_msgs.msg import PoseStamped
from .waypoint_dict import *

CAM_H = 540
CAM_W = 960


def calculate_cam_angle(posx, posy, distance, cam_id):
    center = 0
    if (cam_id == 1):
        center = 0.0
    elif (cam_id == 2):
        center = -math.pi
    else:
        return -1
    
    relative_x = posx - (CAM_W/2)
    relative_y = posy - (CAM_H/2)
    
    angle = math.asin(relative_x/distance)
    return angle

def find_nearest_waypoint(posx, posy):

    designated_x = posx
    designated_y = posy

    min_dist = float('inf')
    nearest_wp = None
    for wp_id in WAYPOINTS.keys():
        wp_x, wp_y = xy_from_wp(wp_id)
        if wp_x is None:
            continue
        dist = math.hypot(designated_x - wp_x, designated_y - wp_y)
        if dist < min_dist:
            min_dist = dist
            nearest_wp = wp_id

    return nearest_wp

def xy_from_wp(wp_id):
    """waypoints 항목에서 x,y만 안전하게 추출"""
    if wp_id not in WAYPOINTS:
        return None, None
    data = WAYPOINTS[wp_id]
    if len(data) >= 2:
        return float(data[0]), float(data[1])
    return None, None

def find_shortest_path(start, end, blocked_nodes=None):

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

def create_pose_stamped(x, y, theta_rad, current_time):
    """
    PoseStamped 생성
    - theta_rad가 주어지면: 해당 각도로 쿼터니언 생성
    - theta_rad가 None이면: 0(rad)로 둠
    (실제 사용에서는 send_* 쪽에서 웬만하면 방향을 계산해서 넘겨줌)
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = current_time.to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    if theta_rad is None:
        theta_rad = 0.0

    pose.pose.orientation.z = math.sin(theta_rad / 2.0)
    pose.pose.orientation.w = math.cos(theta_rad / 2.0)
    return pose



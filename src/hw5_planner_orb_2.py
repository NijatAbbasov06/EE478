#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from scipy.spatial.transform import Rotation
import numpy as np

# apriltag_ros 패키지에서 메시지 타입 가져오기
from apriltag_ros.msg import AprilTagDetectionArray
class QuaternionFilter:
    def __init__(self):
        self.prev_quat = None
    
    def ensure_continuity(self, quat):
        if self.prev_quat is None:
            self.prev_quat = quat
            return quat
        
        # Check both representations
        quat_pos = np.array(quat)
        quat_neg = -quat_pos
        
        # Calculate distances to previous quaternion
        dist_pos = np.linalg.norm(quat_pos - self.prev_quat)
        dist_neg = np.linalg.norm(quat_neg - self.prev_quat)
        
        # Choose the closer one
        if dist_neg < dist_pos:
            result_quat = quat_neg
        else:
            result_quat = quat_pos
            
        self.prev_quat = result_quat
        return result_quat
    
class EulerFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha  # Smoothing factor
        self.prev_euler = None
    
    def smooth_euler(self, euler):
        if self.prev_euler is None:
            self.prev_euler = euler
            return euler
        
        # Handle angle wrapping (e.g., 179° to -179° should be smooth)
        diff = euler - self.prev_euler
        
        # Wrap differences to [-180, 180]
        diff = ((diff + 180) % 360) - 180
        
        # Smooth the change
        smoothed = self.prev_euler + self.alpha * diff
        
        self.prev_euler = smoothed
        return smoothed

class GlobalPlanner:
    def __init__(self):
        rospy.init_node("global_planner")

        # --------------------------
        # 1) Lookahead 파라미터
        # --------------------------
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.0)

        # --------------------------
        # 2) Gate spawn 정의
        # --------------------------
        # name, x_spawn, y_spawn, z_spawn, yaw_deg
        self.gate_defs = [
            ("gate_pair_00",  5.0, -4.0, 0.0,  -45),
            ("gate_pair_01", 15.0, -4.0, 0.0,   45),
            ("gate_pair_02", 15.0,  4.0, 0.0,  135),
            ("gate_pair_03",  5.0,  4.0, 0.0, -135),
        ]
        self.ALTITUDE = 1.00
        self.euler_filter = EulerFilter(alpha=0.2)
        self.quat_filter = QuaternionFilter()

        # --------------------------
        # 3) 전역 웨이포인트 초기화
        # --------------------------
        self.global_waypoints = []
        self.decision_index = 0

        # --------------------------
        # 4) 상태 변수 초기화
        # --------------------------
        self.global_index = 0
        # ORB 기준 카메라의 월드(world) 위치를 저장할 변수
        self.camera_world_pos = None       # type: Point or None
        # Apriltag 검출 시 “카메라 기준 태그 상대 위치”를 저장할 변수
        self.tag_in_cam_pos = None         # type: Point or None
        # 실제 드론(카메라)의 월드 위치 = camera_world_pos + tag_in_cam_pos
        self.cur_position = None           # type: Point or None

        # 로컬 경로
        self.path = None  # list of PoseStamped

        # --------------------------
        # 5) ROS 인터페이스 설정
        # --------------------------
        # A) GPT에게서 LEFT/RIGHT 결정받기
        rospy.Subscriber("/gpt_decision", String, self.decision_callback)

        # B) 로컬 패스를 받아올 Path 구독
        self.path_sub = rospy.Subscriber("/waypoint_generator/waypoints", Path, self.path_callback)

        # C1) 태그 검출 정보 구독 → ApriltagDetectionArray 타입
        rospy.Subscriber("/april_pose", PoseStamped, self.tag_callback)

        # C2) ORB-SLAM3가 뿌려주는 카메라 포즈(월드 기준)를 구독
        rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.orb_callback)

        # D) 퍼블리셔: lookahead(Point)과 글로벌 목표(PoseStamped)
        self.local_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.global_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.arrival_threshold = 0.8  # 글로벌 웨이포인트 도착 임계거리

    # =====================================
    # A) /gpt_decision 콜백: LEFT/RIGHT 처리
    # =====================================
    def decision_callback(self, msg: String):
        if self.decision_index >= len(self.gate_defs):
            return

        decision = msg.data.strip().upper()
        name, x_spawn, y_spawn, z_spawn, yaw_deg = self.gate_defs[self.decision_index]
        theta = math.radians(yaw_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        if decision == "LEFT":
            x_chosen = x_spawn + 1.0 * sin_t
            y_chosen = y_spawn + 1.0 * cos_t
        elif decision == "RIGHT":
            x_chosen = x_spawn - 1.0 * sin_t
            y_chosen = y_spawn - 1.0 * cos_t
        else:
            rospy.logwarn("[Decision] '%s' 은 LEFT/RIGHT 중 하나가 아닙니다. 무시합니다.", msg.data)
            return

        z_chosen = self.ALTITUDE
        pt = Point(x=x_chosen, y=y_chosen, z=z_chosen)
        self.global_waypoints.append(pt)
        rospy.loginfo(
            "[Decision] Gate %d (%s) => %s 로 설정: x=%.3f, y=%.3f, z=%.3f",
            self.decision_index, name, decision, x_chosen, y_chosen, z_chosen,
        )
        self.decision_index += 1

        # 4개 게이트를 모두 결정하면 마지막에 home 포인트 추가
        if self.decision_index == len(self.gate_defs):
            home_pt = Point(x=0.0, y=0.0, z=0.0)
            self.global_waypoints.append(home_pt)
            rospy.loginfo("[Decision] 모든 게이트 결정 완료. Home 포인트 추가됨.")

    # =====================================
    # B) /waypoint_generator/waypoints 콜백: Path 수신
    # =====================================
    def path_callback(self, msg: Path):
        # msg.poses는 PoseStamped 리스트
        self.path = msg.poses
        # 새로 현재 위치가 업데이트되어 있을 수 있으니 다시 로컬 lookahead 계산
        self.publish_lookahead_point()

    # =====================================
    # C1) /tag_detections 콜백: Apriltag 검출 시 처리
    # =====================================
    def tag_callback(self, msg: PoseStamped):
        # detections 리스트가 비어있으면 무시
        

        # 예시로 첫 번째 태그 정보만 사용
        
        # detection.pose 는 PoseWithCovarianceStamped 타입의 pose 필드를 포함
             # type: PoseWithCovarianceStamped

        # pose_stamped.pose.pose.position는 “태그가 카메라 프레임에서 얼마나 떨어져 있는지(상대 위치)”
        rel = msg.pose.pose.position
        self.tag_in_cam_pos = Point(x=rel.x, y=rel.y, z=rel.z)

        # 태그가 검출될 때마다 현재 위치 업데이트 시도
        self.update_current_position()
    # =====================================
    # C2) /orb_slam3/camera_pose 콜백: 카메라 월드 위치 수신
    # =====================================
    def orb_callback(self, msg: PoseStamped):
        # msg.pose.position은 카메라가 월드(world) 좌표계에서 가진 절대 위치
        cam = msg.pose.position
        self.camera_world_pos = Point(x=cam.x, y=cam.y, z=cam.z)

        # ORB 쪽이 들어올 때마다 현재 위치 업데이트 시도
        self.update_current_position()

    # =====================================
    # “카메라 월드 위치”와 “태그 상대 위치”를 합쳐서 current_position 산출
    # =====================================
    def update_current_position(self):
        """
        - camera_world_pos가 None인 경우 → 진행 불가(위치를 모름)
        - tag_in_cam_pos가 None인 경우 → 단순히 camera_world_pos만 current_position으로 사용
        - 둘 다 None이 아니면 → 합산해서 current_position으로 사용
            (x = cam.x + tag.x, y = cam.y + tag.y, z = cam.z + tag.z)
        """
        if self.camera_world_pos is None:
            # ORB 카메라 정보조차 없으면 현 위치를 알 수 없음
            self.cur_position = None
            return

        if self.tag_in_cam_pos is None:
            # 태그 미검출 : 카메라 월드 위치만 current_position으로 사용
            self.cur_position = Point(
                x=self.camera_world_pos.x,
                y=self.camera_world_pos.y,
                z=self.camera_world_pos.z,
            )
        else:
            # 태그 검출 시: 카메라 월드 위치 + 태그 상대 위치(카메라 기준) = 실제 태그 월드 위치
            # (예시: “카메라 월드 좌표” + “카메라→태그 벡터” 합산)
            self.cur_position = Point(
                x=self.camera_world_pos.x + self.tag_in_cam_pos.x,
                y=self.camera_world_pos.y + self.tag_in_cam_pos.y,
                z=self.camera_world_pos.z + self.tag_in_cam_pos.z,
            )

        # current_position이 갱신되었으므로 로컬/글로벌 웨이포인트 로직 다시 시도
        self.publish_lookahead_point()
        self.check_and_publish_global()

    # =====================================
    # D) Lookahead Point 게시
    # =====================================
    def publish_lookahead_point(self):
        if self.cur_position is None or self.path is None:
            # 위치나 경로가 없으면 아무것도 하지 않음
            return

        # path는 PoseStamped 리스트 → 각 PoseStamped.pose.position을 돌면서,
        # 현재 위치에서 lookahead_distance 이상 차이나는 첫 번째 지점 발행
        for pose_stamped in self.path:
            pt = pose_stamped.pose.position
            dist = self.euclidean_distance(pt, self.cur_position)
            if dist >= self.lookahead_distance:
                waypoint = PoseStamped()
                waypoint.header.stamp = rospy.Time.now()
                waypoint.header.frame_id = "odom"
                waypoint.pose.position = pt
                waypoint.pose.orientation = pose_stamped.pose.orientation
                self.local_pub.publish(waypoint)
                rospy.loginfo(
                    "[Local] Lookahead published: x=%.2f, y=%.2f, z=%.2f",
                    pt.x, pt.y, pt.z,
                )
                return

        rospy.logwarn("[Local] No lookahead point found (lookahead_distance=%f)", self.lookahead_distance)

    # =====================================
    # E) Global Waypoint 게시
    # =====================================
    def check_and_publish_global(self):
        # 현재 위치가 없거나(global_index가 범위 초과)이면 리턴
        if self.cur_position is None or self.global_index >= len(self.global_waypoints):
            return

        goal = self.global_waypoints[self.global_index]
        dist = self.euclidean_distance(goal, self.cur_position)

        if dist < self.arrival_threshold:
            rospy.loginfo("Arrived at global waypoint %d", self.global_index)
            self.global_index += 1
            if self.global_index >= len(self.global_waypoints):
                rospy.loginfo("All global waypoints reached")
                return
            goal = self.global_waypoints[self.global_index]

        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = goal
        # orientation을 단위(quat = [0,0,0,1])로 설정
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0
        self.global_pub.publish(waypoint)
        rospy.loginfo(
            "[Global] Waypoint %d published: x=%.2f, y=%.2f, z=%.2f",
            self.global_index, goal.x, goal.y, goal.z,
        )

    
    # =====================================
    # F) 유클리디안 거리 계산
    # =====================================
    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2
            + (p1.y - p2.y) ** 2
            + (p1.z - p2.z) ** 2
        )
    def quaternion_to_matrix(x, y, z, w):
        r = Rotation.from_quat([x, y, z, w])
        return r.as_matrix()



if __name__ == "__main__":
    planner = GlobalPlanner()
    rospy.spin()


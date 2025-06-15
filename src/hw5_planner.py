#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point


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
        self.ALTITUDE = 1.05

        # --------------------------
        # 3) 전역 웨이포인트 초기화
        # --------------------------
        #   - 아직 /gpt_decision 을 받지 않았으므로 빈 리스트
        #   - 순서대로 4개 페어에 대해 LEFT/RIGHT 를 받아서 채워질 예정
        self.global_waypoints = []
        # gate 0~3 순서대로 LEFT/RIGHT 결정: 현재 몇 번째 gate 페어 결정 중인지 인덱스
        self.decision_index = 0

        # --------------------------
        # 4) 상태 변수 초기화
        # --------------------------
        self.global_index = 0
        self.cur_position = None
        self.path = None

        # --------------------------
        # 5) ROS 인터페이스 설정
        # --------------------------
        # - /gpt_decision: String ("LEFT" 또는 "RIGHT")
        rospy.Subscriber("/gpt_decision", String, self.decision_callback)

        # - 로컬 패스(lookahead) 를 위한 Path 구독
#        self.path_sub = rospy.Subscriber(
#            "/waypoint_generator/waypoints", Path, self.path_callback
#        )
        
        self.path_sub = rospy.Subscriber(
            "/planning/bspline_path", Path, self.path_callback
        )        
                
        # - 현재 드론 위치
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_callback
        )

#        self.pose_sub = rospy.Subscriber(
#            "/april_pose", PoseStamped, self.pose_callback
#        )



        # - 퍼블리셔: 로컬 lookahead / 글로벌 목표점
        self.local_pub = rospy.Publisher(
            "/lookahead_waypoint", PoseStamped, queue_size=1
        )
        self.global_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )

        self.arrival_threshold = 0.8  # 글로벌 웨이포인트 도착 임계거리

    # =====================================
    # A) /gpt_decision 콜백: LEFT/RIGHT 처리
    # =====================================
    def decision_callback(self, msg: String):
        """
        msg.data 는 "LEFT" 또는 "RIGHT" (문자열)
        순서대로 gate_defs[self.decision_index] 를 사용해 좌표 계산 후 self.global_waypoints 에 추가
        """
        # 1) 만약 이미 4개 게이트 결정을 다 받았다면 무시
        if self.decision_index >= len(self.gate_defs):
            return

        decision = msg.data.strip().upper()  # "LEFT" 또는 "RIGHT"
        name, x_spawn, y_spawn, z_spawn, yaw_deg = self.gate_defs[self.decision_index]

        # yaw를 radian 으로 변환
        theta = math.radians(yaw_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # LEFT/RIGHT 에 따라 (x, y) 선택
        if decision == "LEFT":
            # left gate center
            # (원래 예제는 x_spawn + 1⋅sinθ, y_spawn + 1⋅cosθ 로 계산)
            x_chosen = x_spawn + 1.0 * sin_t
            y_chosen = y_spawn + 1.0 * cos_t
        elif decision == "RIGHT":
            # right gate center
            x_chosen = x_spawn - 1.0 * sin_t
            y_chosen = y_spawn - 1.0 * cos_t
        else:
            rospy.logwarn("[Decision] '%s' 은 LEFT/RIGHT 중 하나가 아닙니다. 무시합니다.", msg.data)
            return

        z_chosen = self.ALTITUDE

        # Point 메시지 생성
        pt = Point()
        pt.x = x_chosen
        pt.y = y_chosen
        pt.z = z_chosen

        # 리스트에 추가
        self.global_waypoints.append(pt)
        rospy.loginfo(
            "[Decision] Gate %d (%s) => %s 로 설정: x=%.3f, y=%.3f, z=%.3f",
            self.decision_index,
            name,
            decision,
            x_chosen,
            y_chosen,
            z_chosen,
        )

        # 다음 게이트 인덱스 증가
        self.decision_index += 1

        # 4개 게이트 + 홈 포인트: 모두 결정되면 마지막에 홈 포인트 추가
        if self.decision_index == len(self.gate_defs):
            home_pt = Point()
            home_pt.x = 0.0
            home_pt.y = 0.0
            home_pt.z = 0.0
            self.global_waypoints.append(home_pt)
            rospy.loginfo("[Decision] 모든 게이트 결정 완료. Home 포인트 추가됨.")

    # =====================================
    # B) 현재 Pose 와 패스 콜백
    # =====================================
    def pose_callback(self, msg: PoseStamped):
        self.cur_position = msg.pose.position
        self.publish_lookahead_point()
        self.check_and_publish_global()

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.publish_lookahead_point()

    # =====================================
    # C) Lookahead Point 게시 (기존 로직)
    # =====================================
    def publish_lookahead_point(self):
        if self.cur_position is None or self.path is None:
            return

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
                    "[Local] Lookahead published: x=%.2f y=%.2f z=%.2f",
                    pt.x,
                    pt.y,
                    pt.z,
                )
                return

        rospy.logwarn("[Local] No lookahead point found")

    # =====================================
    # D) Global Waypoint 게시 (수정 없음)
    # =====================================
    def check_and_publish_global(self):
        # 아직 드론의 위치를 모른다거나, waypoints 가 비어있으면 return
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

        # Publish 다음 global waypoint
        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = goal
        waypoint.pose.orientation.w = 1.0  # orientation 은 임시로 unit quaternion 사용
        self.global_pub.publish(waypoint)
        rospy.loginfo(
            "[Global] Waypoint %d published: x=%.2f y=%.2f z=%.2f",
            self.global_index,
            goal.x,
            goal.y,
            goal.z,
        )

    # =====================================
    # E) 유클리디안 거리 계산
    # =====================================
    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2
            + (p1.y - p2.y) ** 2
            + (p1.z - p2.z) ** 2
        )


if __name__ == "__main__":
    planner = GlobalPlanner()
    rospy.spin()
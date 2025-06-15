#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs 
import tf  


class GoalSelector:
    def __init__(self):
        rospy.init_node("goal_selector_node")

        self.gate_pairs = [
            { "left":  [0,  1],  "right": [2,  3]   },
            { "left":  [4,  5],  "right": [6,  7]   },
            { "left":  [8,  9],  "right": [10, 11]  },
            { "left":  [12, 13], "right": [14, 15]  },
        ]

        self.detected_tags = {}
        self.current_pose = None
        self.last_decision = ""

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )

        rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, self.tag_callback
        )
        rospy.Subscriber(
            "/gpt_decision", String, self.decision_callback
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.local_pose_callback
        )

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("[GoalSelector] Node initialized.")


    def tag_callback(self, msg: AprilTagDetectionArray):
        self.detected_tags.clear()
        for det in msg.detections:
            if len(det.id) == 0:
                continue
            tag_id = det.id[0]
            cam_pose = PoseStamped()
            cam_pose.header = det.pose.header
            cam_pose.pose = det.pose.pose.pose
            self.detected_tags[tag_id] = cam_pose


    def decision_callback(self, msg: String):
        decision = msg.data.strip().upper()
        if decision not in ["LEFT", "RIGHT"]:
            rospy.logwarn(f"[GoalSelector] Unknown decision received: {decision}")
            return
        self.last_decision = decision
        rospy.loginfo(f"[GoalSelector] Received GPT decision: {decision}")


    def local_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg


    def timer_cb(self, event):
        if self.last_decision not in ["LEFT", "RIGHT"]:
            return

        if self.current_pose is None:
            return

        side = "left" if self.last_decision == "LEFT" else "right"

        candidates = []

        for idx, pair in enumerate(self.gate_pairs):
            tag_ids = pair[side]
            detected_world_poses = []
            for tid in tag_ids:
                if tid in self.detected_tags:
                    cam_pose = self.detected_tags[tid]
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            "odom",
                            cam_pose.header.frame_id,
                            rospy.Time(0),
                            rospy.Duration(0.05)
                        )
                        world_pose = tf2_geometry_msgs.do_transform_pose(
                            cam_pose, transform
                        )
                        detected_world_poses.append(world_pose)
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ExtrapolationException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.InvalidArgumentException
                    ) as e:
                        rospy.logwarn(f"[GoalSelector] TF transform failed for tag {tid}: {e}")
                        continue

            if len(detected_world_poses) > 0:
                candidates.append((idx, detected_world_poses))

        if len(candidates) == 0:
            return

        min_dist = float("inf")
        best_pair_index = None
        best_centroid_pose = None

        cur_x = self.current_pose.pose.position.x
        cur_y = self.current_pose.pose.position.y

        for (pair_idx, world_pose_list) in candidates:
            sum_x = 0.0
            sum_y = 0.0
            sum_z = 0.0
            for p in world_pose_list:
                sum_x += p.pose.position.x
                sum_y += p.pose.position.y
                sum_z += p.pose.position.z
            cnt = float(len(world_pose_list))

            avg_x = sum_x / cnt
            avg_y = sum_y / cnt
            avg_z = sum_z / cnt

            dist = math.hypot(avg_x - cur_x, avg_y - cur_y)

            if dist < min_dist:
                min_dist = dist
                best_pair_index = pair_idx

                centroid = PoseStamped()
                centroid.header.stamp = rospy.Time.now()
                centroid.header.frame_id = "odom"
                centroid.pose.position.x = avg_x
                centroid.pose.position.y = avg_y
                centroid.pose.position.z = avg_z

                centroid.pose.orientation = world_pose_list[0].pose.orientation
                best_centroid_pose = centroid

        if best_centroid_pose is not None:
            self.goal_pub.publish(best_centroid_pose)
            rospy.loginfo(
                f"[GoalSelector] Publishing goal for gate_pair_{best_pair_index} "
                f"({side} side), centroid=({best_centroid_pose.pose.position.x:.2f}, "
                f"{best_centroid_pose.pose.position.y:.2f}, {best_centroid_pose.pose.position.z:.2f})"
            )

            self.last_decision = ""


if __name__ == "__main__":
    try:
        gs = GoalSelector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
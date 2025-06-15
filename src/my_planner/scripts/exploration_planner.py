#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import random

class TreeAvoidanceExplorer:
    def __init__(self):
        rospy.init_node("tree_avoidance_explorer")

        self.safety_distance = rospy.get_param("~safety_distance", 1.5)
        self.exploration_radius = rospy.get_param("~exploration_radius", 10.0)
        self.tree_height_min = rospy.get_param("~tree_height_min", 0.5)
        self.tree_height_max = rospy.get_param("~tree_height_max", 3.0)
        self.z_threshold = rospy.get_param("~z_threshold", 0.5)

        self.current_pose = None
        self.tree_points = []
        self.exploration_points = []
        self.current_goal = None
        self.exploration_state = "FIND_TREE"
        self.tree_center = None
        self.tree_radius = 0
        self.circle_angle = 0
        self.visited_locations = set()
        self.location_resolution = 0.25

        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        self.pointcloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud, self.pointcloud_callback)
        
        self.goal_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.vis_pub = rospy.Publisher("/tree_explorer/visualization", MarkerArray, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.5), self.control_loop)
        
        rospy.loginfo("Tree Avoidance Explorer initialized!")

    def pose_callback(self, msg):
        self.current_pose = msg
        
        if self.current_pose:
            grid_x = int(self.current_pose.pose.position.x / self.location_resolution)
            grid_y = int(self.current_pose.pose.position.y / self.location_resolution)
            self.visited_locations.add((grid_x, grid_y))

    def pointcloud_callback(self, msg):
        if not self.current_pose:
            return
            
        
        self.tree_points = []
        
        for i, point in enumerate(msg.points):
            if point.z * 2 < self.z_threshold:
                continue
                
            self.tree_points.append((point.x, point.y, point.z))
        
        if len(self.tree_points) > 10:
            self.detect_obstacle()
    
        self.visualize_data()

    def detect_obstacle(self):
        x_sum = 0
        y_sum = 0
        z_sum = 0
        for x, y, z in self.tree_points:
            x_sum += x
            y_sum += y
            z_sum += z
        
        center_x = x_sum / len(self.tree_points)
        center_y = y_sum / len(self.tree_points)
        center_z = z_sum / len(self.tree_points)
        
        max_radius = 0
        max_height = 0
        min_height = float('inf')
        
        for x, y, z in self.tree_points:
            dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            if dist > max_radius:
                max_radius = dist
                
            if z > max_height:
                max_height = z
            if z < min_height:
                min_height = z
        
        self.tree_center = Point(center_x, center_y, center_z)
        self.tree_radius = max_radius * 1.2
        
        self.obstacle_height = max(max_height - min_height, 1.0)
        
        rospy.loginfo(f"Obstacle detected at ({center_x:.2f}, {center_y:.2f}) with radius {self.tree_radius:.2f}m")
    
    def control_loop(self, event):
        if not self.current_pose or not self.tree_center:
            return
            
        current_pos = self.current_pose.pose.position
        
        if self.exploration_state == "FIND_TREE":
            if self.tree_center:
                rospy.loginfo("Tree found! Switching to CIRCLE_TREE state")
                self.exploration_state = "CIRCLE_TREE"
        
        elif self.exploration_state == "CIRCLE_TREE":
            tree_distance = self.tree_radius + self.safety_distance
            
            self.circle_angle += 0.2
            if self.circle_angle >= 2 * math.pi:
                rospy.loginfo("Completed tree circle! Switching to EXPLORE state")
                self.exploration_state = "EXPLORE"
                self.circle_angle = 0
            
            next_x = self.tree_center.x + tree_distance * math.cos(self.circle_angle)
            next_y = self.tree_center.y + tree_distance * math.sin(self.circle_angle)
            
            goal = Point(next_x, next_y, current_pos.z)
            self.publish_goal(goal)
        
        elif self.exploration_state == "EXPLORE":
            if not self.current_goal or self.distance(current_pos, self.current_goal) < 0.5:
                self.current_goal = self.find_exploration_point()
                self.publish_goal(self.current_goal)

    def find_exploration_point(self):
        if not self.current_pose or not self.tree_center:
            return None
            
        current_pos = self.current_pose.pose.position
        
        for _ in range(30):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(2.0, self.exploration_radius)
            
            goal_x = current_pos.x + distance * math.cos(angle)
            goal_y = current_pos.y + distance * math.sin(angle)
            
            if self.is_clear_of_tree(goal_x, goal_y):
                grid_x = int(goal_x / self.location_resolution)
                grid_y = int(goal_y / self.location_resolution)
                
                if (grid_x, grid_y) not in self.visited_locations:
                    return Point(goal_x, goal_y, current_pos.z)
        
        angle = random.uniform(0, 2 * math.pi)
        dist = self.tree_radius + self.safety_distance * 2
        goal_x = self.tree_center.x + dist * math.cos(angle)
        goal_y = self.tree_center.y + dist * math.sin(angle)
        
        return Point(goal_x, goal_y, current_pos.z)

    def is_clear_of_tree(self, x, y):
        if not self.tree_center:
            return True
            
        dist_to_tree = math.sqrt((x - self.tree_center.x)**2 + (y - self.tree_center.y)**2)
        
        return dist_to_tree > (self.tree_radius + self.safety_distance)

    def publish_goal(self, point):
        if not point:
            return
            
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Published new goal: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})")

    def visualize_data(self):
        marker_array = MarkerArray()
        
        if self.tree_center:
            center_marker = Marker()
            center_marker.header.frame_id = "map"
            center_marker.header.stamp = rospy.Time.now()
            center_marker.ns = "tree"
            center_marker.id = 0
            center_marker.type = Marker.CYLINDER
            center_marker.action = Marker.ADD
            center_marker.pose.position = self.tree_center
            center_marker.pose.orientation.w = 1.0
            center_marker.scale.x = self.tree_radius * 2
            center_marker.scale.y = self.tree_radius * 2
            center_marker.scale.z = 3.0
            center_marker.color.g = 0.8
            center_marker.color.a = 0.5
            marker_array.markers.append(center_marker)
            
            safety_marker = Marker()
            safety_marker.header.frame_id = "map"
            safety_marker.header.stamp = rospy.Time.now()
            safety_marker.ns = "safety"
            safety_marker.id = 1
            safety_marker.type = Marker.CYLINDER
            safety_marker.action = Marker.ADD
            safety_marker.pose.position = self.tree_center
            safety_marker.pose.orientation.w = 1.0
            safety_marker.scale.x = (self.tree_radius + self.safety_distance) * 2
            safety_marker.scale.y = (self.tree_radius + self.safety_distance) * 2
            safety_marker.scale.z = 0.1
            safety_marker.color.r = 0.8
            safety_marker.color.a = 0.3
            marker_array.markers.append(safety_marker)
        
        if self.current_goal:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = rospy.Time.now()
            goal_marker.ns = "goal"
            goal_marker.id = 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position = self.current_goal
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.5
            goal_marker.scale.y = 0.5
            goal_marker.scale.z = 0.5
            goal_marker.color.b = 1.0
            goal_marker.color.a = 0.8
            marker_array.markers.append(goal_marker)
        
        self.vis_pub.publish(marker_array)

    @staticmethod
    def distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

if __name__ == '__main__':
    try:
        explorer = TreeAvoidanceExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
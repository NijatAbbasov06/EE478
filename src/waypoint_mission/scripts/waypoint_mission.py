#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
<<<<<<< HEAD
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from math import sqrt, pow, cos, sin, pi


# Waypoints [[X list], [Y list], [Z list]]
waypoint_list = [[0, 1, 1, 0, 0], \
                [0, 0, 1, 1, 0], \
                [1, 1, 1, 1, 1]]
=======
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt, pow, cos, sin, pi
import numpy as np


# Waypoints [[X list], [Y list], [Z list]]
waypoint_list = [[0, 0, 8, 8, 0, 0], \
                [0, 4, 4, -4, -4, 0], \
                [1, 1, 1, 1, 1, 1]]

# Center of the circle (change as needed)
center_x = 4
center_y = 0

# Hardcoded yaw setpoints (in radians) for each waypoint, facing the center (4, 0)
yaw_setpoints = [-0.1, -0.785, -1.5708, -2.356, -3.1416, 2.356]  # Example values, adjust as needed
>>>>>>> adding files

def dist(goal, pose_now):
    return sqrt(pow(pose_now.x-goal.x,2) + pow(pose_now.y-goal.y,2) + pow(pose_now.z-goal.z,2))

class WaypointMission:
    def __init__(self):
        self.cur_waypoint_idx = -1
        self.cur_waypoint = Point()
<<<<<<< HEAD

        self.waypoint_server = rospy.Service("waypoint_mission_server", Empty, self.waypoint_service)
        self.waypoint_pub = rospy.Publisher("waypoint_mission", Point, queue_size = 10)
        self.cur_position_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.auto_arrive_checker_cb, queue_size=1)
        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pose_real = Point()
=======
        self.pose_real = Point()
        self.cur_yaw = 0.0

        self.waypoint_server = rospy.Service("waypoint_mission_server", Empty, self.waypoint_service)
        self.waypoint_pub = rospy.Publisher("waypoint_mission", Point, queue_size = 10)
        self.cur_position_sub = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.auto_arrive_checker_cb, queue_size=1)
        self.local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
>>>>>>> adding files

    def waypoint_service(self, req):
        if(self.get_next_waypoint()):
            rospy.loginfo("Waypoint updated")
        else:
            rospy.loginfo("Waypoint update failed")
        return EmptyResponse()
    
    def get_next_waypoint(self):
        #this functioin assigns the next waypoint to the cur_waypoint. This is called when the waypoint server receives a request
        print(len(waypoint_list[0]))
        if(self.cur_waypoint_idx < len(waypoint_list[0]) - 1):
            self.cur_waypoint_idx = self.cur_waypoint_idx + 1

        else:
            print("All waypoint mission are already been finished, starting again")
            self.cur_waypoint_idx = 0
            
        
        self.cur_waypoint.x = waypoint_list[0][self.cur_waypoint_idx]
        self.cur_waypoint.y = waypoint_list[1][self.cur_waypoint_idx]
        self.cur_waypoint.z = waypoint_list[2][self.cur_waypoint_idx]
<<<<<<< HEAD
=======
        self.cur_yaw = yaw_setpoints[self.cur_waypoint_idx]
>>>>>>> adding files

        return True
    
    def auto_arrive_checker_cb(self, msg):
<<<<<<< HEAD
        self.pose = msg
        self.pose_real = self.pose.pose.pose.position
=======
        # Ensure msg is a PoseStamped and has a position
        if not hasattr(msg, 'pose') or not hasattr(msg.pose, 'position'):
            rospy.logerr("Received invalid PoseStamped message")
            return
        self.pose_real.z = (-1) * msg.pose.position.y
        self.pose_real.y  = msg.pose.position.x
        self.pose_real.x = msg.pose.position.z
        rospy.loginfo(f"Current position: x={self.pose_real.x}, y={self.pose_real.y}, z={self.pose_real.z}")
        rospy.loginfo(f"Current waypoint: x={self.cur_waypoint.x}, y={self.cur_waypoint.y}, z={self.cur_waypoint.z}")
>>>>>>> adding files
        if dist(self.cur_waypoint, self.pose_real) < 0.2:
            rospy.loginfo("Reached waypoint! Requesting next waypoint.")
            try:
                waypoint_service_client = rospy.ServiceProxy("waypoint_mission_server", Empty)
                waypoint_service_client() 
<<<<<<< HEAD
            
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)


                

        
    def run(self):

        self.waypoint_pub.publish(self.cur_waypoint)
        
        ##################################################
=======
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

    def run(self):
        self.waypoint_pub.publish(self.cur_waypoint)
>>>>>>> adding files
        # PID Controller
        error_x = self.cur_waypoint.x - self.pose_real.x
        error_y = self.cur_waypoint.y - self.pose_real.y
        error_z = self.cur_waypoint.z - self.pose_real.z

        K_p = 1
        cmd_velocity = TwistStamped()
<<<<<<< HEAD
=======
        cmd_velocity.header.stamp = rospy.Time.now()
        cmd_velocity.header.frame_id = "map"
>>>>>>> adding files

        cmd_velocity.twist.linear.x = K_p * error_x
        cmd_velocity.twist.linear.y = K_p * error_y
        cmd_velocity.twist.linear.z = K_p * error_z

<<<<<<< HEAD
        
        ##################################################
=======
        # Use the precomputed yaw setpoint for this waypoint
        cmd_velocity.twist.angular.z = self.cur_yaw
>>>>>>> adding files

        self.local_vel_pub.publish(cmd_velocity)
    

    
    def check_waypoint_list(self):
        for dim in range(0, len(waypoint_list) - 2):
            if (len(waypoint_list[dim]) != len(waypoint_list[dim+1])):
                return False
            
        return True
    
if __name__ == '__main__':
    rospy.init_node("waypoint_mission_node")
    rate = rospy.Rate(10)

    wp_mission = WaypointMission()
    if not wp_mission.check_waypoint_list():
        rospy.logerr("Waypoint is not well formed")
        quit()

    while not rospy.is_shutdown():
        wp_mission.run()
        rate.sleep()


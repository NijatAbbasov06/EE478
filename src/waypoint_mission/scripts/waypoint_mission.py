#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from math import sqrt, pow, cos, sin, pi


# Waypoints [[X list], [Y list], [Z list]]
waypoint_list = [Point(5, -4, 1.4),
            Point(15, -4, 1.4),
            Point(15, 4, 1.4),
            Point(5, 4, 1.4),
            Point(0, 0, 0),]

def dist(goal, pose_now):
    return sqrt(pow(pose_now.x-goal.x,2) + pow(pose_now.y-goal.y,2) + pow(pose_now.z-goal.z,2))

class WaypointMission:
    def __init__(self):
        self.cur_waypoint_idx = -1
        self.cur_waypoint = Point()

        self.waypoint_server = rospy.Service("waypoint_mission_server", Empty, self.waypoint_service)
        self.waypoint_pub = rospy.Publisher("waypoint_mission", Point, queue_size = 10)
        self.cur_position_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.auto_arrive_checker_cb, queue_size=1)
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pose_real = Point()

    def waypoint_service(self, req):
        if(self.get_next_waypoint()):
            rospy.loginfo("Waypoint updated")
        else:
            rospy.loginfo("Waypoint update failed")
        return EmptyResponse()
    
    def get_next_waypoint(self):
   
        print(len(waypoint_list))
        if(self.cur_waypoint_idx < len(waypoint_list) - 1):
            self.cur_waypoint_idx = self.cur_waypoint_idx + 1

        else:
            print("All waypoint mission are already been finished, starting again")
            self.cur_waypoint_idx = 0
            
        self.cur_waypoint = waypoint_list[self.cur_waypoint_idx]
        return True
    
    def auto_arrive_checker_cb(self, msg):
        self.pose = msg
        self.pose_real = self.pose.pose.pose.position
        if dist(self.cur_waypoint, self.pose_real) < 0.2:
            rospy.loginfo("Reached waypoint! Requesting next waypoint.")
            try:
                waypoint_service_client = rospy.ServiceProxy("waypoint_mission_server", Empty)
                waypoint_service_client() 
            
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)


                

        
    def run(self):

        self.waypoint_pub.publish(self.cur_waypoint)
        
        ##################################################
        # PID Controller
        error_x = self.cur_waypoint.x - self.pose_real.x
        error_y = self.cur_waypoint.y - self.pose_real.y
        error_z = self.cur_waypoint.z - self.pose_real.z

        K_p = 1
        cmd_velocity = TwistStamped()

        cmd_velocity.twist.linear.x = K_p * error_x
        cmd_velocity.twist.linear.y = K_p * error_y
        cmd_velocity.twist.linear.z = K_p * error_z

        
        ##################################################

        self.local_vel_pub.publish(cmd_velocity)
    

        
    def check_waypoint_list(self):
        return all(isinstance(wp, Point) for wp in waypoint_list)
    
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

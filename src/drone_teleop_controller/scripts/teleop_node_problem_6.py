#! /usr/bin/env python

import rospy

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = Path()
path_pub = None

input_vel = Twist()

def pose_callback(msg):
    global input_vel
    input_vel = msg
    print("Input velocity received")
    
def position_callback(msg):
    global path
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "map"

    pose = PoseStamped()
    pose.header = path.header
    pose.pose = msg.pose

    path.poses.append(pose)
    path_pub.publish(path)

if __name__ == "__main__":
    rospy.init_node("teleop_node_py")
    path_pub = rospy.Publisher("/path", Path, queue_size=10)
    pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=position_callback)
    state_sub = rospy.Subscriber("/cmd_vel", Twist, callback = pose_callback)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(30)

    output_vel = TwistStamped()
    
    while(not rospy.is_shutdown()):
        output_vel.header.stamp = rospy.Time.now()
        output_vel.header.frame_id = "base_link"
        output_vel.twist = input_vel
        local_vel_pub.publish(output_vel)
        rate.sleep()

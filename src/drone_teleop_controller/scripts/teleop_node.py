#! /usr/bin/env python

import rospy

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

input_vel = Twist()

def pose_callback(msg):
    global input_vel
    input_vel = msg
    print("Input velocity received")
    

if __name__ == "__main__":
    rospy.init_node("teleop_node_py")

    state_sub = rospy.Subscriber("/cmd_vel", Twist, callback = pose_callback)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(30)

    output_vel = TwistStamped()
    
    # main loop
    while(not rospy.is_shutdown()):
        output_vel.header.stamp = rospy.Time.now()
        output_vel.header.frame_id = "base_link"
        output_vel.twist = input_vel
        local_vel_pub.publish(output_vel)
        rate.sleep()

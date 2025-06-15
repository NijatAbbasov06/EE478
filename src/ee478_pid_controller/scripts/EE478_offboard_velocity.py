#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

current_pose = PoseStamped()

def pose_callback(msg):
    global current_pose
    current_pose = msg
    print("Pose Received")
    print("X : "+str(current_pose.pose.position.x)+", Y : "+str(current_pose.pose.position.y)+", Z : "+str(current_pose.pose.position.z))

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

<<<<<<< HEAD
    state_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pose_callback)
=======
    state_sub = rospy.Subscriber("/orb_slam3_ros/camera_pose", PoseStamped, callback = pose_callback)
>>>>>>> adding files
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(30)

    cmd_velocity = TwistStamped()
    cmd_velocity.twist.linear.x = 0
    cmd_velocity.twist.linear.y = 0
    cmd_velocity.twist.linear.z = 0.5

    target_X = 5.0
    target_Y = 1.0
    target_Z = 20

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_vel_pub.publish(cmd_velocity)
        rate.sleep()
    
    # main loop
    while(not rospy.is_shutdown()):
        ##################################################
        # PID Controller

        error_x = target_X - current_pose.pose.position.x
        error_y = target_Y - current_pose.pose.position.y
        error_z = target_Z - current_pose.pose.position.z

        K_p = 0.5

        cmd_velocity.twist.linear.x = K_p * error_x
        cmd_velocity.twist.linear.y = K_p * error_y
        cmd_velocity.twist.linear.z = K_p * error_z

        
        ##################################################

        local_vel_pub.publish(cmd_velocity)
        rate.sleep()

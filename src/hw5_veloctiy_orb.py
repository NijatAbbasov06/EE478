#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply,quaternion_inverse 

current_pose = PoseStamped()
target_pose = PoseStamped()


def pose_callback(msg):
    global current_pose
    current_pose = msg

def waypoint_callback(msg):
    global target_pose
    target_pose = msg

def get_yaw_from_orientation(orientation):
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw




def yaw_error(target_yaw, current_yaw):
    # Normalize to [-pi, pi]
    error = target_yaw - current_yaw
    return math.atan2(math.sin(error), math.cos(error))

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

if __name__ == "__main__":
    rospy.init_node("offb_node_py")



    rospy.Subscriber("/april_pose", PoseStamped, callback=pose_callback)

#    rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, callback=pose_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback=waypoint_callback)

    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    rate = rospy.Rate(20)

    cmd_velocity = TwistStamped()

    for _ in range(100):
        if rospy.is_shutdown():
            break
        local_vel_pub.publish(cmd_velocity)

    K_P = 0.3
    K_YAW = 0.3

    MAX_VEL_XY = 1.0   # m/s
    MAX_VEL_Z  = 0.5   # m/s
    MAX_YAW_RATE = 0.7 # rad/s

    while not rospy.is_shutdown():
        # --- position error ---
        error_x = target_pose.pose.position.x - current_pose.pose.position.x
        error_y = target_pose.pose.position.y - current_pose.pose.position.y
        error_z = target_pose.pose.position.z - current_pose.pose.position.z

        vx = K_P * error_x
        vy = K_P * error_y
        vz = K_P * error_z

        # --- clamp velocity ---
        cmd_velocity.twist.linear.x = clamp(vx, -MAX_VEL_XY, MAX_VEL_XY)
        cmd_velocity.twist.linear.y = clamp(vy, -MAX_VEL_XY, MAX_VEL_XY)
        cmd_velocity.twist.linear.z = clamp(vz, -MAX_VEL_Z, MAX_VEL_Z)

        # --- yaw control given code---
        current_yaw = get_yaw_from_orientation(current_pose.pose.orientation)
#        target_yaw = get_yaw_from_orientation(target_pose.pose.orientation)

        if abs(error_x) < 1e-6 and abs(error_y) < 1e-6:
            # 
            target_yaw = current_yaw
        else:
            target_yaw = math.atan2(error_y, error_x)


        error_yaw = yaw_error(target_yaw, current_yaw)
        wz = K_YAW * error_yaw


        cmd_velocity.twist.angular.z = clamp(wz, -MAX_YAW_RATE, MAX_YAW_RATE)

        local_vel_pub.publish(cmd_velocity)
        rate.sleep()

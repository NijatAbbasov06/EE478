#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation as R

class PoseFusionNode:
    def __init__(self):
        rospy.init_node('pose_fusion_node', anonymous=True)
        
        # Publishers
        self.fused_pose_pub = rospy.Publisher('/fused_pose', PoseStamped, queue_size=10)
        
        # Subscribers
        self.april_sub = rospy.Subscriber('/april_pose', PoseStamped, self.april_callback)
        self.orb_sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, self.orb_callback)
        
        # State variables
        self.correction_matrix = None
        self.last_orb_pose = None
        self.last_april_time = None
        self.april_stream_active = False
        self.april_timeout = 0.2  # If no AprilTag for 200ms, consider stream stopped
        
        # Timer to check AprilTag stream status
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_april_stream)
        
        rospy.loginfo("Pose Fusion Node initialized")
        
    def pose_to_matrix(self, pose):
        """Convert ROS Pose to 4x4 transformation matrix"""
        # Extract position
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # Extract quaternion and convert to rotation matrix
        quat = [pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w]
        rotation = R.from_quat(quat)
        rot_matrix = rotation.as_matrix()
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = t
        
        return T
    
    def matrix_to_pose(self, T):
        """Convert 4x4 transformation matrix to ROS Pose"""
        pose = Pose()
        
        # Extract position
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]
        
        # Extract rotation and convert to quaternion
        rotation = R.from_matrix(T[:3, :3])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def april_callback(self, msg):
        """Callback for AprilTag pose data"""
        self.last_april_time = rospy.Time.now()
        
        # Update correction matrix if we have ORB data
        if self.last_orb_pose is not None:
            april_matrix = self.pose_to_matrix(msg.pose)
            orb_matrix = self.pose_to_matrix(self.last_orb_pose.pose)
            
            try:
                # Compute correction: T_correction = T_april * T_orb^(-1)
                self.correction_matrix = april_matrix @ np.linalg.inv(orb_matrix)
                rospy.logdebug("Updated correction matrix")
                
            except np.linalg.LinAlgError:
                rospy.logwarn("Failed to compute correction matrix (singular matrix)")
        
        # Always publish AprilTag when stream is active
        if self.april_stream_active:
            fused_msg = PoseStamped()
            fused_msg.header = msg.header
            fused_msg.header.frame_id = "map"
            fused_msg.pose = msg.pose
            
            self.fused_pose_pub.publish(fused_msg)
    
    def orb_callback(self, msg):
        """Callback for ORB-SLAM3 pose data"""
        self.last_orb_pose = msg
        
        # ONLY use ORB when AprilTag stream is STOPPED
        if not self.april_stream_active:
            if self.correction_matrix is not None:
                # Apply last correction to current ORB pose
                orb_matrix = self.pose_to_matrix(msg.pose)
                corrected_matrix = self.correction_matrix @ orb_matrix
                corrected_pose = self.matrix_to_pose(corrected_matrix)
                
                # Publish corrected ORB pose
                fused_msg = PoseStamped()
                fused_msg.header = msg.header
                fused_msg.header.frame_id = "map"
                fused_msg.pose = corrected_pose
                
                self.fused_pose_pub.publish(fused_msg)
                rospy.logdebug("Published corrected ORB pose (AprilTag stream stopped)")
                
            else:
                rospy.logwarn_throttle(1, "AprilTag stream stopped but no correction matrix available!")
                # Emergency: publish raw ORB (better than nothing)
                fused_msg = PoseStamped()
                fused_msg.header = msg.header
                fused_msg.header.frame_id = "map"
                fused_msg.pose = msg.pose
                self.fused_pose_pub.publish(fused_msg)
    
    def check_april_stream(self, event):
        """Timer callback to check if AprilTag stream is active"""
        current_time = rospy.Time.now()
        
        if self.last_april_time is not None:
            time_since_april = (current_time - self.last_april_time).to_sec()
            
            if time_since_april < self.april_timeout:
                # Stream is active
                if not self.april_stream_active:
                    rospy.loginfo("🟢 AprilTag stream ACTIVE - using AprilTag only")
                self.april_stream_active = True
            else:
                # Stream stopped
                if self.april_stream_active:
                    rospy.logwarn("🔴 AprilTag stream STOPPED - switching to corrected ORB")
                self.april_stream_active = False
        else:
            # Never received AprilTag
            self.april_stream_active = False
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PoseFusionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Fusion Node shutting down")
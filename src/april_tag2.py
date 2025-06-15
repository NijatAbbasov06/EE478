#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from apriltag_ros.msg import AprilTagDetectionArray  # Assuming this is your detection message type
from scipy.spatial.transform import Rotation
import numpy as np
from math import cos, sin
from scipy.spatial.transform import Rotation as R
import numpy as np



class QuaternionFilter:
    """Simple quaternion filter to ensure continuity"""
    def __init__(self):
        self.last_quat = None
    
    def ensure_continuity(self, quat):
        """Ensure quaternion sign consistency"""
        if self.last_quat is None:
            self.last_quat = np.array(quat)
            return quat
        
        # Check if we should flip the quaternion sign
        if np.dot(quat, self.last_quat) < 0:
            quat = [-q for q in quat]
        
        self.last_quat = np.array(quat)
        return quat


class AprilTagLocalizationNode:
    def __init__(self):
        rospy.init_node('april_tag_localization_node', anonymous=True)
        
        # Initialize quaternion filter
        self.quat_filter = QuaternionFilter()
        
        # Gate definitions: (name, x, y, z, yaw_degrees)
        self.gate_defs = [
            ("gate_pair_00",  5.0, -4.0, 0.0,  -45),
            ("gate_pair_01", 15.0, -4.0, 0.0,   45),
            ("gate_pair_02", 15.0,  4.0, 0.0,  135),
            ("gate_pair_03",  5.0,  4.0, 0.0, -135),
        ]
        # Camera to base_link transform matrix
        self.transform_matrix = np.array([
            [0,  0,  1],  # base_link_x = camera_z (forward)
            [-1, 0,  0],  # base_link_y = -camera_x (left)
            [0, -1,  0]   # base_link_z = -camera_y (up)
        ])
        
        # Publishers
        self.pose_pub = rospy.Publisher('april_pose', PoseStamped, queue_size=10)
        
        # Subscribers
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        
        rospy.loginfo("AprilTag Localization Node initialized")
        rospy.loginfo(f"Loaded {len(self.gate_defs)} gate definitions")
        
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        detection = msg.detections[0]
        
        # Get the tag IDs (should be 4 consecutive values)
        tag_ids = detection.id  # e.g., [0,1,2,3] or [4,5,6,7] or [8,9,10,11] or [12,13,14,15]
        
        # Handle case where detection.id might be a single integer or a list
        if isinstance(tag_ids, int):
            tag_ids = [tag_ids]
        
        # Determine which gate pair based on the first tag ID
        first_tag_id = tag_ids[0]
        gate_index = first_tag_id // 4  # 0//4=0, 4//4=1, 8//4=2, 12//4=3
        
        # Get the corresponding gate definition
        if gate_index < len(self.gate_defs):
            gate_name, gate_x, gate_y, gate_z, gate_yaw = self.gate_defs[gate_index]
            gate_yaw_radian = np.radians(gate_yaw)
            rospy.loginfo(f"Detected gate: {gate_name} with tags {tag_ids}")
            rospy.loginfo(f"Gate world position: ({gate_x}, {gate_y}, {gate_z}) with yaw: {gate_yaw}°")
            
            # Get pose from detection
            pose_stamped = detection.pose
            rel_pose = pose_stamped.pose.pose.position
            rel_pose_arr = [rel_pose.x, rel_pose.y, rel_pose.z]
            world_pose = self.transform_matrix @ rel_pose_arr
             
            world_pose_final = [world_pose[0] * cos(gate_yaw_radian) + world_pose[1] * sin(gate_yaw_radian), world_pose[0] * sin(gate_yaw_radian) + (-1) * world_pose[1] * cos(gate_yaw_radian), world_pose[2]]
            print(world_pose[0] * sin(gate_yaw_radian))
            print((-1) * world_pose[1] * cos(gate_yaw_radian))
            print(world_pose[0] * sin(gate_yaw_radian) + (-1) * world_pose[1] * cos(gate_yaw_radian))

            base_pose = [gate_x - world_pose_final[0], gate_y - world_pose_final[1], 0.75+ gate_z - world_pose_final[2]]


            rel_orientation = pose_stamped.pose.pose.orientation
            quat = [rel_orientation.x, rel_orientation.y, rel_orientation.z, rel_orientation.w]
            # Create T_camera_to_tagbundle (your current T)
            T_camera_to_tagbundle = np.eye(4)
            T_camera_to_tagbundle[0:3, 0:3] = Rotation.from_quat(quat).as_matrix()
            T_camera_to_tagbundle[0:3, 3] = [rel_pose.x, rel_pose.y, rel_pose.z]
            
            # Create T_world_to_gate (gate position in world frame)
            gate_yaw_rad = np.radians(gate_yaw)
            T_world_to_gate = np.eye(4)
            T_world_to_gate[0:3, 0:3] = Rotation.from_euler('z', gate_yaw_rad).as_matrix()
            T_world_to_gate[0:3, 3] = [gate_x, gate_y, gate_z]
            
            # If tagbundle frame = gate frame, then:
            # T_world_to_camera = T_world_to_gate * T_gate_to_camera
            # T_world_to_camera = T_world_to_gate * inv(T_camera_to_gate)
            T_world_to_camera = T_world_to_gate @ np.linalg.inv(T_camera_to_tagbundle)
            
            # Extract camera position in world frame
            camera_world_pos = T_world_to_camera[0:3, 3]
            base_world_pose = self.transform_matrix @ camera_world_pos
            camera_world_rot = Rotation.from_matrix(T_world_to_camera[0:3, 0:3])
            camera_euler = camera_world_rot.as_euler('xyz', degrees=True)
            
            # rospy.loginfo(f"Camera world position: ({camera_world_pos[0]:.2f}, {camera_world_pos[1]:.2f}, {camera_world_pos[2]:.2f})")
            # rospy.loginfo(f"Base world position: ({base_world_pose[0]:.2f}, {base_world_pose[1]:.2f}, {base_world_pose[2]:.2f})")
            # rospy.loginfo(f"Camera world orientation: Roll={camera_euler[0]:.1f}°, Pitch={camera_euler[1]:.1f}°, Yaw={camera_euler[2]:.1f}°")
            rospy.loginfo(f"rel_pose: ({rel_pose_arr[0]:.2f}, {rel_pose_arr[1]:.2f}, {rel_pose_arr[2]:.2f})")
            rospy.loginfo(f"world_pose: ({world_pose[0]:.2f}, {world_pose[1]:.2f}, {world_pose[2]:.2f})")
            rospy.loginfo(f"world_pose_final: ({world_pose_final[0]:.2f}, {world_pose_final[1]:.2f}, {world_pose_final[2]:.2f})")
            rospy.loginfo(f"Base_pose: ({base_pose[0]:.2f}, {base_pose[1]:.2f}, {base_pose[2]:.2f})")
            
            if(base_pose[2]<6):
            # Publish the base world pose
                self.publish_base_pose(base_pose, gate_yaw_rad, msg.header.stamp)
            else:
                rospy.logwarn("Gimbal Lock")
                
        else:
            rospy.logwarn(f"Unknown gate for tag IDs: {tag_ids}")
    
    def publish_base_pose(self, base_pose, gate_yaw_radian, timestamp):
        """Publish the base world pose to april_pose topic"""
        pose_msg = PoseStamped()
        
        # Set header
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "world"  # or "map" depending on your frame convention
        
        # Set position (base_link position in world frame)
        pose_msg.pose.position.x = base_pose[0]
        pose_msg.pose.position.y = base_pose[1]
        pose_msg.pose.position.z = base_pose[2]

        rotation = R.from_rotvec([0, 0, gate_yaw_radian])  # Rotation vector around z-axis
        base_quat = rotation.as_quat()
        

        # For orientation, we need to transform the camera orientation to base_link orientation
        # This is a simplified approach - you might need to adjust based on your specific setup
      
        
        # Transform camera orientation to base_link orientation
        # base_link_rotation = camera_rotation * camera_to_base_rotation
        # This is an approximation - you may need to adjust this transformation
        
     
        
        pose_msg.pose.orientation.x = base_quat[0]
        pose_msg.pose.orientation.y = base_quat[1]
        pose_msg.pose.orientation.z = base_quat[2]
        pose_msg.pose.orientation.w = base_quat[3]
        
        # Publish the pose
        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published base pose to april_pose topic")
    
    def run(self):
        """Main loop"""
        rospy.loginfo("AprilTag Localization Node is running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AprilTagLocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("AprilTag Localization Node interrupted")
        pass
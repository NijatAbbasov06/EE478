#!/usr/bin/env python

import rospy
import os
import tf.transformations as tft
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import math

def spawn_model(name, sdf_path, x, y, z, yaw_deg):
    with open(sdf_path, "r") as f:
        sdf = f.read()

    yaw_rad = math.radians(yaw_deg)
    quat = tft.quaternion_from_euler(0, 0, yaw_rad)
    pose = Pose(position=Point(x, y, z), orientation=Quaternion(*quat))

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawner(name, sdf, "", pose, "world")
    rospy.loginfo(f"Spawned {name} at ({x:.1f}, {y:.1f}) yaw={yaw_deg}")

def main():
    rospy.init_node("spawn_gate_and_qr")

    base_dir = os.path.join(os.path.dirname(__file__), "..", "models")

    gate_defs = [
        # name        x     y     z     yaw
        ("gate_pair_00",  5.0, -4.0, 0.0,  -45),
        ("gate_pair_01", 15.0, -4.0, 0.0,   45),
        ("gate_pair_02", 15.0,  4.0, 0.0,  135),
        ("gate_pair_03",  5.0,  4.0, 0.0, -135),
    ]

    for i, (gate_name, x, y, z, yaw) in enumerate(gate_defs):
        gate_model = os.path.join(base_dir, gate_name, "model.sdf")
        spawn_model(gate_name, gate_model, x, y, z, yaw)

        # QR 코드 모델은 qr_code_01, qr_code_02, ...
        qr_name = f"qr_code_{i+1:02d}"
        qr_model = os.path.join(base_dir, qr_name, "model.sdf")
        qr_z = z + 1.5  # 게이트 위에
        spawn_model(qr_name, qr_model, x, y, qr_z, yaw)

if __name__ == "__main__":
    main()

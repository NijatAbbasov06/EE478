cmake_minimum_required(VERSION 3.0.2)
project(ee478_px4_sim)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spawn_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(youbot_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
  gazebo_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cube_detector.py
  scripts/pick_place_mission.py
  scripts/spawn_task_objects.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

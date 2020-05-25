cmake_minimum_required(VERSION 2.8.3)

project(ur_rossharp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  ur_gazebo
  ur5_moveit_config
)

catkin_package()

install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

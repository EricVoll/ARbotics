#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/catkin_ws/devel/setup.bash
source /home/ros-sharp_ws/devel/setup.bash
roslaunch --wait file_server ros_sharp_communication.launch 
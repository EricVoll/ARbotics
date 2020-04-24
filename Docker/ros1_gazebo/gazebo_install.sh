#!/bin/bash
source /opt/ros/melodic/setup.bash
echo "WORKING"
sudo apt-get install -y apt-utils

mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git
cd /home/catkin_ws/
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -y
# building
catkin_make

sudo apt-get install python-rospkg

sudo apt-get install -y ros-melodic-moveit 

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc 

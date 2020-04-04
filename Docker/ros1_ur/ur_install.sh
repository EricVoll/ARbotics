#!/bin/bash
source /opt/ros/melodic/setup.bash
echo "WORKING"
sudo apt-get install -y apt-utils

mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git
cd home/catkin_ws/
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -y
# building
catkin_make

sudo apt-get install python-rospkg

sudo apt-get install -y ros-melodic-moveit 

echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc 
echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc 

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# #soruce workspace
# source /home/catkin_ws/devel/setup.bash
# sudo apt-get install ros-melodic-moveit
# sudo docker build -t ros1_ros-sharp:1.0 .
#sudo docker run -it --network=host ros1_ros-sharp:1.0
#rostopic pub -r 1  euclidean_goal_pose geometry_msgs/Pose '{position:  {x: 0.6, y: 0.6, z: 0.40}, orientation: {x: 0.0,y: 0.0,z: 0.0, w: 1.2}}'
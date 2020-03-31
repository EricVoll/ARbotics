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


# #soruce workspace
# source /home/catkin_ws/devel/setup.bash
# sudo apt-get install ros-melodic-moveit
# sudo docker build -t ros1_ros-sharp:1.0 .
#sudo docker run -it --network=host ros1_ros-sharp:1.0
#rostopic pub -r 10  euclidean_goal_pose geometry_msgs/Pose {position:  {x: 0.6, y: 0.6, z: 0.40}, orientation: {x: 0.0,y: 0.0,z: 0.0, w: 1.2}}'
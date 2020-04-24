#!/bin/bash
source /opt/ros/melodic/setup.bash
echo "WORKING"
sudo apt-get install -y apt-utils
mkdir -p /home/ros-sharp_ws/src
cd /home/ros-sharp_ws
catkin_make

source /home/ros-sharp_ws/devel/setup.bash

sudo apt-get -y install ros-melodic-rosbridge-server 

cd /home/ 
git clone https://github.com/siemens/ros-sharp.git -b master
mv ros-sharp/ROS/file_server ros-sharp_ws/src
cd /home/ros-sharp_ws 
source /opt/ros/melodic/setup.bash
catkin_make

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc 
echo 'source /home/ros-sharp_ws/devel/setup.bash' >> ~/.bashrc 
# sudo docker build -t ros1_ros-sharp:1.0 .
#sudo docker run -it --network=host ros1_ros-sharp:1.0
#rostopic pub -r 10  euclidean_goal_pose geometry_msgs/Pose {position:  {x: 0.6, y: 0.6, z: 0.40}, orientation: {x: 0.0,y: 0.0,z: 0.0, w: 1.2}}'
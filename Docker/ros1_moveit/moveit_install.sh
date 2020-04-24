#!/bin/bash
source /opt/ros/melodic/setup.bash

sudo apt-get install -y apt-utils
sudo apt-get install -y ros-melodic-moveit 
sudo apt-get install python-rospkg

mkdir -p /home/catkin_ws/src

cd /home/catkin_ws/
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -y
# building
catkin_make
#soruce workspace
source /home/catkin_ws/devel/setup.bash

mv /home/test_moveit_interface/ /home/catkin_ws/src/
source /opt/ros/melodic/setup.bash 
cd /home/catkin_ws
catkin_make 
source /home/catkin_ws/devel/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc 


# sudo docker build -t ros1_ros-sharp:1.0 .
#sudo docker run -it --network=host ros1_ros-sharp:1.0
#rostopic pub -r 10  euclidean_goal_pose geometry_msgs/Pose {position:  {x: 0.6, y: 0.6, z: 0.40}, orientation: {x: 0.0,y: 0.0,z: 0.0, w: 1.2}}

#roslaunch ur_description ur5_upload.launch
mkdir -p /home/gazebo2_ws/src
cd /home/gazebo2_ws
source /opt/ros/eloquent/setup.bash

colcon build 
source /home/gazebo2_ws/install/setup.bash

sudo apt install -y ros-eloquent-gazebo-ros-pkgs
sudo apt install -y ros-eloquent-ros-core ros-eloquent-geometry2

cp -b /opt/ros/eloquent/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world /home/
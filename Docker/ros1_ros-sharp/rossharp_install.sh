mkdir -p /home/ros-sharp_ws/src
cd /home/ros-sharp_ws/src
cd /home/ros-sharp_ws
catkin_make
source /home/ros-sharp/devel/setup.bash

sudo apt-get -y install ros-melodic-rosbridge-server 

cd /home/ 
git clone https://github.com/siemens/ros-sharp.git -b master
mv ros-sharp/ROS/file_server ros-sharp_ws/src
cd /home/ros-sharp_ws 
source /opt/melodic/setup.bash
catkin_make
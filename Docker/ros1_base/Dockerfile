FROM ros:melodic-ros-base-bionic

# This is the base image for all ROS1 containers
# If you add a workspace append it to ~./bashrc

# Setting default shell to bash
SHELL ["/bin/bash", "--login", "-c"]

RUN apt-get update && apt-get install -y dos2unix

RUN apt-get clean && apt-get update && apt-get install -y locales 
RUN sudo apt update && sudo apt install -y \
  vim 

#Create a workspace / get a bridge running between ROS1 and ROS2
RUN mkdir /home/catkin_ws
RUN mkdir /home/catkin_ws/src

RUN source /opt/ros/melodic/setup.bash && \
    cd  /home/catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc

WORKDIR /home
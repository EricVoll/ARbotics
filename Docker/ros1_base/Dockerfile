# Base image used to create costum ROS1 containers
FROM ros:melodic-ros-base-bionic

# Setting default shell to bash
SHELL ["/bin/bash", "--login", "-c"]

RUN apt-get update && apt-get install -y dos2unix

RUN apt-get clean && apt-get update && apt-get install -y locales 
RUN sudo apt update && sudo apt install -y \
  vim 

# Creating default catkin_ws
RUN mkdir /home/catkin_ws
RUN mkdir /home/catkin_ws/src
# Make and append sourceing workspace to bashrc
RUN source /opt/ros/melodic/setup.bash && \
    cd  /home/catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc

# Set working directory to home as default location
WORKDIR /home
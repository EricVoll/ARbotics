FROM ros:melodic-ros-base-bionic

#since we are in a minimal docker enviroment 
#https://stackoverflow.com/questions/39760663/docker-ubuntu-bin-sh-1-locale-gen-not-found
RUN apt-get clean && apt-get update && apt-get install -y locales 
RUN sudo apt update && sudo apt install -y \
  vim 

#Create a workspace / get a bridge running between ROS1 and ROS2
RUN mkdir /home/catkin_ws
RUN mkdir /home/catkin_ws/src

RUN /bin/bash -c  "source /opt/ros/melodic/setup.bash && \
                  cd  /home/catkin_ws && \
                  catkin_make -DCMAKE_BUILD_TYPE=Release "

#SHELL ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash"] 
# RUN /bin/bash -c  " source ~/catkin_ws/devel/setup.bash"


WORKDIR /home
FROM ros2_base:1.0

###############################copied from here##################### 
#https://github.com/osrf/docker_images/blob/b075c7dbe56055d862f331f19e1e74ba653e181a/ros/melodic/ubuntu/bionic/ros-core/ros_entrypoint.sh

# setup timezone
# RUN echo 'Etc/UTC' > /etc/timezone && \
#     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro melodic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*
####################################################################


RUN apt-get clean && apt-get update && apt-get install -y locales 
RUN /bin/bash -c  "echo 'source /opt/ros/melodic/setup.bash' >>  ~/.bashrc " 
RUN sudo apt update && sudo apt install -y \
  vim 

#Create a workspace / get a bridge running between ROS1 and ROS2
RUN mkdir /home/catkin_ws
RUN mkdir /home/catkin_ws/src
RUN /bin/bash -c  "source /opt/ros/melodic/setup.bash && \
                  cd  /home/catkin_ws && \
                  catkin_make -DCMAKE_BUILD_TYPE=Release "

RUN /bin/bash -c  "echo 'source ~/catkin_ws/devel/local_setup.bash' >>  ~/.bashrc " 
# RUN /bin/bash -c  " source ~/catkin_ws/devel/setup.bash"

WORKDIR /home
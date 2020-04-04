# Kudos to DOROWU for his amazing VNC 18.04 LXDE image
FROM dorowu/ubuntu-desktop-lxde-vnc:bionic
LABEL maintainer "fabrizio.bottarel@iit.it"

# Fix dirmngr
RUN sudo apt purge dirmngr -y && sudo apt update && sudo apt install dirmngr -y

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS
RUN sudo apt-get update && sudo apt-get install -y \
    ros-melodic-desktop-full \
		wget \
    git \
    nano

RUN sudo apt install -y  \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python-rosdep

# Source ROS initially  
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
SHELL ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash"] 
RUN rosdep init && rosdep update

RUN echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc

# Creating ROS_WS
RUN mkdir -p ~/ros_ws/src

# Set up the workspace
RUN cd ~/ros_ws/ && \
    catkin_make && \
    echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc && \
    echo 'source ~/ros_ws/devel/setup.bash' >> /root/.bashrc

# Updating ROSDEP and installing dependencies
RUN cd ~/ros_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

# # Sourcing
RUN cd ~/ros_ws/ && \
    rm -rf build devel && \
    catkin_make"

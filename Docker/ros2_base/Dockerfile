FROM osrf/ros:eloquent-desktop
#ros:eloquent-ros-core-bionic
#osrf is the experimental version

SHELL ["/bin/bash","-c"]

#since we are in a minimal docker enviroment 
#https://stackoverflow.com/questions/39760663/docker-ubuntu-bin-sh-1-locale-gen-not-found
RUN apt-get clean && apt-get update && apt-get install -y locales 
RUN locale-gen en_US.UTF-8

RUN /bin/bash -c  "echo 'source /opt/ros/eloquent/setup.bash' >>  ~/.bashrc " 

RUN sudo locale-gen en_US en_US.UTF-8 
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
RUN export LANG=en_US.UTF-8

RUN sudo apt update && sudo apt install curl gnupg2 lsb-release 
RUN   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN sudo apt update && sudo apt install -y \
  build-essential \
  apt-utils \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-lark-parser \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  vim \
  wget

# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

# install Fast-RTPS dependencies
RUN sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

#create workspace
RUN  cd /home && mkdir /home/ws_ros2 
RUN  export COLCON_WS=/home/ws_ros2 
RUN  mkdir  /home/ws_ros2/src

ENTRYPOINT ["bash"] 
WORKDIR /home
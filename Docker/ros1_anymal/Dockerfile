FROM ros1_base:latest

RUN apt-get update

RUN sudo apt-get install -y apt-utils
RUN mkdir -p /home/catkin_ws/src

RUN cd /home/catkin_ws/src/ && git clone https://github.com/ANYbotics/anymal_b_simple_description.git

RUN source /opt/ros/melodic/setup.bash && \
		cd /home/catkin_ws/ && \
		catkin_make

RUN apt-get install -y python-rospkg
RUN apt-get install -y ros-melodic-moveit 

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
RUN apt update

RUN apt install -y ros-melodic-desktop-full
RUN apt-get install -y ros-melodic-ros-control ros-melodic-ros-controllers
RUN echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc

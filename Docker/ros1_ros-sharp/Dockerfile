FROM ros1_base:latest

RUN mkdir -p /home/ros-sharp_ws/src
RUN source /opt/ros/melodic/setup.bash && \
    cd /home/ros-sharp_ws && catkin_make

RUN apt-get -y install ros-melodic-rosbridge-server 
RUN source /home/ros-sharp_ws/devel/setup.bash

RUN cd /home/ && git clone -b ros_docker_dev https://github.com/luchspeter/3D_Vision_AR_RobotVis.git
RUN mv /home/3D_Vision_AR_RobotVis/ROS/file_server /home/ros-sharp_ws/src
RUN rm -r -f /home/3D_Vision_AR_RobotVis

RUN source /opt/ros/melodic/setup.bash && \
		source /home/ros-sharp_ws/devel/setup.bash && \
		cd /home/ros-sharp_ws && \
		catkin_make
RUN apt-get install -y python-twisted
RUN apt-get update
RUN apt install -y python3-pip
RUN apt-get install -y python-pip 
RUN pip install autobahn tornado bson pymongo pillow
RUN echo "HELLO"
RUN echo 'source /home/ros-sharp_ws/devel/setup.bash' >> ~/.bashrc
RUN echo 'source /home/ros-sharp_ws/devel/setup.bash' >> ~/.bashrc 
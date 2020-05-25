
FROM ros1_ur:latest

#INSTALL ROSSHARP

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

#add lauch to ur_package and rebuild ws
RUN source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make

RUN mkdir -p /home/catkin_ws/src/universal_robot/ur_rossharp
COPY ./ur_rossharp/* /home/catkin_ws/src/universal_robot/ur_rossharp/
COPY ./worlds/* /usr/share/gazebo-9/worlds/

RUN source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make

RUN sudo apt-get install -y python-pip 
RUN pip install scipy

RUN echo 'source /home/ros-sharp_ws/devel/setup.bash' >> ~/.bashrc

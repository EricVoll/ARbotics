# Universal robot container
FROM ros1_ur:latest

# Creating rossharp workspace
RUN mkdir -p /home/ros-sharp_ws/src
RUN source /opt/ros/melodic/setup.bash && \
    cd /home/ros-sharp_ws && catkin_make
RUN apt-get -y install ros-melodic-rosbridge-server 
RUN source /home/ros-sharp_ws/devel/setup.bash
# Rossharp fileserver fork to allow dynamic loading of robots
RUN cd /home/ && git clone -b ros_docker_dev https://github.com/luchspeter/ARbotics.git
RUN mv /home/ARbotics/ROS/file_server /home/ros-sharp_ws/src
RUN rm -r -f /home/ARbotics

RUN source /opt/ros/melodic/setup.bash && \
		source /home/ros-sharp_ws/devel/setup.bash && \
		cd /home/ros-sharp_ws && \
		catkin_make

# Create catkin_ws 
RUN source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make
# Install universal robot package in container
RUN mkdir -p /home/catkin_ws/src/universal_robot/ur_rossharp
COPY ./ur_rossharp/* /home/catkin_ws/src/universal_robot/ur_rossharp/
# Include a custom gazebo world to avoid the universal robot bugging into the ground plane
COPY ./worlds/* /usr/share/gazebo-9/worlds/
# Rebuild and source the catkin_ws
RUN source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make

RUN sudo apt-get install -y python-pip 
RUN pip install scipy

RUN echo 'source /home/ros-sharp_ws/devel/setup.bash' >> ~/.bashrc

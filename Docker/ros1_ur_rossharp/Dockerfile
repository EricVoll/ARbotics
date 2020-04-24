
FROM ros1_ur:latest

COPY ./rossharp_install.sh /
RUN dos2unix /ur_install.sh

RUN apt-get update 

RUN bash -c "chmod +x /rossharp_install.sh  && /rossharp_install.sh "

RUN bash -c "source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make"

RUN mkdir -p /home/catkin_ws/src/universal_robot/ur_rossharp
COPY ./ur_rossharp/* /home/catkin_ws/src/universal_robot/ur_rossharp/

RUN bash -c "source /opt/ros/melodic/setup.bash && \
source /home/catkin_ws/devel/setup.bash && \
source /home/ros-sharp_ws/devel/setup.bash && \
cd /home/catkin_ws && catkin_make"

RUN bash -c "echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc "
WORKDIR /home

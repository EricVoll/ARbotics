FROM ros2_base:1.0
#create workspace
RUN  cd /home && mkdir -p /home/ws_ros2/src/moveit2
RUN  export COLCON_WS=/home/ws_ros2 

#install moveit2
RUN /bin/bash -c " cd  /home/ws_ros2/src && git clone https://github.com/ros-planning/moveit2.git -b master "


RUN /bin/bash -c "  source /opt/ros/eloquent/setup.bash && \
  printenv | grep -i ROS &&\
  cd /home/ws_ros2/src/ && \
  ls && \
  sudo vcs import < moveit2/moveit2.repos || true "
RUN /bin/bash -c "  source /opt/ros/eloquent/setup.bash && \
  cd /home/ws_ros2  &&\
  rosdep install -r --from-paths . --ignore-src --rosdistro eloquent -y"

# RUN /bin/bash -c " cd  /home/ws_ros2 && \
#    rosdep install -r --from-paths . --ignore-src --rosdistro eloquent -y "
# # #source workspace
RUN /bin/bash -c "  source /opt/ros/eloquent/setup.bash && cd  /home/ws_ros2 && colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release "
RUN /bin/bash -c  "echo 'source /home/ws_ros2/install/local_setup.bash' >>  ~/.bashrc " 
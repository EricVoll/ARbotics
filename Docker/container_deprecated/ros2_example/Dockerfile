FROM ros2_base:1.0
RUN cd /home
RUN  cd /home && mkdir /home/ws_test 
RUN  export BRIDGE_WS=/home/ws_test
RUN  mkdir  /home/ws_test/src
RUN /bin/bash -c "cd /home/ws_test &&  colcon build"
RUN /bin/bash -c "cd  /home/ws_test/src && git clone https://github.com/ros2/examples.git -b eloquent "
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash && cd  /home/ws_test && colcon build "

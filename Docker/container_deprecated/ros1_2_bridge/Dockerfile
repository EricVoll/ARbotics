FROM ros1_2_example

#create workspace bridge
RUN  cd /home && mkdir /home/ws_bridge 
RUN  export BRIDGE_WS=/home/ws_bridge
RUN  mkdir  /home/ws_bridge/src
RUN /bin/bash -c " cd /home/ws_bridge &&  colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release "
RUN /bin/bash -c  "echo 'source /home/ws_bridge/install/local_setup.bash' >>  ~/.bashrc " 




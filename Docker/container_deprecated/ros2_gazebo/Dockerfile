FROM ros2_base:1.0
RUN echo "HALLO"
COPY gazebo_install.sh /
RUN chmod +x /gazebo_install.sh && /gazebo_install.sh
RUN mkdir /home/host_dev
WORKDIR /home



FROM ros2_base:1.0

COPY gazebo_install.sh /
RUN chmod +x /gazebo_install.sh && /gazebo_install.sh

ENTRYPOINT ["bash"] 
WORKDIR /home

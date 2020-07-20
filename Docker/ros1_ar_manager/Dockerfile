FROM ros1_base:latest

SHELL ["/bin/bash", "--login", "-c"]

# Update
RUN apt-get update
RUN apt-get install -y apt-utils

# Install python depedencies 
RUN apt-get install -y python3-pip 
RUN pip3 install rospkg catkin_pkg docker
RUN pip3 install dataclasses dataclasses-json PyYaml
RUN pip3 install flask_restful APScheduler coloredlogs 
RUN pip3 install -U flask-cors

# Blocks using cache to always repull the repository 
RUN echo "HELLO"
RUN git clone -b ros_docker_dev https://github.com/luchspeter/ARbotics.git

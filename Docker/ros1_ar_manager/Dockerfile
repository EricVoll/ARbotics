FROM ros1_base:latest

SHELL ["/bin/bash", "--login", "-c"]
RUN apt-get update

RUN apt-get install -y apt-utils
RUN apt-get install -y python3-pip 

RUN pip3 install rospkg catkin_pkg docker
RUN pip3 install dataclasses dataclasses-json PyYaml

RUN git clone -b ros_docker_dev https://github.com/luchspeter/3D_Vision_AR_RobotVis.git 
RUN pip3 install flask_restful APScheduler coloredlogs
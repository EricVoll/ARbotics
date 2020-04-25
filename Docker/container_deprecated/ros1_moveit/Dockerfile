FROM ros1_base:latest

COPY ./moveit_install.sh /
RUN dos2unix /moveit_install.sh

COPY ./copy /home/
RUN chmod +x /moveit_install.sh && /moveit_install.sh

WORKDIR /home

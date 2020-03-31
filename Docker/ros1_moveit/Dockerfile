FROM ros1_base:1.0

COPY ./moveit_install.sh /
COPY ./copy /home/
RUN chmod +x /moveit_install.sh && /moveit_install.sh

WORKDIR /home

FROM ros1_base:latest

COPY ./ur_install.sh /
RUN dos2unix /ur_install.sh

RUN chmod +x /ur_install.sh && /ur_install.sh

WORKDIR /home

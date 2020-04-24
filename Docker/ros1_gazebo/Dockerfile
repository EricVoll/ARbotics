
FROM gazebo

COPY ./gazebo_install.sh /
RUN dos2unix /gazebo_install.sh

RUN mkdir /newdir
RUN /bin/bash -c 'chmod +x /gazebo_install.sh  && cd / && ./gazebo_install.sh'
RUN mkdir /test

RUN echo "Completly build faslsch"

# # COPY ./ur_install.sh /
# # RUN chmod +x /ur_install.sh && /ur_install.sh
# # 
# RUN echo "working"
# COPY ./ur_install.sh /
# RUN chmod +x /ur_install.sh && /ur_install.sh
# WORKDIR /home
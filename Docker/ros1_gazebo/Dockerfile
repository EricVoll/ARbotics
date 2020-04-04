
FROM gazebo:1.0 

COPY ./ur_install.sh /
RUN mkdir /newdir
RUN /bin/bash -c 'chmod +x /ur_install.sh  && cd / && ./ur_install.sh'
RUN mkdir /test

RUN echo "Completly build faslsch"

# # COPY ./ur_install.sh /
# # RUN chmod +x /ur_install.sh && /ur_install.sh
# # 
# RUN echo "working"
# COPY ./ur_install.sh /
# RUN chmod +x /ur_install.sh && /ur_install.sh
# WORKDIR /home
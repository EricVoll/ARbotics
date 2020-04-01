FROM ros1_base:1.0

RUN echo "HELLO"
COPY ./rossharp_install.sh /
RUN apt-get update 

RUN bash -c "chmod +x /rossharp_install.sh  && /rossharp_install.sh "


#RUN chmod +x /entrypoint.sh 

#ENTRYPOINT ["/entrypoint.sh"]


WORKDIR /home

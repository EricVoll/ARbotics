FROM ros1_base:latest

COPY ./rossharp_install.sh /
RUN dos2unix /rossharp_install.sh

RUN apt-get update 

RUN bash -c "chmod +x /rossharp_install.sh  && /rossharp_install.sh "

WORKDIR /home

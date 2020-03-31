FROM ros1_base:1.0

COPY ./rossharp_install.sh /

RUN chmod +x /rossharp_install.sh && /rossharp_install.sh

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh 

ENTRYPOINT ["/entrypoint.sh"]


WORKDIR /home

FROM python:3.8.3-alpine3.12

RUN pip install Flask
RUN pip install -U flask-cors

ARG CACHEBUST=1
ARG CACHEBUST=2
COPY ./flask_test.py /home/
COPY ./AR-Manager_swagger_cfg.json /home/AR-Manager_swagger_cfg.json

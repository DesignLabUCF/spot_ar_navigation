FROM python:3.7-slim

# Install the API wheels and other requirements.
COPY docker-requirements.txt prebuilt/*.whl ./

RUN python3 -m pip install -r docker-requirements.txt --find-links .
RUN python3 -m pip install pandas

RUN mkdir -p /data/transform_recordings

COPY transform_recorder_service.py /app/
WORKDIR /app

### SDL
ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

#ENTRYPOINT ["python3", "/app/transform_recorder_service.py"]
# Default arguments for running on the CORE I/O
#CMD [ "192.168.50.3", "--host-ip=192.168.50.5"]

ENTRYPOINT ["python3", "/app/transform_recorder_service.py", "192.168.50.3", "--host-ip=192.168.50.5"]


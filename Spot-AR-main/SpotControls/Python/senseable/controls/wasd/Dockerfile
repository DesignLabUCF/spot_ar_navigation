FROM python:3.8.10
#FROM ubuntu:18.04

ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

# Remnant from a different library, but keeping for safety
# https://github.com/docker/build-push-action/issues/504
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /temp
ADD requirements.txt .
ADD prebuilt ./prebuilt
RUN python -m pip install -r requirements.txt --find-links .
WORKDIR /
RUN rm -r /temp

# Helpful for debugging
RUN apt-get update
RUN apt-get install usbutils -y

# Evdev library in python
RUN pip install evdev

WORKDIR /
ADD wasd.py .

#CMD ["python", "./wasd.py", "192.168.80.3", "21003", "write_test.txt"]
#CMD ["python3.8", "./wasd.py"]
#CMD ["python", "./wasd.py"]
CMD ["python", "./wasd.py", "192.168.80.3"]
#CMD ["/bin/bash"]
#ENTRYPOINT ["tail", "-f", "/dev/null"]


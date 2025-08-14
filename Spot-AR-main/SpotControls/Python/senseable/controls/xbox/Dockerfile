FROM python:3.8.10

ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

WORKDIR /temp
ADD requirements.txt .
ADD prebuilt ./prebuilt
RUN pip install -r requirements.txt --find-links .
#RUN pip install --no-index --no-deps prebuilt/*.whl
RUN pip install -f prebuilt/*.whl
WORKDIR /
RUN rm -r /temp

RUN apt-get update

# For debugging
RUN apt-get install usbutils -y

# Bluetooth support needed for wireless controllers - https://atar-axis.github.io/xpadneo/
RUN apt-get install -y bluez bluetooth
RUN apt-get install pulseaudio-module-bluetooth -y
RUN pip install pybluez

# Add functionality so Ubuntu recognizes the xbox one controller - https://www.thetechedvocate.org/how-to-set-up-an-xbox-one-controller-in-ubuntu/
RUN apt-get install xboxdrv joystick -y

# Python library for recognizing ubunutu input events
RUN pip install evdev

# Add in necessary scripts
WORKDIR /
ADD docker-entrypoint.sh .
ADD xbox.py .
ADD looptest.py .


WORKDIR /
#ENTRYPOINT /bin/bash -c ./docker-entrypoint.sh && cd / && python xbox.py 192.168.80.3
ENTRYPOINT /bin/bash -c ./docker-entrypoint.sh && cd / && python xbox.py 192.168.80.3 --use-bluetooth True

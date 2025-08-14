FROM python:3.8.10

ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

RUN pip install opencv-python-headless # Saves space while installing dependencies
RUN pip install numpy
RUN pip install Pillow
RUN pip install bosdyn.client
RUN pip install bosdyn.api

ADD listen_for_call.py .

CMD ["python", "./listen_for_call.py", "--avoid-obstacles", "true", "--spot-ip", "192.168.80.3", "--port", "21006", "192.168.80.3"]


FROM python:3.8.10

ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

ADD spot_image.py .

RUN pip install opencv-python-headless # Saves space while installing dependencies
RUN pip install numpy
RUN pip install scipy
RUN pip install bosdyn.client
RUN pip install bosdyn.api

ADD listen_for_image_call.py .

#CMD ["python", "./listen_for_image_call.py", "192.168.200.45", "21001", "back_fisheye_image", "PIXEL_FORMAT_RGB_U8"]
CMD ["python", "./listen_for_image_call.py", "192.168.80.3", "21001", "back_fisheye_image", "PIXEL_FORMAT_RGB_U8"]


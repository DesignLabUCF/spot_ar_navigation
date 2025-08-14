FROM python:3.8.10

WORKDIR /temp
ADD requirements.txt .
ADD prebuilt ./prebuilt
RUN pip install -r requirements.txt --find-links .
WORKDIR /
RUN rm -r /temp

# Helpful for debugging
RUN apt-get update
RUN apt-get install usbutils -y
RUN apt-get install -y rfkill

# Install bluetooth support - https://stackoverflow.com/questions/28868393/accessing-bluetooth-dongle-from-inside-docker
RUN apt-get install -y bluez bluetooth

# Python bluetooth library
RUN pip install pybluez

# Have to run these after starting. Look into entrypoint
ADD docker-entrypoint.sh .

WORKDIR /
#ADD leave_running.py .
ADD *.py .


#CMD ["python", "./leave_running.py", "192.168.80.3"]
#ENTRYPOINT ["/bin/bash", "-c", "./docker-entrypoint.sh"]
ENTRYPOINT /bin/bash -c ./docker-entrypoint.sh && python detect_devices.py 192.168.80.3
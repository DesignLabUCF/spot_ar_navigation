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
# Plays mp3 in terminal - 'mpg [FILE_NAME]''
RUN apt-get install mpg123 -y

# Install bluetooth support - https://stackoverflow.com/questions/28868393/accessing-bluetooth-dongle-from-inside-docker
RUN apt-get install -y bluez bluetooth
#RUN apt-get install -y bluez-tools
# Install manually latest version
#RUN apt-get install wget -y
#RUN mkdir /temp
#WORKDIR /temp
#RUN wget http://www.kernel.org/pub/linux/bluetooth/bluez-5.66.tar.xz
#RUN tar xf bluez-5.66.tar.xz

# Python bluetooth library
RUN pip install pybluez

# Fix the can pair but not connect issue - https://askubuntu.com/questions/831732/bluetooth-paired-but-not-connected-ubuntu-16-04
RUN apt-get install pulseaudio-module-bluetooth -y
#RUN pactl load-module module-bluetooth-discover

# Ubunutu audio libraries - https://pypi.org/project/PyAudio/
RUN apt-get update -y
RUN apt-get install portaudio19-dev -y
RUN apt-get install python3-pyaudio -y
RUN pip install PyAudio

# Python audio library 
#RUN pip install playsound
#RUN pip install pygobject
#RUN apt-get install python3-gi
#RUN apt-get install libgstreamer1.0-dev

# Add audio files
WORKDIR /
ADD test_audio.mp3 .
ADD beep.mp3 .

# Have to run these after starting. Look into entrypoint
ADD docker-entrypoint.sh .

WORKDIR /
ADD *.py .

ENTRYPOINT /bin/bash -c ./docker-entrypoint.sh && python connect_and_play_sound.py 192.168.80.3

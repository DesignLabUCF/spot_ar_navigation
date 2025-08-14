#ARG UBUNTU_VERSION=18.04
## Use the official ubuntu:18.04 image as the parent image
#FROM ubuntu:${UBUNTU_VERSION}

# Requires GPU usage for Kinect depth sensor
# https://dev.bostondynamics.com/docs/payload/coreio_documentation.html#working-with-the-gpu
# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base
# Seems to use Arm64, but is based on Ubunutu 20.04. Kinect requires 18.04 so need to figure out how to get an 18.04 instance of it potentially.
FROM nvcr.io/nvidia/l4t-base:r35.1.0


## SDL: Spot params
ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

## Set the working directory to /app
WORKDIR /app

ARG ARCH=arm64

ADD setup-ubuntu.sh /app
ADD sources.list /app

## Tell Ubuntu non-interactive install
ARG DEBIAN_FRONTEND=noninteractive

RUN ./setup-ubuntu.sh ${ARCH}

RUN apt-get install -y ca-certificates
RUN update-ca-certificates

## SDL: Install necessary python packages to run Kinect examples
RUN apt-get install python3-pip -y
#RUN apt-get install python3-venv -y
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install Pillow

## SDL: Copy over python code
WORKDIR /app
ADD src src
WORKDIR /
COPY kinect_scan_service.py /app/
COPY kinect_read.py /app/
COPY debug_leave_running.py /app/
WORKDIR /app

## SDL: Create necessary directories
RUN mkdir -p /data/kinect

## SDL: Install k4a (https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/python/k4a/docs/building.md)
RUN cp /usr/lib/aarch64-linux-gnu/libk4a1.4/libdepthengine.so.2.0 src/python/k4a/src/k4a/_libs
RUN cp /usr/lib/aarch64-linux-gnu/libk4a.so src/python/k4a/src/k4a/_libs

#Unsure start
RUN cp /usr/lib/aarch64-linux-gnu/libk4a.so.1.4 src/python/k4a/src/k4a/_libs
RUN cp /usr/lib/aarch64-linux-gnu/libk4a.so.1.4.1 src/python/k4a/src/k4a/_libs
#Unsure end

## SDL: Install OpenGL (dependancy for azure kinect sdk - depth engine needs it) - Version 4.4 required (per https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/973#issuecomment-571261360)
RUN apt-get update
RUN apt-get upgrade
RUN apt-get install mesa-utils -y
RUN apt-get install libglu1-mesa-dev -y

WORKDIR /app/src/python/k4a

# Build k4a library
RUN python3 -m pip install wheel
# Build size: 7231335 
RUN ./build_wheel.csh
RUN python3 -m pip install build/k4a-0.0.2-py3-none-any.whl

# Makes numpy work in ARM. Not sure why it's needed as we already got the ARM version of numpy but whatever, it works.
ENV OPENBLAS_CORETYPE ARMV8

# TODO: Depth engine needs OpenGL i believe (see documentation)
# Depth engine requires a display value set for OpenGL to function properly
ENV DISPLAY 0

# Library's cant be found, so hardcore them to where python installation copies them to
#ENV LD_LIBRARY_PATH /usr/local/lib/python3.6/dist-packages/k4a/_libs/
ENV LD_LIBRARY_PATH /usr/local/lib/python3.8/dist-packages/k4a/_libs/

# SDL: Apply 
#ENV LANG C.UTF-8
#ENV PATH /usr/local/bin:$PATH

## SDL: Install the API wheels and other requirements for Boston dynamics service.
WORKDIR /
COPY docker-requirements.txt prebuilt/*.whl ./
RUN python3 -m pip install -r docker-requirements.txt --find-links .

## SDL: Data directory for output
RUN mkdir -p /data/kinect

## SDL: For debugging gpu link
RUN apt-get install lshw -y

WORKDIR /app

#ENTRYPOINT ["tail", "-f", "/dev/null"]
#ENTRYPOINT ["python3", "/app/debug_leave_running.py"]
ENTRYPOINT ["python3", "/app/kinect_scan_service.py", "192.168.50.3", "--host-ip=192.168.50.5"]
### Pull the base ROS humble image - https://github.com/osrf/docker_images/blob/3f4fbca923d80f834f3a89b5960bad5582652519/ros/humble/ubuntu/jammy/desktop/Dockerfile
### Then add on Unity-TCP endpoint stuff - https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros-environment

FROM ros:humble-ros-base

######### Unity-ROS

WORKDIR /

# Debug
RUN apt-get update
RUN apt-get install iftop -y

# Not sure if this does anything, but keeping it untill I can confirm it does not
RUN apt-get install ros-humble-rmw-fastrtps-cpp -y
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

# Add python3.8 and pip
RUN apt-get update
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:deadsnakes/ppa
RUN apt-get update
RUN apt install -y python3.8
RUN apt-get install -y python3-pip

# Bosdyn libraries
WORKDIR /temp
ADD requirements.txt .
ADD prebuilt ./prebuilt
#RUN pip install -r requirements.txt --find-links .
RUN pip install -r requirements.txt --find-links .
#RUN pip install -f prebuilt/*.whl
RUN pip install -f prebuilt/*.whl
WORKDIR /
RUN rm -r /temp

# Minimal OpenCV/Image manipulation installation
#RUN pip install opencv-python-headless
RUN pip install opencv-python
RUN apt-get update
RUN apt-get install ros-humble-cv-bridge -y
RUN pip install numpy --upgrade # https://stackoverflow.com/questions/68916893/typeerror-numpy-dtypemeta-object-is-not-subscriptable
#RUN pip install scipy # TODO replace this in the future since it is only used for a single image rotation function

# ROS_DISTRO set by base image. Install test packages normally found in the Humble Desktop variant (which is not availble for ARM64 so we need to supply them ourself). Have in both Dockerfiles to keep consistent.
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*
# launch ros package
CMD /bin/bash

### Create ROS workspace with colcon. Colcon already installed by base images
RUN mkdir -p /senseable_ws/src
WORKDIR /senseable_ws
RUN git clone https://github.com/ros2/examples src/examples -b humble

# Copy ROS2 packages into workspace
COPY ros2_packages/ /senseable_ws/src

# Check out ROS-TCP-Endpoint, ROS2 version
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint /senseable_ws/src/ros_tcp_endpoint -b ROS2v0.7.0

# Overwrite the server.py ROS-TCP-Endpoint file to add the remove_subscribers functionality to Unity that seems to be unfinished - https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues/166
COPY TCP-Endpoint-Fix/server.py /senseable_ws/src/ros_tcp_endpoint/ros_tcp_endpoint/server.py

# Change to workspace on sign in
RUN echo "cd /senseable_ws" >> ~/.bashrc

# Source it and build in same bash. Shares env vars between them.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \ 
#&&    colcon build --symlink-install \
&&    colcon build \
&&    source install/setup.bash"

######### Spot 

# Authentication argument
ARG BOSDYN_CLIENT_USERNAME
ARG BOSDYN_CLIENT_PASSWORD
ENV BOSDYN_CLIENT_USERNAME $BOSDYN_CLIENT_USERNAME
ENV BOSDYN_CLIENT_PASSWORD $BOSDYN_CLIENT_PASSWORD

######### Launch container 

# Reference script with commands to source workspace
COPY spot_ros_entrypoint.sh /
RUN ["chmod", "+x", "/spot_ros_entrypoint.sh"]

# Run commands
WORKDIR /senseable_ws
ENTRYPOINT ["/spot_ros_entrypoint.sh"]
CMD /bin/bash
### Pull the base ROS humble image
### https://github.com/osrf/docker_images/blob/3f4fbca923d80f834f3a89b5960bad5582652519/ros/humble/ubuntu/jammy/desktop/Dockerfile
# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-base
### https://hub.docker.com/r/arm64v8/ros/tags
#FROM arm64v8/ros:humble-ros-base

# ROS_DISTRO set by base image. Install test packages normally found in the Humble Desktop variant (which is not availble for ARM64 so we need to supply them ourself)
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*
# launch ros package
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
CMD /bin/bash

### Add the Spot ROS wrapper on top
# TODO
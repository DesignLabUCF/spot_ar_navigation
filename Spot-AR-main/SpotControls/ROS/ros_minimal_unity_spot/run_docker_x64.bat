: docker run -it --rm -p 10000:10000 ros-minimal-unity-x64 /bin/bash

: Should use 0.0.0.0 IP if running tcp endpoint in container
: ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

: docker run -it --rm -p 10000:10000 ros-minimal-unity-spot-x64 ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
docker run -it --rm -p 10000:10000 ros-minimal-unity-spot-x64
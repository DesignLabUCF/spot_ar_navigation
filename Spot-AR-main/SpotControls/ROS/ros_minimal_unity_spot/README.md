# HoloLens - ROS2

https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md

## HoloLens to Windows/Ubunutu For Testing

Communication will all be routed through the TCP Endpoint container. Because of this, it is the only one that needs ports exposed or net host (if Ubunutu). 

### Container 1 - TCP Endpoint

First, launch a container containing the Unity tcp endpoint. This handles traffic between Unity and ROS2. We are using the default port 10000 here.

```
docker run -it --rm -p 10000:10000 ros-minimal-unity-spot-x64 /bin/bash
```

On this container run the endpoint. IP of 0.0.0.0 listens to all traffic.

```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```
### Container 2 - Velocity Driver

This container subscribes to the velocity topic and then transfer it directly to Spot controls that set it's velocity. Note: this will directly run the python script as opposed to launching it through ROS2. This is necessary for passing the host name argument to the Spot authentication from the SDK.

```
docker run -it --rm ros-minimal-unity-spot-x64 python3 src/unity_robotics_demo/unity_robotics_demo/spot_velocity_controller.py 192.168.80.3
```

### Container 3 or Unity

Now we need to publish some velocities to the velocity topic. You can do this Unity, or by running the debug test script we configured in ROS already.

```
docker run -it --rm ros-minimal-unity-spot-x64 ros2 run unity_robotics_demo debug_velocity_publisher
```

## HoloLens to Spot CORE IO

No discovery server is needed. Simply run the configuration of containers specifice in the docker-compose.yml file. ROS-TCP endpoint container should run as host.

### Connecting to CORE IO Wifi
0. Set up CORE IO wifi
1. Add wifi network to the HoloLens (WPA2 personal)
2. On the HoloLens, set up IP information so the device recieves a valid IP address when connecting

### In Unity

Set the ROS2 IP to 192.168.50.5 (the CORE IO IP) and the port to 21150 (the port we specified for the TCP-Endpoint ROS2 package).

### Notes

Run all containers except the TCP endpoint NOT on host and WITHOUT any ports exposed. Looks like they will do all communication through the TCP endpoint node, which SHOULD run with --net=host.
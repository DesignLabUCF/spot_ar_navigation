# HoloLens - ROS2

https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md

For now, have to always source when entering a container. Need to move this to a launch script so it is done automatically...

```
source install/setup.bash
```

First, launch a container containing the Unity tcp endpoint. This handles traffic between Unity and ROS2. We are using the default port 10000 here.

```
docker run -it --rm -p 10000:10000 ros-minimal-unity-x64 /bin/bash
```

On this container run the endpoint. IP of 0.0.0.0 listens to all traffic.

```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Run a second container in another terminal. This one will be our subscriber/publisher

```
docker run -it ros-minimal-unity-x64 /bin/bash
```

The following command lists the available executables to run our python ROS2 scripts.

```
ros2 pkg executables
```

We can run our chosen executable by pasting the executable and topic directly onto he end of a 'ros2 run' command.

```
ros2 run [PACKAGE_NAME] [SCRIPT NAME]
ros2 run unity_robotics_demo transform_publisher
```

This will list the available topics that are currently active we can listen to directly if we want, and then actually listen to that topic

```
ros2 topic list
ros2 topic echo transform
ros2 topic echo [TOPIC_NAME]
```
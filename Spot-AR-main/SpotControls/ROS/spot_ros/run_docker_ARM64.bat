:: Test variant
docker run -it --platform linux/arm64 spot-ros-arm64 ros2 launch demo_nodes_cpp talker_listener.launch.py
:: Deploy variant
:docker run -it --platform linux/arm64 spot-ros-arm64 /bin/bash
:: Launch Spot Driver
:docker run -it --platform linux/arm64 spot-ros-arm64 ros2 launch spot_driver spot_driver.launch.py
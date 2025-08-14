:: Test variant
:docker run -it --cap-add=NET_ADMIN spot-ros2-x64 ros2 launch demo_nodes_cpp talker_listener.launch.py
:: Deploy variant
docker run -it --cap-add=NET_ADMIN spot-ros2-x64 /bin/bash
:: Launch Spot Driver
:docker run -it --cap-add=NET_ADMIN spot-ros2-x64 ros2 launch spot_driver spot_driver.launch.py
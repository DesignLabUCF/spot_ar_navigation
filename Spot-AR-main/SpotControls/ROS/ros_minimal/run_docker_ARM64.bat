:: Test variant
docker run -it --platform linux/arm64 ros-minimal-arm64 ros2 launch demo_nodes_cpp talker_listener.launch.py
:: Deploy variant
:docker run -it --platform linux/arm64 ros-minimal-arm64 /bin/bash
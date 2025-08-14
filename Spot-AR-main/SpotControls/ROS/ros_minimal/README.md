https://docs.ros.org/en/rolling/Tutorials.html


# Nodes

Used to publish/subscribe to send data across terminals.

Can run talker/listener with .bat files.

Can do custom echos/publishes by doing the following:

#### Subscriber

1. Launch a docker container in /bin/bash
2. Run 'ros2 topic list -t' to get a list of available topics. Default talker/listener test uses /chatter
3. Run 'ros2 topic echo /chatter' to get it to subscribe to the topic

#### Publisher

1. Launch a docker container in /bin/bash
2. Run 'ros2 topic pub /chatter std_msgs/msg/String "{data: 1}"'

# Services

TODO
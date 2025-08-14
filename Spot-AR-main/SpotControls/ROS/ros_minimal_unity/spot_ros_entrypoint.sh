#!/bin/bash
set -e

# Source the ros2 spot environment,
#source "/home/senseable_ws/install/setup.bash"

source "/senseable_ws/install/setup.bash"
echo 'source "/senseable_ws/install/setup.bash"' >> ~/.bashrc

exec "$@"

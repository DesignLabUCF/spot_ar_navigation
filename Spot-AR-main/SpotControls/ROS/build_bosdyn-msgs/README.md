### What is this?

TODO TEST THIS. IT USED TO BE BUILT INTO THE OTHER IMAGE, BUT IS NOW SEPERATED TO ACCOMODATE SPOT TALKING TO x64 SYSTEMS.

Building this Docker Image will build the bosdyn_msgs .deb file (https://github.com/bdaiinstitute/spot_ros2/tree/main) that you then use in the Spot ROS2 wrapper. The publically avilable .deb files don't always seem up to date or accomodating to Spot's with an ARM64 CORE IO. It will build for whatever architecture the Docker container is configured for/running. View the .bat file commands for examples of the arguments/commands needed to build for both system types.

An ARM64 and AMD64 (x64) variant are provided.

Run this container, then grab the .deb from the base directory in the Docker Desktop GUI. Copy-paste it into the spot-ros directory so it's Dockerfiles can access them for copying to the container.


## Why Use ROS2 Wrapper?

More versatility. Other systems more easily communicate with Spot without going directly to the Spot SDK. This is especially important with other control methods like the HoloLens 2. This isn't just for sending commands to Spot though, its to improve Spots ability to send the user information about its current status, vision, and other real-time data. When in the field, Spot's onboard computer will be relied on to do the heavy lifting in computing, and ROS2 is a proven relatively lightweight way to handle networking and communication.

## Current State

x64 ROS can talk and listen to itself in different terminals

ARM64 ROS talk and listen to itself in different terminals (only works when running on Spot)

x64 ROS and ARM64 ROS can NOT communicate with test scripts or multicast in PC vs Spot test. Might have something to do with the different base images, but both x64 desktop pc and Spot NEED these different base images to function correctly individually. Need to figure out for testing. NOTE: HoloLens 2 is an ARM64 system so that may be useful in future.

I think it might be a different systems thing and not a different base image thing. I got the x64 version to build and run on spot (with no error for some reason, not sure why this worked) and it was able to run by itself fine (in one terminal AND also in seperate terminals), but still couldnt communicate with pc. Maybe related to ROS2 DDS?

7/24/23 Update:
	We have identified that it is an issue with Windows firewall preventing it from recieving ROS2 communication. When testing with two Linux machines, they were able to send ROS2 talk-listener and multicast send/recieve messaged between eachother. Investigating this now (https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through and https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html). 

Goals when we get back:
1. Get ROS on Spot/PC to talk succesfully
X 2a. Build x64 ROS wrapper .deb
2b. Test x64 ROS wrapper .deb
3. Do we need ROS installed to send ROS actions/commands?
X 4. Test x64 ROS2 build on Spot (SEEMS TO WORK FOR SOME REASON? NO ERRORS? Still no success in the systems talking tho. Just Ros2, no wrapper)
5. Get ROS2 running on Spot in the SENSEable extension (Launches, but container gets locked (it seems at least) during colcon process. Running two seperate containers using the image that comes along with the extension installation works though)
6. Clean up ROS2 docker files and test
7. Get Spot Follow from Spot-sdk running
8. Get ROS-TCP in Unity running on PC
9. Get ROS-TCP in Unity running on HoloLens 2
10. Explore colcon build/colcon symlink stuff/ROS2 config files
11. Explore BD ROS2 wrapper spot driver

8/3/23 Update:
	* --net=host does not work on Windows
	* Spot does not relay container connections between himself and the CORE IO (so far). We have been able to get Linux container to talk to a container on the CORE IO using ROS if the CORE IO has a wifi dongle attached and is running on its own wifi. This means it has it's own IP address we need to consider, but for now ROS can detect stuff just fine.
	* Still only works when both are running --net=host, however we did tcpdump on the container running on the non-spot Linux and found that the ports it was using when ROS_DOMAIN_ID=0 for talker/listener was 1 (the 0th one was the tcpdump container), and it was using port 7412 and 7413. We need to find a way to better control this participant ID (ideal solution), or have it use a wide range of ports and hope it stays within the range (bad solution). Participant ID seems to be determined by order of ROS containers launched on same machine. First one is 0, second one in 1, and so on.
	* We decided that passing in a range of ports is acceptable, but this still does not work as it it uses the ephimeral ports for outgoing traffic. For example: running on the Linux machine with --net=host will sometime used an outgoing port of around 48000 to the incoming port of 7413. It may not work when passing in specific ports because this outgoing port is not opened, however this port is in a wide range and auto assigned by the computer.
	* We restricted the ephimeral port range to just 21800-21805 to so outgoing traffic on the container only used ports in that range. Confirmed this was the case on the container as well. Still did not work. Appears to be one piece of the puzzle missing still...
	* Discovery server no longer working for some reason, so cant test with that.

Goals for return from LA:
1. Resolve why colcon builds are failing in extension launch. Likely need to get our dockerfile closer to the Spot ROS2 one to prevent crashing as that one is allegedly proven to work.
2. Use ROS2 to transmit video feed
3. Figure out these containers not communicating
4. Get HoloLens 2 to be able to connect to Spot Wifi
5. Test Unity Fiducual Follow on HoloLens 2 (must run estop on PC for now)


## To Run

In our experience the ARM64 variant of ROS2 will run into issues recieving messages when running on an AMD64 system, even if run in an ARM64 container. It's best to work directly on the dog, or work on your PC then transfer the Dockerfiles to Spot to build/test after you have verified they will build without error on your PC. We recommend the second method. It'll send out messages, but recieving them seems to be a problem (even if multicast testing seems like everything is working normally https://answers.ros.org/question/300370/ros2-talker-cannot-communicate-with-listener/). For a quick guide on using a USB on Spot, see https://askubuntu.com/questions/37767/how-to-access-a-usb-flash-drive-from-the-terminal. Do everything in the persistent data directory, and be careful.

Always be weary of issues related to differences between Unix line ending characters and Windows line ending characters. Anytime you see an error related to something like "'\r'", its likely this. Do some googling on how to get around it.

Step 0 is building or accessing the bosdyn_msgs .deb package. There are availbe releases on GitHub (https://github.com/bdaiinstitute/bosdyn_msgs/releases), but they may be outdated for the current Spot SDK version of desired system architecture. View the README in the 'build_bosdyn-msgs' for more information.

First, we need ROS2 to run on Spot. View the folders 'ros_test' and 'ros_minimal_arm64' to see examples of this done on both AMD64 (our PC we worked on) and ARM64 (Our Spot's architecture). Build the 'ros_minimal_arm64' image so you have access to it for the next step. This should include Colcon and all the tools you need to test ROS2 and build up your ROS2 workspace.

Next, we need to get the ROS2 wrapper provided by Boston Dynamics to run on Spot. Getting this set up and tested may be a pain, because you might have to SSH into spot when it's on an external wifi, build the image (with the correct spot wifi IP address passed in), disconnect and switch spot to its own wifi, reconnect with SSH, THEN run the container. For testing, ROS2 actions can then be triggered using a second instance of the container on Spot, or in an AMD64 built version of the container on your PC (TODO verify this).

## Quick ROS2 Notes

TODO sourcing and launching into bash. Workspace sourcing before and after colcon builds? Colcon vs Catkin and ROS1 vs ROS2.

## Testing

First make sure two seperate instances of the ROS2 container can communicate. In two seperate cmd/terminals, run the container with /bin/bash at the end. In the first one, use the command 'ros2 multicast receive'. In the second, 'ros2 multicast send'. This tests that the two instances of ROS2 can communicate.
Source: https://answers.ros.org/question/300370/ros2-talker-cannot-communicate-with-listener/

Next, test that the talker listener test package works. Launch the container into /bin/bash and then run 'ros2 launch demo_nodes_cpp talker_listener.launch.py'. The same terminal will talk and recieve via ROS2. Confirm it is doing both by readin the console printouts. NOTE: This will NOT work on an AMD64 system even if the container is an ARM64 one. Test this by putting it on the dog via usb/ssh and then build and run the container there.
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Next, test your ROS2 Spot API Wrapper. Like the sdk examples running when running on the CORE IO, it wont be able to properly authenticate unless Spot is on his own wifi and not an external one. Like the previous test, this one will not succeed when run on an AMD64 system. Everything will launch and look like it will go smoothly, but the ROS2 components will not be able to properly recieve messages, so they will just sit there waiting forever.
https://github.com/bdaiinstitute/spot_ros2/tree/spot_msgs-v3.2.0/examples/simple_walk_forward


## ROS2 in Unity/HoloLens 2

Have to incorporate the ROS2 TCP endpoint package into your ROS2 workspace so Unity can talk to ROS (https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2). It is copied over as part of the Dockerfile.
Run the ROS2 docker container with port 21150 (we have chosen this port as it is with the Spot-friendly range):
	docker run -it -p 21150:21150 spot-ros-x64 /bin/bash
Then navigate to your workspace, source ROS2, colcon build, then source your workspace.
After building and sourcing your workspace, run the ROS2 TCP endpoint server. Running with the IP of 0.0.0.0 will listen for all IP addresses:
	ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address>
	ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=21150
This server will act as an intermediate between your ROS2 clients and Unity.
Next, open a second instance of the container with --net=host to access all outside network. Source and build again, then you can use this to echo your ROS2 topics with information sent out from Unity.
	docker run -it --net=host spot-ros-x64 /bin/bash
If running the pos_rot test (see XYZ), configure your Unity ROS environment to use the above port, run the scene, then in the 2nd container terminal, run:
	ros2 topic echo pos_rot

## TODO REVIEW THIS ROS Test

https://github.com/osrf/docker_images/blob/3f4fbca923d80f834f3a89b5960bad5582652519/ros/humble/ubuntu/jammy/desktop/Dockerfile

https://roboticseabass.com/2021/04/21/docker-and-ros/

TODO REVIEW VVVV
Each time ROS2 is launched, ros2 AND the workspace will need to be sourced.
1. For ROS2 itself: 'source /opt/ros/humble/setup.bash' in /bin/bash
2. For the workspace: 'source source install/local_setup.bash' in /bin/bash



## TODO REVIEW THIS Getting ROS on Spot

https://roboticseabass.com/2021/04/21/docker-and-ros/

ROS will run as a container that is part of your custom extension. To create this container, you must first create the image. 

1. Base this image off the nvidia/cudagl implementation of Ubuntu 20. The ARM64 variation is not necessary as the cudagl implementation allows for multi-architecture setups (the ARM64 version is deprecated).
	'docker pull nvidia/cudagl:11.4.2-base-ubuntu20.04'
	This command was sourced by searching 'nvidia/cuda' on Docker Desktop to view available images online
2.

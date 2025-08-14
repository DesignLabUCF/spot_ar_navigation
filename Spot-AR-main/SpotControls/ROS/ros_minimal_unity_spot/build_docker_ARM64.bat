docker buildx create --use --name multiarchbuilder
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker build -t ros-minimal-unity-spot-arm64 --platform linux/arm64 --build-arg BOSDYN_CLIENT_USERNAME=%BOSDYN_CLIENT_USERNAME% --build-arg BOSDYN_CLIENT_PASSWORD=%BOSDYN_CLIENT_PASSWORD% -f Dockerfile .
docker save ros-minimal-unity-spot-arm64 > ros-minimal-unity-spot-arm64.tgz
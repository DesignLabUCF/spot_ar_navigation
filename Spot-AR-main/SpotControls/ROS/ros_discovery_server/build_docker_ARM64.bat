docker build -t ros-discovery-arm64 -f Dockerfile.ARM64 --platform linux/arm64 .

:: Save to TGZ file
docker save ros-discovery-arm64 > ros-discovery-arm64.tgz
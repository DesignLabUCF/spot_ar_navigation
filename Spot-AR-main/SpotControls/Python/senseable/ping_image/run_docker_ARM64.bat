::docker run -p 21001:21001 ping-image-dockerization 

:: docker run -it -p 21001:21001 --platform linux/arm64 --network=host ping-image-dockerization-arm64 192.168.200.45

docker run -it -p 21001:21001 --platform linux/arm64 ping-image-dockerization-arm64
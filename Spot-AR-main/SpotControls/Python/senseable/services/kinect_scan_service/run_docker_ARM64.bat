docker run -it --platform linux/arm64 kinect-scan-service-arm64

:: Run on spot if container not launching properly in extension
:: sudo docker run -it --device /dev/bus:/dev/bus -v /data:/data kinect-scan-service-arm64 /bin/bash
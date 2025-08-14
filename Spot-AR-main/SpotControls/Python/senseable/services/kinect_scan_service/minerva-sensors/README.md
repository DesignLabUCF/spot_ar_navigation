# Minerva-sensors  <!-- omit from toc -->
Sensors and external libraries for the Minerva project

## Contents  <!-- omit from toc -->
- [Installation of IR Camera Code on Linux](#installation-of-ir-camera-code-on-linux)
- [How to Add New IR Camera Examples](#how-to-add-new-ir-camera-examples)
- [IR Camera Examples](#ir-camera-examples)
  - [Maintained](#maintained)
    - [basic\_opencv\_test](#basic_opencv_test)
    - [ici-test](#ici-test)
    - [multiple-cameras-v2](#multiple-cameras-v2)
    - [video-only](#video-only)
  - [Deprecated](#deprecated)
    - [multiple-cameras](#multiple-cameras)
    - [post-processing](#post-processing)
- [I2C-tb](#i2c-tb)
    - [Example config.h file](#example-configh-file)
- [BME680-tb](#bme680-tb)
    - [Example config.h file](#example-configh-file-1)



In the root directory, enable all **6** cameras with this command:

```
sudo bash openports.sh
```
***

# Installation of IR Camera Code on Linux

1. Clone the repo with the recursive flag in order to install the external libraries

``` 
git clone https://github.com/DesignLabUCF/minerva-sensors.git --recursive 
```

and install the depndencies
```
sudo apt install build-essential cmake git libgtk-3-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev openexr libatlas-base-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-dev gfortran -y
```

2. Make a build directory and move into that directory

``` 
mkdir build
cd build
```
3. Run CMake
``` 
cmake .. 
```

4. Run Make
``` 
make 
```

# How to Add New IR Camera Examples 

The CMake files are set up to help make this process very simple for a basic example. Depending on the complexity of the example, there may be other steps that will need to be taken such as packages that need to be added.

1. Make a new directory in the [examples](/examples/) folder to hold the project.

2. Create a ```CMakeLists.txt```. A basic example will look like this:
```
cmake_minimum_required(VERSION 2.8)
project( <PROJECT-NAME-HERE> )


find_package( OpenCV REQUIRED )
add_executable( <PROJECT-NAME-HERE> main.cpp )
link_directories(~/Documents/minerva-sensors/build)
target_link_libraries( <PROJECT-NAME-HERE>   FSLP FLIR_C_SDK UART_Connector ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so)
target_include_directories( <PROJECT-NAME-HERE> PRIVATE ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})
```
This includes the BosonSDK, OpenCV, and the ICI SDK. 

3. Create a ```main.cpp```. The file can be named whatever, but the name must be reflected in the argument passed to the ```add_executable``` line in the ```CMakeList.txt```.

4. Add the subdirectory of the example in [examples/CMakeLists.txt](examples/CMakeLists.txt).

5. Run CMake
``` 
cmake .. 
```

6. Run Make
``` 
make 
```


# IR Camera Examples 

[Full documentation](/examples/README.md)
## Maintained

### [basic_opencv_test](/examples/basic_opencv_test/)
### [ici-test](/examples/ici-test/)
### [multiple-cameras-v2](/examples/multiple-cameras-v2/)
### [video-only](/examples/video-only/)
## Deprecated

### [multiple-cameras](/examples/multiple-cameras/)
### [post-processing](/examples/postprocessing/)

# I2C-tb 
The I2C-tb folder contains code for the Arduino to send data from the [SCD30](https://www.seeedstudio.com/Grove-CO2-Temperature-Humidity-Sensor-SCD30-p-2911.html) and the [multichannel gas sensor](https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/) to a [Thingsboard](https://thingsboard.io/) instance. Information about the Thingsboard instance needs to be put into a congfig.h file before running.

This code should be opened, compiled, and uploaded to the device using the Arduino IDE.

### Example config.h file
```
#define WIFI_AP "<WIFI NAME>"
#define WIFI_PASSWORD "<PASSWORD>"

#define TOKEN "<YOUR THINGSBOARD TOKEN>"

#define HOSTIP "000.000.0.000"
```

# BME680-tb 
The BME680-tb folder contains code for the Arduino to send data from the [BME680](https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-for-Arduino-BME680.html) to a [Thingsboard](https://thingsboard.io/) instance. Information about the Thingsboard instance needs to be put into a congfig.h file before running.

This code should be opened, compiled, and uploaded to the device using the Arduino IDE.

### Example config.h file
```
#define WIFI_AP "<WIFI NAME>"
#define WIFI_PASSWORD "<PASSWORD>"

#define TOKEN "<YOUR THINGSBOARD TOKEN>"

#define HOSTIP "000.000.0.000"
```

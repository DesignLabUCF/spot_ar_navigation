# IR Camera Examples 

## basic_opencv_test

This example checks the frame rate of a camera using OpenCV.

## ici-test

This simple example will read from the first camera that is plugged into the device (/video0, /ttyACM0). By passing in the argument 'r', the files will be recorded to a folder called "basic-capture" in the build directory.

The files that will be saved are:
- a raw grey16 image
- a white-hot thermal image
- a jet thermal image
- an inferno thermal image
- an autumn thermal image
- a hot thermal image
- a text file containing the temperature values for each pixel and the FPA value for that capture

Two frames will be saved before the program closes. To manually change the number of frames or the name of the folder, ```video_frames``` and ```folder_name``` can be changed in the code respectively. Remember that in order for these changes to go into effect make has to be run again.

**_Important_**

```calibrationFileDirectoryPath``` throws bugs if an absolute path is not passed in. Before running the example, make sure that the path to the bin folder is correct for the current device.


## video-only

This example does not try to get radiometric data. It displays a 3 by 2 matrix of the camera streams. Passing in 'r' as an argument will save the video file. During runtime, 'h', 'j', and 'w' can be entered to change the colormapping of the streams. 'h' will change it to HOT, 'j' will change it to JET, and 'w' will change it to the default - white hot.

To exit the program, press ESC.

This program will only run if all 6 cameras are connected.

## multiple-cameras *Deprecated*

This example contains two classes: **UART_Connector_wrapper** and **Boson_Camera**.

The program recognizes 1 command line argument and that is 'r' for recording. The number of cameras is automatically detected. The example will run with either 1 or 6 cameras. It will exit with any other amount. If the recording flag is not passed in, only one camera will stream. All cases will stream indefinitely until 'q' is pressed.

Images and text files are saved to a folder labeled 'BosonImages'.


### UART_Connector_wrapper

- ```Initialize(int32_t port_num, int32_t baud_rate)```
  - Input: 
    - port number that corresponds to an index from the lists of ports created by the Boson SDK, starts at 16 on linux
    - baud rate for the camera, use 921600
  - Output:
    - FLR_RESULT
  - Description:
    - Initializes an instance of the camera in the Boson SDK
- ```bosonGetCameraSN(uint32_t *data)```
  - Input:
    - Takes a uint32_t variable which will have the value passed by reference
  - Output:
    - FLR_RESULT
  - Description:
    - Used to get the serial number of the camera
- ```bosonSetGainMode(FLR_BOSON_GAINMODE_E gainMode)```
  - Input:
    - Takes a FLR_BOSON_GAINMODE_E enum value. It has only been tested with FLR_BOSON_HIGH_GAIN, but the possible values are:
      - FLR_BOSON_HIGH_GAIN = 0
      - FLR_BOSON_LOW_GAIN = 1
      - FLR_BOSON_AUTO_GAIN = 2
      - FLR_BOSON_DUAL_GAIN = 3
      - FLR_BOSON_MANUAL_GAIN = 4
      - FLR_BOSON_GAINMODE_END = 5 
  - Output:
    - FLR_RESULT
  - Description:
    - Sets the mode of the camera's temperature compensation operation
- ```bosonRunFFC()```
  - Input:
    - none
  - Output:
    - FLR_RESULT
  - Description:
    - performs flat field correction
- ```roicGetFPATemp(uint16_t *data)```
  - Input:
    - takes a uint32_t variable which will have the fpa value passed by reference
  - Output:
    - FLR_RESULT
  - Description:
    - Used to get the fpa value for the frame
- ```FLR_RESULT bosonlookupFPATempDegCx10(int16_t *data)```
  - Input:
    - Takes a reference to a int16_t variable
  - Output:
    - FLIR_RESULT
    - The temperature value is passed by reference
  - Description:
    - Returns the temperature value of the sensor which is the calibrated FPA value.
- ``` FLR_RESULT sysctrlGetCameraFrameRate(uint32_t *frameRate)```
  - Input:
    - Takes a reference to a uint32_t variable that will have the framerate value passed in
  - Output:
    - FLIR_RESULT
    - The framerate is passed by reference
  - Description:
    - Returns the framerate of the camera
- ```Close()```
  - Input:
    - none
  - Output:
    - none
  - Description:
    - Closes the connection to the Flir Boson backend and destroys the object

### Boson_Camera

- Private variables
  - UART_Connector_wrapper cam; 
    - The Flir camera component
  - cv::VideoCapture cap; 
    - OpenCV VideoCapture component
  - uint32_t camera_sn;
    - Camera serial number
  - int width = 640;
  - int height = 512;
  - int totalSize = width * height;
- Public Members
  - ```Boson_Camera(int32_t camPort, int capPort)```
    - Input:
      - cam port is the port number for initializing the UART_Connector_wrapper object
      - cap port is the port number for initializing the OpenCV VideoCapture object
    - Output:
      - none
    - Description:
      - Constructor that initializes the private variables, sets the gain mode to high, and runs a flat field correction
  - ```~Boson_Camera()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Deconstructor that closes the UART_Connector_wrapper and the VideoCapture objects
  - ```PrintToFile(long frame, float *temperatureInCelsius)``` *Deprecated*
    - Input:
      - The frame variable which will be used as an index for naming the file
      - temperatureInCelsius is the value returned from the ICI SDK temperature function
    - Output:
      - none
    - Description:
      - Prints the temperature values to a file with each temperature value for each pixel on a new line
  - ```ReadWriteFrame(long frameCount)```
    - Input:
      - frameCount is used for naming the image and the text file 
    - Output:
      - none
    - Description:
      - Reads and saves a 16-bit gray image from the Boson camera. It also writes the fpa value for the frame to a text file.
  -  ```void Boson_Camera::CaptureFrame()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Captures a single frame and the corresponding FPA value and pushes it onto the class queue
    - REQUIRED - WriteFrame needs to be called in order to pop frames off the queue and prevent the program from crashing
  - ```void Boson_Camera::WriteFrame()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Intended to be run in a separate thread to allow for images to be saved while the camera(s) is capturing images. There are two main sections. The first runs while the isStreaming flag is lifted. Inside the loop is a check for if the queue has any frames that need saving. The second section finishes saving the rest of the frames in the queue once the isStreaming flag has been removed. The frame count is handled internally.
    - REQUIRED - CaptureFrame needs to run at some point to put frames in the queue
    - REQUIRED - isStreaming needs to be set to false in order for the method to end
  - ```AGC_Basic_Linear(cv::Mat input_16, int height, int width)```
    - Input:
      - Mat containing the 16-bit image from the camera
      - height of the image
      - width of the image
    - Output:
      - OpenCV Mat containing the post AGC image
    - Description:
      - Performs a basic AGC function that results in a gray 8 image
  - ```Stream()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Streams one camera without saving anything. While the stream is running, pressing 'w' will save the current frame in a folder labeled "StreamImages".
  - ```float getSensorTemp()```
    - Input:
      - none
    - Output:
      - the sensor temperature in Celsius
    - Description:
      - Outputs the current temperature of the camera
  - ```uint32_t getFrameRate()```
    - Input:
      - none
    - Output:
      - the sensor temp in Celsius 
    - Description:
      - Returns the calibrated FPA value which is the sensor temperature in Celsius.
  - ```bool isStreaming = true```
    - not a function. IsStreaming is a public variable that can be accessed and modified without a function.

## multiple-cameras-v2

This is the developed version for recording from multiple IR cameras.  

This example contains 5 classes: **UART_Connector_wrapper**, **Boson_Camera**, **Timer**, **Keyboard**, and **Frame**. 

**UART_Connector_wrapper**, **Boson_Camera**, and **Frame** are the only ones developed specifically for this project. Therefore they are the only ones that will be described here

The program recognizes 1 command line argument and that is 'r' for recording. The number of cameras is automatically detected. The example will run with either 1 or 6 cameras. It will exit with any other amount. If the recording flag is not passed in, only one camera will stream. If in streaming mode, pressing 'w' will save the current frame in a folder labeled 'StreamImages'. Pressing 'q' will end the stream.

If in recording mode, camera objects and their corresponding image queues will be set up. The writer threads will be set up to run in the background while a capture thread for each camera will operate in parallel. Images are captured and stored in the camera's image queue at a constant rate. Meanwhile, the writer thread is popping off the images and saving them. The Keyboard class is supposed to stop the capture process when 'ESC' is pressed. However, this quite rarely (almost never) happens. 

Images and text files are saved to a folder labeled 'BosonImages' and captured at a rate of 30 FPS.

### UART_Connector_wrapper

- ```Initialize(int32_t port_num, int32_t baud_rate)```
  - Input: 
    - port number that corresponds to an index from the lists of ports created by the Boson SDK, starts at 16 on linux
    - baud rate for the camera, use 921600
  - Output:
    - FLR_RESULT
  - Description:
    - Initializes an instance of the camera in the Boson SDK
- ```bosonGetCameraSN(uint32_t *data)```
  - Input:
    - Takes a uint32_t variable which will have the value passed by reference
  - Output:
    - FLR_RESULT
  - Description:
    - Used to get the serial number of the camera
- ```bosonSetGainMode(FLR_BOSON_GAINMODE_E gainMode)```
  - Input:
    - Takes a FLR_BOSON_GAINMODE_E enum value. It has only been tested with FLR_BOSON_HIGH_GAIN, but the possible values are:
      - FLR_BOSON_HIGH_GAIN = 0
      - FLR_BOSON_LOW_GAIN = 1
      - FLR_BOSON_AUTO_GAIN = 2
      - FLR_BOSON_DUAL_GAIN = 3
      - FLR_BOSON_MANUAL_GAIN = 4
      - FLR_BOSON_GAINMODE_END = 5 
  - Output:
    - FLR_RESULT
  - Description:
    - Sets the mode of the camera's temperature compensation operation
- ```bosonRunFFC()```
  - Input:
    - none
  - Output:
    - FLR_RESULT
  - Description:
    - performs flat field correction
- ```roicGetFPATemp(uint16_t *data)```
  - Input:
    - takes a uint32_t variable which will have the fpa value passed by reference
  - Output:
    - FLR_RESULT
  - Description:
    - Used to get the fpa value for the frame
- ```FLR_RESULT bosonlookupFPATempDegCx10(int16_t *data)```
  - Input:
    - Takes a reference to a int16_t variable
  - Output:
    - FLIR_RESULT
    - The temperature value is passed by reference
  - Description:
    - Returns the temperature value of the sensor which is the calibrated FPA value.
- ``` FLR_RESULT sysctrlGetCameraFrameRate(uint32_t *frameRate)```
  - Input:
    - Takes a reference to a uint32_t variable that will have the framerate value passed in
  - Output:
    - FLIR_RESULT
    - The framerate is passed by reference
  - Description:
    - Returns the framerate of the camera
- ```Close()```
  - Input:
    - none
  - Output:
    - none
  - Description:
    - Closes the connection to the Flir Boson backend and destroys the object

### Boson_Camera

- Private variables
  - UART_Connector_wrapper cam; 
    - The Flir camera component
  - cv::VideoCapture cap; 
    - OpenCV VideoCapture component
  - uint32_t camera_sn;
    - Camera serial number
  - int width = 640;
  - int height = 512;
  - int totalSize = width * height;
- Public Members
  - ```Boson_Camera(int32_t camPort, int capPort)```
    - Input:
      - cam port is the port number for initializing the UART_Connector_wrapper object
      - cap port is the port number for initializing the OpenCV VideoCapture object
    - Output:
      - none
    - Description:
      - Constructor that initializes the private variables, sets the gain mode to high, and runs a flat field correction
  - ```~Boson_Camera()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Deconstructor that closes the UART_Connector_wrapper and the VideoCapture objects
  - ```PrintToFile(long frame, float *temperatureInCelsius)``` *Deprecated*
    - Input:
      - The frame variable which will be used as an index for naming the file
      - temperatureInCelsius is the value returned from the ICI SDK temperature function
    - Output:
      - none
    - Description:
      - Prints the temperature values to a file with each temperature value for each pixel on a new line
  - ```ReadWriteFrame(long frameCount)```
    - Input:
      - frameCount is used for naming the image and the text file 
    - Output:
      - none
    - Description:
      - Reads and saves a 16-bit gray image from the Boson camera. It also writes the fpa value for the frame to a text file.
  -  ```void Boson_Camera::CaptureFrame()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Captures a single frame and the corresponding FPA value and returns a frame object
    - REQUIRED - WriteFrame needs to be called in order to pop frames off the queue and prevent the program from crashing
  - ```void Boson_Camera::WriteFrame()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Intended to be run in a separate thread to allow for images to be saved while the camera(s) is capturing images. There are two main sections. The first runs while the isStreaming flag is lifted. Inside the loop is a check for if the queue has any frames that need saving. The second section finishes saving the rest of the frames in the queue once the isStreaming flag has been removed. The frame count is handled internally.
    - REQUIRED - CaptureFrame needs to run at some point to put frames in the queue
    - REQUIRED - isStreaming needs to be set to false in order for the method to end
  - ```AGC_Basic_Linear(cv::Mat input_16, int height, int width)```
    - Input:
      - Mat containing the 16-bit image from the camera
      - height of the image
      - width of the image
    - Output:
      - OpenCV Mat containing the post AGC image
    - Description:
      - Performs a basic AGC function that results in a gray 8 image
  - ```Stream()```
    - Input:
      - none
    - Output:
      - none
    - Description:
      - Streams one camera without saving anything. While the stream is running, pressing 'w' will save the current frame in a folder labeled "StreamImages".
  - ```float getSensorTemp()```
    - Input:
      - none
    - Output:
      - the sensor temperature in Celsius
    - Description:
      - Outputs the current temperature of the camera
  - ```uint32_t getFrameRate()```
    - Input:
      - none
    - Output:
      - the sensor temp in Celsius 
    - Description:
      - Returns the calibrated FPA value which is the sensor temperature in Celsius.
  - ```bool isStreaming = true```
    - not a function. IsStreaming is a public variable that can be accessed and modified without a function. This is a flag to let the writer thread know when streaming has ended.

### Frame

- Private variable
  - cv::Mat image 
    - Holds the OpenCV Mat for the frame
  - uint16_t fpa
    - The FPA value for the frame
  - uint32_t serialnumber
    - The serial number of the camera that captured the frame
- Public Members
  - ```Frame()```
    - Description:
      - constructor that takes no inputs and assigns image, fpa, serialnumber, and ms to NULL
  - ```Frame(cv::Mat Newimage, uint16_t Newfpa, uint32_t NewSN, uint64_t NewMS)```
    - Input:
      - OpenCV Mat containing the frame image
      - a uint32_t variable that contains the FPA value
      - the serial number of the camera as a uint32_t variable
      - the unix timestamp of the frame in milliseconds
    - Output:
      - none
    - Description:
      - constructor that instantiates values for the image, fpa, serialnumber, and ms variables
  - ```~Frame()``` 
    - Description:
      - deconstructor that releases the OpenCV Mat
  - ```cv::Mat getImage()```
    - Output:
      - the OpenCV Mat containing the image
    - Description:
      - returns the image from the frame object
  - ```uint16_t getFPA()``` 
    - Output:
      - the FPA value as a uint16_t variable
    - Description:
      - returns the FPA value from the frame object
  - ```uint32_t getSerialNum()```
    - Output:
      - the serial number of the camera that captured the frame as a uint32_t variable
    - Description:
      - returns the serial number of the camera that captured the frame
  - ```uint64_t ms```
    - not a function. ms is a public variable that can be accessed and modified without a function. It is intended to store the unix timestamp of the camera capture in milliseconds.
## postprocessing *Deprecated*

This example is not compatible with the current standards, but it remains here as a reference of past work.

This example does not use the Boson camera but instead runs postprocessing on the images produced by the multiple-camera example.

There is the option to pass an absolute path to the folder containing the images as a command line argument. If not, the program will prompt for this information.

The images in the folder are then read in and a linear AGC is run. The images are saved in subfolders based on the serial number of the camera. Additionally, a video is created for each camera and saved in that camera's folder. The temperature values are stored in npy files to allow for further analyses.


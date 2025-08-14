# Mulitple Cameras
## *Work in Progress Notes*

Currently, we have implemented in the C version of the Boson SDK and ICI SDK with C++ for the main code. The problem is C is not an OOP language, so there is no encapsulation, and it is causing issues with initializing more than one camera at once.

This example is set up to replicate the errors we are experienceing, so that we can find a way to solve them. AS I work through the process, I will keep notes in this readme to have a record of what has been tried and the results.

Possible solutions:
- rewrite everything using the Python version of the Boson SDK and have the main code in Python and have the C ICI code interface with the Python.
- Implement OOP workarounds in the BosonSDK to allow for multiple camera "classes"


Replicate the UART_connector in to a C++ class that wrap in the folder with the example
- class varibles not static!!!!

- need class and header file in this directory

### 4/2
- 1st port: 16 - 0
- 2nd port: 17 - 2
- 3rd port: 18 - 4
- 4th port: 19 - 6
- 5th port: 20 - 8
- 6th port: 21 - 10

All of the camera functions are available in the Boson_Camera class. Main will focus on the multiprocessing of the different cameras with the information encapsulated in each camera object.

The main will need to parse the arguments, identify how many cameras are plugged in, start the selected process for each camera.

### 4/3
- saveFrames needs to be deprcated and replaced with ReadWriteFrame (singluar!!!). The loop should not be in the class and instead should be implemented by the main function.
- more metadata needs to be collected i.e. sensor temp, etc
    - it would be great if this could be stored with the image but I am not seeing a way to do that with the current set up so this might be a goal for later and fonow it ill just be saved to file
- the read write function just needs to write the most basic informations and other stuff can be done in post to help alleviate the processing power
    - 16 bit image and fpa value


### 4/5
There is a segmentation fault in the postprocessing example that occurs at image 65. I don't know if this is a memory allocation error or if there is something wrong with the image that is causing this to occur. The vector does not seems to be the problem because the file names all have to be added to the vector before it starts saving images. Since 65 images are saved, it seems that maybe there is an issue with the image that might be causing issues. 

If that is the case, the issue needs to be identified and then checkes need to be put in place.

### 4/20
Post Processing is not working for massive amounts of images. 18 minutes of images has caused the program to be killed in reading in all of the files. I will try to reduce the  memory that is being used.

### 5/29
https://www.bogotobogo.com/cplusplus/multithreading_pthread.php - link that talks about how c++ threads work.

I'm thinking that there will be a global queue where the capture will add images to the queue and the write thread will pop them off the back and write them. The "global" queue will be less global but more individual to each camera so that way they can keep up with what camera it comes from. It is only global in the sense that the threads will have access to it so I guess the threads will have to be implemented in the class level. I am unsure if that is thread safe though. 

### 5/30
8 cores, no more than 16 threads so that way the cores don't cross. Right now just check 180 frames without writing to see what the fps is. Start timer before loop, go through 180 times then check start time against current time. Then 180 / timeTaken = fps.

Each camera a thread or potentially a core if we need to get it even faster or it just doesn't work.

### 6/2 
I'm a little overwhelmed with where to start beause there are so many possibilities and I'm unsure where I want to start. I'm thinking of making a function for capturing images that passes the captured images to a class vector. Then making another function that will save the files. I'm not sure about whether it will be thread safe though. 

https://github.com/AriNguyen/Multithreading-OpenCV-CPP/blob/master/src/WebcamStream.cpp
https://github.com/Qengineering/Multithread-Camera-OpenCV/blob/main/include/ThreadCam.h 


 ### 6/6 
 get an atomic with a try catch variable to get all the threads to stop at once
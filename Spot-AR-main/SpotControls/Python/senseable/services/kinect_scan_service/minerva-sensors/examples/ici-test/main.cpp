#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <thread>
#include <fstream>
#include <string>

#include <stdbool.h>    ///********************Fixes   _Bool????????????????????????????????? #JTK

// random number
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */




extern "C" {
    #include "Client_API.h"
    #include "EnumTypes.h"
    #include "UART_Connector.h"



    // #include "UART_HalfDuplex.h"
    // #include "flirChannels.h"
    // #include "flirCRC.h"
    // #include "rs232.h"

}


#define YUV   0
#define RAW16 1

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

// Global variables to keep this simple
int width;
int height;

// Types of sensors supported
enum sensor_types {
  Boson320, Boson640
};

/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width) {
    int i, j;  // aux variables

    // auxiliary variables for AGC calcultion
    unsigned int max1=0;         // 16 bits
    unsigned int min1=0xFFFF;    // 16 bits
    unsigned int value1, value2, value3, value4;

    // RUN a super basic AGC
    for (i=0; i<height; i++) {
        for (j=0; j<width; j++) {
            value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
            value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
            value3 = ( value1 << 8) + value2;
            if ( value3 <= min1 ) {
                min1 = value3;
            }
            if ( value3 >= max1 ) {
                max1 = value3;
            }
            //printf("%X.%X.%X  ", value1, value2, value3);
        }
    }
    //printf("max1=%04X, min1=%04X\n", max1, min1);

    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
            value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
            value3 = ( value1 << 8) + value2;
            value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
            // printf("%04X \n", value4);

            output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
        }
    }

}

/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT

int main(int argc, char** argv )
{
    // variables defined in FLIR/BosonUSB
    int ret;
    int fd;
    int i;
    struct v4l2_capability cap;
    long frame=0;     // First frame number enumeration
    char video[20];   // To store Video Port Device
    char label[50];   // To display the information
    char thermal_sensor_name[20];  // To store the sensor name
    char filename[60];  // PATH/File_count
    char folder_name[30];  // To store the folder name
    char video_frames_str[30];
    // Default Program options
    int  video_mode=RAW16;
    int  video_frames=5; // number of frames captured
    int  zoom_enable=0;
    int  record_enable=0;
    sensor_types my_thermal=Boson640;
    // max cameras 6
    int devicesLength = 7; // in the case of web cam
    int devices[devicesLength];
    bool device_viable[devicesLength];


    /* initialize random seed: */
    srand (time(NULL));

    // To record images
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PXM_BINARY);
    compression_params.push_back(100);

    // file stream to hold the temperature pixel values // later it might  make more sense to just store the fpa values for each frame at and collect some of the metadata
    std::ofstream TemperatureValues; 
    char textFileName[30];

    // Read command line arguments
    for (i=0; i<argc; i++) {
    
        // Check if recording is enabled
        if ( argv[i][0]=='r') {  // File name has to be more than two chars
                    record_enable=1;
                    strcpy(folder_name, "basic-capture");
            }
        
    }

    // brute force opening multiple cameras
    int j = 0;
    int k = 0;
    while ( j < devicesLength && k <= 11) {
        for (k = 0; k <= 11; k+=2 ){ // not sure how to filter out the metadata files because they are being recognized as video sources, so this is just skipping the metadata under the assumption that the second file is the
            // We open the Video Device
            sprintf(video, "/dev/video%i", k);
            printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
            
            if((fd = open(video, O_RDWR)) >= 0){
                // Check VideoCapture mode is available
                if(ioctl(fd, VIDIOC_QUERYCAP, &cap) >= 0){
                    if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
                        if (ioctl(fd, VIDIOC_G_INPUT) == 0) {
                            devices[j] = k;
                            std::string card;
                            card.append(reinterpret_cast<char*>(cap.card));
                            if (card.substr(0, 5) == "Boson") {
                                device_viable[j] = true;
                            } else {
                                device_viable[j] = false;
                            }
                            j++;
                            std::cout << j << std::endl;
                        }
                    }
                }

            }
            close(fd);
        }
    } 

    std::cout << "there were " << j << " number of devices" << std::endl;
    if (j == 0) {
        exit(1);
    }
    devicesLength = j;

    if (device_viable[0]) {
        sprintf(video, "/dev/video%i", devices[0]);
    } else if (devicesLength > 1) {
        sprintf(video, "/dev/video%i", devices[1]);
    }


    sprintf(thermal_sensor_name, "Boson_640");

    // Folder name
    if (record_enable==1) {
        if ( strlen(folder_name)<=1 ) {  // File name has to be more than two chars
                strcpy(folder_name, thermal_sensor_name);
            }
            mkdir(folder_name, 0700);
            chdir(folder_name);
                printf(WHT ">>> Folder " YEL "%s" WHT " selected to record files\n", folder_name);
    }


    // We open the Video Device
    printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
    if((fd = open(video, O_RDWR)) < 0){
        perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
        exit(1);
    }

    // Check VideoCapture mode is available
    if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
        perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
        exit(1);
    }

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
        fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
        exit(1);
    }

    struct v4l2_format format;
    
    CLEAR(format);

    printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");

    // I am requiring thermal 16 bits mode
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

    // Select the frame SIZE (will depend on the type of sensor)
    switch (my_thermal) {
        case Boson320:  // Boson320
                    width=320;
                    height=256;
                    break;
        case Boson640:  // Boson640
                    width=640;
                    height=512;
                    break;
        default:  // Boson320
                    width=320;
                    height=256;
                    break;
    }

    // Common varibles
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;

    // request desired FORMAT
    if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
        perror(RED "VIDIOC_S_FMT" WHT);
        exit(1);
    }

    // we need to inform the device about buffers to use.
    // and we need to allocate them.
    // we’ll use a single buffer, and map our memory using mmap.
    // All this information is sent using the VIDIOC_REQBUFS call and a
    // v4l2_requestbuffers structure:
    struct v4l2_requestbuffers bufrequest;
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = 1;   // we are asking for one buffer

    if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
        perror(RED "VIDIOC_REQBUFS" WHT);
        exit(1);
    }

    // Now that the device knows how to provide its data,
    // we need to ask it about the amount of memory it needs,
    // and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
    // and its v4l2_buffer structure.

    struct v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));

    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
        perror(RED "VIDIOC_QUERYBUF" WHT);
        exit(1);
    }

    void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

    if(buffer_start == MAP_FAILED){
        perror(RED "mmap" WHT);
        exit(1);
    }

    // Fill this buffer with ceros. Initialization. Optional but nice to do
    memset(buffer_start, 0, bufferinfo.length);

    // Activate streaming
    int type = bufferinfo.type;
    if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
        perror(RED "VIDIOC_STREAMON" WHT);
        exit(1);
    }

    // Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
    // Boson320 , Boson 640
    cv::Mat thermal16(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
    cv::Mat thermal16_linear(height,width, CV_8U, 1);         // OpenCV output buffer : Data used to display the video


    // // Reaad frame, do AGC, paint frame
    // for (;;) {

    //  // Put the buffer in the incoming queue.
    //  if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
    //      perror(RED "VIDIOC_QBUF" WHT);
    //      exit(1);
    //  }

    //  // The buffer's waiting in the outgoing queue.
    //  if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
    //      perror(RED "VIDIOC_QBUF" WHT);
    //      exit(1);
    //  }


    //  // -----------------------------
    //  // RAW16 DATA
    //  if ( video_mode==RAW16 ) {
    //      AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

    //      // Display thermal after 16-bits AGC... will display an image
    //      if (zoom_enable==0) {
    //                  sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
    //                      imshow(label, thermal16_linear);
    //              } else {
    //              resize(thermal16_linear, thermal16_linear_zoom, size);
    //                          sprintf(label, "%s : RAW16  Linear Zoom", thermal_sensor_name);
    //                          imshow(label, thermal16_linear_zoom);
    //              }


    //          if (record_enable==1) {
    //                  sprintf(filename, "%s_raw16_%lu.tiff", thermal_sensor_name, frame);
    //                  imwrite(filename, thermal16 , compression_params );

    //                     sprintf(filename, "%s_agc_%lu.tiff", thermal_sensor_name, frame);
    //                     imwrite(filename, thermal16_linear , compression_params );
    //          frame++;
    //              }
    //          }
    //     // Press 'q' to exit
    //  if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
    //      printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
    //      break;
    //  }
    //  // Stop if frame limit reached.
    //  if (video_frames>0 && frame+1 > video_frames) {
    //      printf(WHT ">>>" GRN "'Done'" WHT " Frame limit reached, Quitting !\n");
    //      break;
    //  }
    // }

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Intitialize the Boson Camera
    FLR_RESULT result;
    result = Initialize(16, 921600); // /dev/ttyACM0, 921600 baud
    printf("Initialize: ");
    if (result)
    {
        printf("Failed to initialize, exiting.\n");
        Close();
        return 1;
    }
    printf("0x%08X\n", result);
    printf("SUCCESS\n\n");


    // set high gain mode
    bosonSetGainMode(FLR_BOSON_HIGH_GAIN);

    // Step 3: Set Camera Output to 8-bit or 16-bit
    // not completely sure of the purpose but it seemed like it might be important
    FLR_DVO_TYPE_E currDvoSource;
    FLR_DVO_TYPE_E dvo8Bit = FLR_DVO_TYPE_MONO8;
    FLR_DVO_TYPE_E dvo16Bit = FLR_DVO_TYPE_MONO16;

    result = dvoGetType(&currDvoSource);
    printf("Initial DVO Source: ");
    if (result)
    {
        printf("Failed DVO with status 0x%08X, exiting.\n",result);
        Close();
        return 1;
    }
    printf("Result: 0x%08X -- Value: 0x%08X \n", result, currDvoSource);
    printf("Change Source: ");
    result = dvoSetType(dvo16Bit);
    // result = dvoSetType(dvo8Bit);
    if (result)
    {
        printf("Failed DVO with status 0x%08X, exiting.\n",result);
        Close();
        return 1;
    }
    printf("0x%08X\n", result);
    printf("SUCCESS\n\n");



    // start of ICI code
    // get the handle to enter into the loop
    auto const handle = ICICreateFlirBosonRawCountsToTemperatureInCelsiusHandle();
    if (handle)
    {
        // retrieve the serial number form camera in order to look up respective bin file
        printf("CameraSN: ");
        uint32_t camera_sn;
        result = bosonGetCameraSN(&camera_sn);
        if (result)
        {
            printf("Failed CameraSN with status 0x%08X, exiting.\n",result);
            Close();
            return 1;
        }
        printf("%d \n", camera_sn);
        printf("SUCCESS\n\n");
        auto const calibrationFileDirectoryPath = "/home/sensable/Documents/minerva-sensors/bin";
        if (ICILoadCalibrationFileData(handle, calibrationFileDirectoryPath, camera_sn))
        {
            // capture frame, get raw counts 
            for(;;) {
                        // Put the buffer in the incoming queue.
                if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
                    perror(RED "VIDIOC_QBUF" WHT);
                    exit(1);
                }

                // The buffer's waiting in the outgoing queue.
                if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
                    perror(RED "VIDIOC_QBUF" WHT);
                    exit(1);
                }


                // -----------------------------
                // RAW16 DATA
                if ( video_mode==RAW16 ) {
                    AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

                    // Display thermal after 16-bits AGC... will display an image
                    
                    sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
                    cv::imshow(label, thermal16_linear);
                        


                    if (record_enable==1) {
                        cv::Mat coloredFrame;
                        cv::applyColorMap(thermal16_linear, coloredFrame, cv::COLORMAP_JET);
                        

                        sprintf(textFileName, "%d_raw16_%lu.txt", camera_sn, frame);
                        TemperatureValues.open(textFileName);


                        sprintf(filename, "%d_raw16_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, thermal16 , compression_params );

                        sprintf(filename, "%d_acg_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, thermal16_linear , compression_params );
                        
                        //white hot, infrno, jet, autumns, hot
                        sprintf(filename, "%d_jet_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, coloredFrame , compression_params );

                        cv::applyColorMap(thermal16_linear, coloredFrame, cv::COLORMAP_INFERNO);

                        sprintf(filename, "%d_inferno_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, coloredFrame , compression_params );

                        cv::applyColorMap(thermal16_linear, coloredFrame, cv::COLORMAP_AUTUMN);

                        sprintf(filename, "%d_autumn_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, coloredFrame , compression_params );

                        cv::applyColorMap(thermal16_linear, coloredFrame, cv::COLORMAP_HOT);

                        sprintf(filename, "%d_hot_%lu.tiff", camera_sn, frame);
                        cv::imwrite(filename, coloredFrame , compression_params );
                        
                        frame++;
                    }
                }
                int rows = thermal16.rows;
                int cols = thermal16.cols;

                int totalSize = rows * cols;
                // auto rawCounts = std::vector<uint16_t>{32637, 32643, 32751, 32635};
                uint16_t fpa;
                result = roicGetFPATemp(&fpa);

                printf("FPA: %d \n", fpa);

                auto temperatureInCeslius = ICIConvertFlirBosonCameraRawCountsToTemperatureInCelsius(
                    handle, (uint16_t *)thermal16.data, totalSize, fpa);
                if (temperatureInCeslius){
                    std::cout<<"this: " << temperatureInCeslius[163519] << "\n";
                     std::cout<<"this: " << temperatureInCeslius[23] << "\n";
                      std::cout<<"this: " << temperatureInCeslius[400] << "\n";

                    for (int i = 0; i < totalSize; i++) {
                        TemperatureValues << temperatureInCeslius[i] << "\n";
                    }
                    TemperatureValues.close();
                }
                // Stop if frame limit reached.
                if (video_frames>0 && frame+1 > video_frames) {
                    printf(WHT ">>>" GRN "'Done'" WHT " Frame limit reached, Quitting !\n");
                    break;
                }
            }
            ICIDestroyFlirBosonRawCountsToTemperatureInCelsiusHandle(handle);
        }
    }

    // Deactivate streaming
    if( ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 ){
        perror(RED "VIDIOC_STREAMOFF" WHT);
        exit(1);
    };

    close(fd);
    Close();
    return 1;
}
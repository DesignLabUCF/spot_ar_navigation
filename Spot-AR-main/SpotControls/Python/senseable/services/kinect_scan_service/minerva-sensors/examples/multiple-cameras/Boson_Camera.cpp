#include "Boson_Camera.h"
#include "UART_Connector_wrapper.h"
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <fstream>
#include <sys/stat.h>
#include "Frame.h"


// initalizes the boson camera object
// input is the cam port number which corresponds to the Boson SDK port list index and the cap port which corresponds to the ACM usb port on the linux os
Boson_Camera::Boson_Camera(int32_t camPort, int capPort) {
    FLR_RESULT result;

    result = cam.Initialize(camPort, 921600); // initalize the Boson SDK
    printf("Initialize: ");
    if (result)
    {
        printf("Failed to initialize, exiting.\n");
        //Close();
        exit(1);
    }
    printf("0x%08X\n", result);
    printf("SUCCESS\n\n");

    cap.open(capPort, cv::CAP_V4L); // initalize the opencv connection

    // set the opencv capture to Y16 to get the grey16 images that have the temperature values in it
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

    // get the camera serial number
    printf("CameraSN: ");
    result = cam.bosonGetCameraSN(&camera_sn);
    if (result)
    {
        printf("Failed CameraSN with status 0x%08X, exiting.\n",result);
        exit(1);
    }
    printf("%d \n", camera_sn);
    printf("SUCCESS\n\n");

    // set gain mode for the camera
    result = cam.bosonSetGainMode(FLR_BOSON_HIGH_GAIN);
    if (result)
    {
        printf("Failed GAIN with status 0x%08X, exiting.\n",result);
        exit(1);
    }
    printf("SUCCESS\n\n");

    // run flat field correction
    result = cam.bosonRunFFC();
    if (result)
    {
        printf("Failed FFC with status 0x%08X, exiting.\n",result);
        //Close();
        exit(1);
    }
    printf("SUCCESS\n\n");
}

// saves the temperature values for each pixel to a text file
// input is the frame count and the temperature values that are returned by the ICI sdk
void Boson_Camera::PrintToFile(long frame, float *temperatureInCelsius) {
    std::ofstream TemperatureValues;
    char textFileName[30];
    sprintf(textFileName, "%d_raw16_%lu.txt", camera_sn, frame);
    TemperatureValues.open(textFileName);
    for (int i = 0; i < totalSize; i++) {
        TemperatureValues << temperatureInCelsius[i] << "\n";
    }
    TemperatureValues.close();
}

// Reads a singluar frame from the camera and saves the gray16 image along with a txt file containing the corresponding fpa value
// takes in a frame count as an input that will act as the frame index 
// TODO: figure out a way to get metadata in the TIFF image rather than in another text file
void Boson_Camera::ReadWriteFrame(long frameCount) {
    cv::Mat thermalFrame; // gray16 image
    uint16_t fpa; // focal plane area
    std::ofstream FPAfile; // txt file to hold the fpa value
    char textFileName[30]; // file name for the fpa value for the image
    char filename[60];  // file name for the image
    
    sprintf(textFileName, "%lu-raw16_%lu.txt", frameCount, camera_sn);
    FPAfile.open(textFileName); // open text file

    cap >> thermalFrame; // read in frame

    // get the fpa value
    cam.roicGetFPATemp(&fpa);
    FPAfile << fpa; // read in fpa value
    sprintf(filename, "%lu-raw16_%d.tiff", frameCount, camera_sn); // create file name for the picture
    cv::imwrite(filename, thermalFrame); // save the grey16 image
    FPAfile.close();
}


// alternate option for the CaptureFrame method that has the looping logic inside of the function allowing for the method to only be called once
// -------------------------------------------------
// void Boson_Camera::CaptureFrame() {
//     cv::Mat thermalFrame; // gray16 image
//     uint16_t fpa; // focal plane area
//     // for(;;) { 
//     for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds{1}; now = std::chrono::steady_clock::now()) {
//         cap >> thermalFrame; // read in frame

//         // get the fpa value
//         cam.roicGetFPATemp(&fpa);

//         //save to the object
//         Frame frame(thermalFrame, fpa);
//         // add to queue
//         frameQueue.push(frame);
//         cv::imshow("Recording", thermalFrame); // need to open an opencv window in order to use waitkeys. If there is a better solution I'm all ears
//         if( cv::waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
//             std::cout << "ending stream" << std::endl;
//             isStreaming = false;
//             break;
//         }
//     }
//     isStreaming = false;
// }
// --------------------------------------------------


// Captures a single frame and the corresponding FPA value and pushes it onto the class queue
// REQUIRED - WriteFrame needs to be called in order to pop frames off the queue and prevent the program from crashing
void Boson_Camera::CaptureFrame() {
    cv::Mat thermalFrame; // gray16 image
    uint16_t fpa; // focal plane area

    cap >> thermalFrame; // read in frame

    // get the fpa value
    cam.roicGetFPATemp(&fpa);

    //save to the object
    Frame frame(thermalFrame, fpa);
    // add to queue
    frameQueue.push(frame);
    cv::imshow("Recording", thermalFrame); // need to open an opencv window in order to use waitkeys. If there is a better solution I'm all ears
}

// Intended to be run in a separate thread to allow for images to be saved while the camera(s) is capturing images.
// There are two main sections. The first runs while the isStreaming flag is lifted. Inside the loop is a check for if the queue has any frames that need saving.
// The second section finishes up saving the rest of the frames in the queue once the isStreaming flag has been removed. The frame count is 
// handled internally.
// REQUIRED - CaptureFrame needs to run at some point to put frames in the queue
// REQUIRED - isStreaming needs to be set to false in order for the method to end
// TODO: figure out a way to get metadata in the TIFF image rather than in another text file
void Boson_Camera::WriteFrame() {
    while (isStreaming) // isStreaming needs to be set to false in order to stop the loop and end the thread
    {
        if (!frameQueue.empty()) { // this will prevent the program from trying to save anything is there are no frames ready
            Frame currentFrame = frameQueue.front(); // store current frame
            frameQueue.pop(); // remove frame
            std::ofstream FPAfile; // txt file to hold the fpa value
            char textFileName[30]; // file name for the fpa value for the image
            char filename[60];  // file name for the image
            
            sprintf(textFileName, "%lu-raw16_%lu.txt", classFrameCount, camera_sn);
            FPAfile.open(textFileName); // open text file

            FPAfile << currentFrame.getFPA(); // read in fpa value
            sprintf(filename, "%lu-raw16_%d.tiff", classFrameCount, camera_sn); // create file name for the picture
            cv::imwrite(filename, currentFrame.getImage()); // save the grey16 image
            FPAfile.close();
            // add one to count
            classFrameCount++;
        }
std::cout << ""; // I do nt know why the code only works if there is a cout call here
    }

    while (!frameQueue.empty()) { // this will get the frames that are left over once the streaming has been ended
        Frame currentFrame = frameQueue.front(); // store current frame
        frameQueue.pop(); // remove frame
        std::ofstream FPAfile; // txt file to hold the fpa value
        char textFileName[30]; // file name for the fpa value for the image
        char filename[60];  // file name for the image
        
        sprintf(textFileName, "%lu-raw16_%lu.txt", classFrameCount, camera_sn);
        FPAfile.open(textFileName); // open text file

        FPAfile << currentFrame.getFPA(); // read in fpa value
        sprintf(filename, "%lu-raw16_%d.tiff", classFrameCount, camera_sn); // create file name for the picture
        cv::imwrite(filename, currentFrame.getImage()); // save the grey16 image
        FPAfile.close();
        // add one to count
        classFrameCount++;
    }

    std::cout << "All frames saved" << std::endl;
    std::cout << "Camera " << camera_sn << " captured " << classFrameCount << " frames \n";
}



// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
cv::Mat Boson_Camera::AGC_Basic_Linear(cv::Mat input_16, int height, int width) {
    cv::Mat newFrame(height, width, CV_8UC1);
    cv::normalize(input_16, newFrame, 0, 255, cv::NORM_MINMAX);
    newFrame.convertTo(newFrame, CV_8UC1);
    return newFrame;
}

// does not record anything and just provides a stream of the camera images
void Boson_Camera::Stream() {
    cv::Mat thermalFrame;
    cv::Mat linearFrame;
    int count = 0;

    for(;;) {
        cap >> thermalFrame; // read in frame
        
        linearFrame = AGC_Basic_Linear(thermalFrame, height, width);

        cv::imshow("Gray 8", linearFrame);

        // Press 'q' to exit
        if( cv::waitKey(1) == 'w' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
            uint16_t fpa; // focal plane area
            std::ofstream FPAfile; // txt file to hold the fpa value
            char textFileName[30]; // file name for the fpa value for the image
            char filename[60];  // file name for the image
            
            sprintf(textFileName, "%d-raw16_%lu.txt", count, camera_sn);
            FPAfile.open(textFileName); // open text file

            cap >> thermalFrame; // read in frame

            // get the fpa value
            cam.roicGetFPATemp(&fpa);
            FPAfile << fpa; // read in fpa value
            sprintf(filename, "%d-raw16_%d.tiff", count, camera_sn); // create file name for the picture
            cv::imwrite(filename, thermalFrame); // save the grey16 image
            FPAfile.close();

            count++;
        }

        // Press 'q' to exit
        if( cv::waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
            break;
        }
    }

}

// gets the sensor temp in celcius. This is the calibrated FPA value.
float Boson_Camera::getSensorTemp() {
    FLR_RESULT result;
    int16_t fpa_temp = 0;
    float cpu_temp = 0;
    result = cam.bosonlookupFPATempDegCx10(&fpa_temp);
    if (result)
        {
                printf("Failed FPATemp with status 0x%08X, exiting.\n",result);
                exit(1);
        }
    cpu_temp = fpa_temp / 10.0;
    return cpu_temp;
} 

// The framerate of the camera in frames per second (60/30 or 9)
uint32_t Boson_Camera::getFrameRate() {
    FLR_RESULT result;
    uint32_t frameRate = 0;
    result = cam.sysctrlGetCameraFrameRate(&frameRate);
    if (result)
        {
                printf("Failed frame rate with status 0x%08X, exiting.\n",result);
                exit(1);
        }
    return frameRate;
}
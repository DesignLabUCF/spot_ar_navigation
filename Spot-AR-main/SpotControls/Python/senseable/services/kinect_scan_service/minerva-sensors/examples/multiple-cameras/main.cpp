#include "Boson_Camera.h"
#include "UART_Connector_wrapper.h"
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <linux/videodev2.h>
#include <chrono>
#include <thread>
#include <mutex>

// thread function for writing
void Write(Boson_Camera* camera) {
    camera->WriteFrame();
}

// thread function for capturing
void Capture(Boson_Camera* camera) {
    camera->CaptureFrame();
}

int main(int argc, char** argv ) {
    int fd;
    char video[20];   // To store Video Port Device
    struct v4l2_capability cap;
    // max cameras 6
    int devicesLength = 7; // in the case of web cam
    int devices[devicesLength];
    int record_enable = 0; // by default it will not be set to record
    int portList[6] = {16, 17, 18, 19, 20, 21};
    char foldername[30];


    // Read command line arguments
    for (int i=0; i < argc; i++) {
    
        // Check if recording is enabled
        if ( argv[i][0]=='r') {  // File name has to be more than two chars
            record_enable = 1;
        }
        
    }

    // brute force opening multiple cameras
    int j = 0;
    int k = 0;
    while ( j < devicesLength && k <= 11) {
        for (k = 0; k <= 11; k+=2 ){ // not sure how to filter out the metadata files because they are being recognized as video sources, so this is just skipping the metadata under the assumption that the second file is the
            // We open the Video Device
            sprintf(video, "/dev/video%i", k);
            printf(">>>%s selected\n", video);
            
            if((fd = open(video, O_RDWR)) >= 0){
                // Check VideoCapture mode is available
                if(ioctl(fd, VIDIOC_QUERYCAP, &cap) >= 0){
                    if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
                        if (ioctl(fd, VIDIOC_G_INPUT) == 0) {
                            std::string card;
                            card.append(reinterpret_cast<char*>(cap.card));
                            if (card.substr(0, 5) == "Boson") { // checks that device is boson camera
                                devices[j] = k; // adds index to device list
                                j++;
                            } 
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


    // Split based on if it is recording or not
    // set up basic process for each

    if (record_enable == 1) { // recording
        if (devicesLength == 1) {
            // set up pointer for camera object
            Boson_Camera* myCam = new Boson_Camera(portList[0], devices[0]);
            std::cout << "Recording \n";

            // create folder for the camera and create it if it doesnt already exist
            sprintf(foldername, "BosonImagesTESTER");
            mkdir(foldername, 0700);
            chdir(foldername);


            // starting the write thread
            std::thread writeThread(Write, std::move(myCam));
            for(;;) {
            // for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds{3}; now = std::chrono::steady_clock::now()) {

                // myCam.ReadWriteFrame(count); // old method 
                myCam->CaptureFrame(); // capture one frame at a time
                // Press 'q' to exit
                if( cv::waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
                    break;
                }
                
            }
            // stop streaming
            myCam->isStreaming = false;
            // join the write thread
            writeThread.join();
        } else if (devicesLength == 6) {
            // set up pointers to all 6 camera objects
            Boson_Camera* camera1 = new Boson_Camera(portList[0], devices[0]);
            Boson_Camera* camera2 = new Boson_Camera(portList[1], devices[1]);
            Boson_Camera* camera3 = new Boson_Camera(portList[2], devices[2]);
            Boson_Camera* camera4 = new Boson_Camera(portList[3], devices[3]);
            Boson_Camera* camera5 = new Boson_Camera(portList[4], devices[4]);
            Boson_Camera* camera6 = new Boson_Camera(portList[5], devices[5]);

            // create folder for the camera and create it if it doesnt already exist
            sprintf(foldername, "BosonImagesTESTER");
            mkdir(foldername, 0700);
            chdir(foldername); // EJP: TODO this part may cause issues and the filename might just need to be expanded


            // write threads
            std::thread th1(Write, std::move(camera1));
            std::thread th3(Write, std::move(camera2));
            std::thread th5(Write, std::move(camera3));
            std::thread th7(Write, std::move(camera4));
            std::thread th9(Write, std::move(camera5));
            std::thread th11(Write, std::move(camera6));


            // capture threads
            // std::thread th2(Capture, std::move(camera1));
            // std::thread th4(Capture, std::move(camera2));
            // std::thread th6(Capture, std::move(camera3));
            // std::thread th8(Capture, std::move(camera4));
            // std::thread th10(Capture, std::move(camera5));
            // std::thread th12(Capture, std::move(camera6));

            //join capture threads
            // th2.join();
            // th4.join();
            // th6.join();
            // th8.join();
            // th10.join();
            // th12.join();


            for(int i = 0; ; i++) { // will run until the user presses 'q'
            // for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds{120}; now = std::chrono::steady_clock::now()) {

                camera1->CaptureFrame();
                camera2->CaptureFrame();
                camera3->CaptureFrame();
                camera4->CaptureFrame();
                camera5->CaptureFrame();
                camera6->CaptureFrame();
                // Press 'q' to exit
                if( cv::waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
                    break;
                }
            }
            // turn off the streaming flag for each camera
            camera1->isStreaming = false;
            camera2->isStreaming = false;
            camera3->isStreaming = false;
            camera4->isStreaming = false;
            camera5->isStreaming = false;
            camera6->isStreaming = false;
            
            // join write threads
            th1.join();
            th3.join();
            th5.join();
            th7.join();
            th9.join();
            th11.join();

        } else {
            std::cout << "Check that all 6 cameras are connected correctly \n";
            return 1;
        }
    } else { // not recording
        sprintf(foldername, "StreamImages");
        mkdir(foldername, 0700);
        chdir(foldername);
        Boson_Camera myCam(portList[0], devices[0]);
        std::cout << "Streaming \n";
        myCam.Stream();
    }



    return 0;
}
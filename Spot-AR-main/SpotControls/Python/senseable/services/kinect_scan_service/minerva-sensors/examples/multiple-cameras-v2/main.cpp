#include <iostream>
#include <fstream>

#include <vector>
#include <queue>

#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <omp.h>

#include <sys/stat.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>

#include <linux/videodev2.h>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include "timer.hpp"
#include "keyboard.hpp"
#include "Boson_Camera.h"
#include "Frame.h"
#include "UART_Connector_wrapper.h"
#include "FlirBosonRawCountsToTemperatureConversion.h"


const int devicesLength = 7;   // +1 in the case of web cam 
int devices[devicesLength];    // max IR cameras 6
int portList[6] = {16, 17, 18, 19, 20, 21};
struct v4l2_capability cap;

std::vector< std::queue<Frame> > mImageQueue; // vector that stores the image queues for each camera
std::vector< uint > mFrameCount; // vector that stores the frame count of the images, is intended to be in sync with the image queues
std::vector<Boson_Camera*> mCameras; // vector that stores all of the camera objects that have be instantiated
int detectedLength = -1; // number of cameras found
int record_enable = 0; // by default it will not be set to record


// std::ofstream FPAfile; // txt file to hold the fpa value
// char textFileName[30]; // file name for the fpa value for the image
char filename[80];  // file name for the image


using namespace std::chrono;
std::mutex mtConsole;
void writer_thread_funtion(int thread_number, int totalDevices)
{
    int count = 0;
    //uint16_t fpa;
    double numOfDevices = 1.0 * totalDevices;
    Frame currentFrame;
    omp_lock_t countlock;
    omp_init_lock(&countlock);
    Timer timetowrite(true);
    while (record_enable)
    {
        if (totalDevices % 2 == 0) { // even number of devices
            for(int i=thread_number*(numOfDevices/2); i< (numOfDevices/2) + thread_number*(numOfDevices/2); i++){
                if (mImageQueue[i].size() > 0) 
                {
                    //timetowrite.Reset();
                    omp_set_lock(&countlock); // lock added since the frame count will encounter race conditions
                    currentFrame = mImageQueue[i].front();
                    sprintf(filename, "%u-raw_%d_%d_%ld.tiff", mFrameCount[i], currentFrame.getFPA(), currentFrame.getSerialNum(), currentFrame.ms); // create file name for the picture
                    try{
                    cv::imwrite(filename, currentFrame.getImage()); // save the grey16 image
                    }
                    catch(const std::exception &e)
                    {std::cout<<"error writing at :" << i << " " << mFrameCount[i] <<std::endl;}

                    mImageQueue[i].pop();
                    mFrameCount[i]++;
                    omp_unset_lock(&countlock);
                    count=0;
                    
                }
                else{
                count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
        } else { // odd number of devices
            for(int i=thread_number*ceil(numOfDevices/2); i<ceil(numOfDevices/2) + thread_number*floor(numOfDevices/2); i++){
                if (mImageQueue[i].size() > 0) 
                {
                    //timetowrite.Reset();
                    currentFrame = mImageQueue[i].front();
                    sprintf(filename, "%u-raw_%d_%d_%ld.tiff", mFrameCount[i], currentFrame.getFPA(), currentFrame.getSerialNum(), currentFrame.ms); // create file name for the picture
                    try{
                    cv::imwrite(filename, currentFrame.getImage()); // save the grey16 image
                    }
                    catch(const std::exception &e)
                    {std::cout<<"error writing at :" << i << " " << mFrameCount[i] <<std::endl;}

                    mImageQueue[i].pop();
                    mFrameCount[i]++; 
                    count=0;
                    
                }
                else{
                count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
        }



        if (count > 50) { 
            std::cout<<"Exiting Thread.."<<std::endl; 
            // std::cout << "Average time to capture: " << totaltime / (mFrameCount[0] * detectedLength) << " milliseconds \n";
            break;
            }  //this is a shitty way to pause to check its empty.

    }
    return;
}



int main(int argc, char** argv ) 
{
    int fd;
    char video[20];   // To store Video Port Device
    char foldername[60];
    Timer timer(true);
    float etime = 0.0f;


    std::cout << "Multithread OMP MultiCapture" << std::endl;
    std::cout << "Number of threads detected on machine = " 
              <<  std::thread::hardware_concurrency() << std::endl;


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

    std::cout << "   ...there were " << j << " number of devices detected" << std::endl;
    if (j == 0) {
        exit(1);
    }
    detectedLength = j;




    if(record_enable){ // if recording

        // create folder for the camera and create it if it doesnt already exist
        sprintf(foldername, "BosonImages");
        //sprintf(foldername, "/media/sensable/Minerva/images/");
        mkdir(foldername, 0700);
        chdir(foldername); // EJP: TODO this part may cause issues and the filename might just need to be expanded
  

        std::cout << "\nSetting up Cameras and Recording Queues... \n";
        for (int i = 0; i < detectedLength; i++){
            std::cout << "Intializing Camera : " << i <<  "\n";
            std::cout << "-----------------------------\n";

            //Set up individual Queues for the camera
            std::queue<Frame> v1;
            mImageQueue.push_back(v1);
            mFrameCount.push_back(0);

            // Set up pointers to all 6 camera objects
            mCameras.push_back( new Boson_Camera(portList[i], devices[i]) );
            mCameras[i]->isStreaming = false; // unnecessary because the function this is used for is not being called since Joe rewrote everything for some reason that is unclear to me 
        }

        // starts the writer threads, 2 used since the backlog of only using one would cause the program to be killed
        std::thread worker_thread0(writer_thread_funtion, 0, detectedLength); 
        std::thread worker_thread1(writer_thread_funtion, 1, detectedLength); 


        std::cout << "  Recording IR Images... \n";
        int i = 0;
        omp_set_num_threads(detectedLength); // creates a thread for each camera
        omp_lock_t writelock;
        omp_init_lock(&writelock);
        cKeyboard kb;

        using clock = std::chrono::steady_clock;
        auto next_frame = clock::now(); // used to make sure the frames are captured at 30 FPS

        timer.Reset(); // starts timer 


        // while (i < 30) {
        while (true){
            next_frame += std::chrono::milliseconds(33);  // sets the time that needs to be waited for until the next frame is captured
            #pragma omp parallel for 
            for(int j=0; j<detectedLength; j++)
            {   
                //t = 5*omp_get_thread_num();
                //std::this_thread::sleep_for(std::chrono::milliseconds(t));       
                   omp_set_lock(&writelock);
                   try{
                    mImageQueue[omp_get_thread_num()].push( mCameras[ omp_get_thread_num() ]->CaptureFrame() ); // captures frame from each camera and stores it to the corresponding camera's queue
                   } catch (const std::exception &e){std::cout<<"error cap at :" << omp_get_thread_num() << " " <<std::endl;}
                    omp_unset_lock(&writelock);
                    std::this_thread::sleep_until(next_frame);                
            }
            i++;

            if( kb.getKeyState(KEY_ESC) ){ std::cout << "CAUGHT A BREAK!" << std::endl; break;}

        }
        etime = timer.Elapsed().count();
        std::cout << "Elapsed time: " << std::fixed << etime << "ms , (seconds: "<< etime/1000 <<")\n";
        kb.~cKeyboard();    
        worker_thread0.join();
        worker_thread1.join();
        std::cout << "Joined to main thread: "<< mFrameCount[0] << std::endl;
 

    }
    else { // not recording
        Boson_Camera myCam(portList[0], devices[0]);
        std::cout << "  Streaming IR Images... \n";
        myCam.Stream();
    }

    std::cout<<mImageQueue[0].size()<<std::endl;
    std::cout << "Exiting cleanly... " << std::endl;



   // Return success
   return 1;
}




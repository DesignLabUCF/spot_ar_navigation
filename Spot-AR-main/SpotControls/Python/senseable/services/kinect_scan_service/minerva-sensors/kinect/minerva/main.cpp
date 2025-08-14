// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "utilities.h"



std::mutex mtConsole;
void writer_thread_funtion(const uint32_t device_count)
{
    std::stringstream ofilename; // file name for the image
    std::string directory = "KinectImagesTEST/";
    int count = 0;

    uint8_t* colorBuffer = NULL;
    uint8_t* depthBuffer = NULL;
    uint8_t* irBuffer = NULL;
    int rows;
    int cols;

    while (true)
    {
        
        for(uint i=0; i<device_count; i++){
            if (mImageQueue[i].size() > 0)
            {
                Frame currentFrame = mImageQueue[i].front();

                colorBuffer = currentFrame.rgbcapture.get_buffer();
                rows = currentFrame.rgbcapture.get_height_pixels();
                cols = currentFrame.rgbcapture.get_width_pixels();
                cv::Mat colorMat(rows , cols, CV_8UC4, (void*)colorBuffer, cv::Mat::AUTO_STEP);
                //cv::imwrite("Image-color.tiff", colorMat);

                ofilename<<directory<<mFrameCount[i] 
                    << "_RGB_" << currentFrame.getSerialNum() 
                    << "_"<< currentFrame.rgbcapture.get_system_timestamp().count()
                    << "_"<< currentFrame.rgbcapture.get_device_timestamp().count()<<".jpg"; // create file name for the picture
                cv::imwrite(ofilename.str() , colorMat);
            //    std::cout<<ofilename.str()<<std::endl;
                ofilename.str(""); 


                // convert the raw buffer to cv::Mat
                depthBuffer = currentFrame.depthcapture.get_buffer();
                rows = currentFrame.depthcapture.get_height_pixels();
                cols = currentFrame.depthcapture.get_width_pixels();
                cv::Mat depthMat(rows, cols, CV_16UC1, (void*)depthBuffer, cv::Mat::AUTO_STEP);
                ofilename<<directory<<mFrameCount[i] 
                    << "_DEPTH_raw16_" << currentFrame.getSerialNum() 
                    << "_"<< currentFrame.depthcapture.get_system_timestamp().count()
                    << "_"<< currentFrame.depthcapture.get_device_timestamp().count()<<".tiff"; // create file name for the picture
                cv::imwrite(ofilename.str(), depthMat);
                ofilename.str(""); 


                // convert the raw buffer to cv::Mat
                irBuffer = currentFrame.ircapture.get_buffer();
                rows = currentFrame.ircapture.get_height_pixels();
                cols = currentFrame.ircapture.get_width_pixels();
                cv::Mat irMat(rows, cols, CV_16UC1, (void*)irBuffer, cv::Mat::AUTO_STEP);
                ofilename<<directory<<mFrameCount[i] 
                    << "_IR_raw16_" << currentFrame.getSerialNum() 
                    << "_"<< currentFrame.ircapture.get_system_timestamp().count()
                    << "_"<< currentFrame.ircapture.get_device_timestamp().count()<<".tiff"; // create file name for the picture
                cv::imwrite(ofilename.str(), irMat);
                ofilename.str(""); 


                //IntializeEncoder(9 | LZMA_PRESET_EXTREME);
                //compress(colorBuffer, colorImage.get_size());
                //std::cout<<colorImage.get_size()<<std::endl;
                //std::cout<<colorImage.get_format()<<std::endl;
                //std::cout<<colorImage.get_stride_bytes()<<std::endl<<std::endl;
                //std::cout<<(int)colorBuffer[0]<<std::endl;

                //std::cout<<depthImage.get_size()<<std::endl;
                //std::cout<<depthImage.get_format()<<std::endl;
                //std::cout<<depthImage.get_stride_bytes()<<std::endl<<std::endl;

                //std::cout<<irImage.get_size()<<std::endl;
                //std::cout<<irImage.get_format()<<std::endl;
                //std::cout<<irImage.get_stride_bytes()<<std::endl<<std::endl;

                mImageQueue[i].pop();
                mFrameCount[i]++; 
                count=0;
            }
            else{
             count++;
             std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
        if (count > 52000) { std::cout<<"Exiting Thread Empty.."<<std::endl; break;}  //this is a shitty way to pause to check its empty.

    }

    //delete colorBuffer;
    //delete depthBuffer;
    //delete irBuffer;
    return;
}



int main()
{
    try
    {
        char foldername[60];
        Timer timer(true);
        float etime = 0.0f;
        int result = 0;

        // create folder for the camera and create it if it doesnt already exist
        sprintf(foldername, "KinectImagesTEST"); 
        mkdir(foldername, 0700);
        result = chdir(foldername); 

        if (result) {
            std::cout << "yippee\n";
        }

        // Check for devices
        const uint32_t device_count = k4a::device::get_installed_count();
        if (device_count == 0)
        {
            throw std::runtime_error("No Azure Kinect devices detected!");
        }
        std::cout<<"Number of Devices detected: "<< device_count <<"\n";
        
        OpenDetectedDevices(device_count);
        SetUpDataStorage(device_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::thread worker_thread(writer_thread_funtion, device_count); 


        // Poll the device for new image data.
            //
          // We set the timeout to 0 so we don't block if there isn't an available frame.
          //
          // This works here because we're doing the work on the same thread that we're
          // using for the UI, and the ViewerWindow class caps the framerate at the
          // display's refresh rate (the EndFrame() call blocks on the display's sync).
          //
          // If we don't have new image data, we'll just reuse the textures we generated
         // from the last time we got a capture.
        //
        k4a::capture capture;

         std::cout << "  Recording KINECT Images... \n";
        int i = 0;
        omp_set_num_threads(device_count);
        omp_lock_t writelock;
        omp_init_lock(&writelock);
        cKeyboard kb;

        using clock = std::chrono::steady_clock;
        auto next_frame = clock::now();
        //bool ret;

        //while (i < 10) {
        while (true){
            next_frame += std::chrono::milliseconds(33); //33
            #pragma omp parallel for 
            for(uint j=0; j<device_count; j++)
            {   
                omp_set_lock(&writelock);
                
                try{
                mCameras[omp_get_thread_num() ].get_capture(&mCaptures[omp_get_thread_num() ], std::chrono::milliseconds(-1));
                //ret = mCameras[omp_get_thread_num() ].get_capture(&mCaptures[omp_get_thread_num() ], std::chrono::milliseconds(-1));
                //std::cout<<i<<": " <<ret<<std::endl;
                }
                catch (const std::exception &e)
                {}

                try{
                Frame newFrame(mCaptures[omp_get_thread_num() ].get_color_image(),mCaptures[omp_get_thread_num() ].get_depth_image(),mCaptures[omp_get_thread_num() ].get_ir_image(), mCameras[omp_get_thread_num()].get_serialnum ());
           mImageQueue[omp_get_thread_num()].push( newFrame );
                }
                catch (const std::exception &e)
                {}
     
                omp_unset_lock(&writelock);
                mCaptures[omp_get_thread_num() ].reset();
                std::this_thread::sleep_until(next_frame);
            }
            i++;
            if( kb.getKeyState(KEY_ESC) ){ std::cout << "CAUGHT A BREAK!" << std::endl; break;}

        }
        etime = timer.Elapsed().count();


        
        std::cout << "Elapsed time: " << std::fixed << etime << "ms , (seconds: "<< etime/1000 <<")\n";
        worker_thread.join();
        std::cout << "Joined to main thread: "<< mFrameCount[0] << std::endl;
        CloseDetectedDevices(device_count);
        //kb.~cKeyboard();  
        std::cout << "Exiting ... " << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }

    //std::cout<<mImageQueue[0].size()<<std::endl;
    std::cout << "Exiting cleanly... " << std::endl;

    return (int)0;
}

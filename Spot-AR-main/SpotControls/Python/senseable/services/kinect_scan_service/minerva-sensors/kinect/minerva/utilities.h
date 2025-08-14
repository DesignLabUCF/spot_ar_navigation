#include <vector>
#include <queue>
#include<string>
#include <sstream>

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <k4a/k4a.hpp>
#include <lzma.h>

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "timer.hpp"
#include "keyboard.hpp"
#include "Frame.h"
#include "timer.hpp"

#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <omp.h>

#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>


std::vector<k4a::device> mCameras;
std::vector< std::queue<Frame> > mImageQueue;
std::vector< uint > mFrameCount;
std::vector< k4a::capture > mCaptures;

lzma_stream strm;


#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()
#define IN_BUF_MAX  409600
#define OUT_BUF_MAX 409600
#define CHUNK_SIZE 409600


void SetUpDataStorage(uint32_t device_count)
{
    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        //Set up individual Queues for the camera
        std::queue<Frame> v1;
        mImageQueue.push_back(v1);
        mFrameCount.push_back(0);
        k4a::capture cap;
        cap.reset ();
        mCaptures.push_back(cap);
    }
}


void OpenDetectedDevices(uint32_t device_count)
{
    FILE* outfile;
    char filename[120]; 
    k4a_calibration_t calibration;

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        std::cout << "Started opening K4A device..." <<(int)deviceIndex<< std::endl;

        mCameras.push_back(k4a::device::open(deviceIndex)); // pushes handle from device into camera vector
        if (!mCameras[deviceIndex])
            throw std::runtime_error(SSTR("Failed to open device: "<<deviceIndex<< "\n"));
        


        // The start_cameras() function requires a device configuration which
        // specifies the modes in which to put the color and depth cameras.
        // See DeviceConfiguration, EImageFormat, EColorResolution, EDepthMode,
        // EFramesPerSecond, and EWiredSyncMode.
        //
        k4a_wired_sync_mode_t sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
        uint32_t delay_off_master_usec = 160; // Delay off master to avoid IR interference.
        if (mCameras[deviceIndex].is_sync_out_connected() == true && mCameras[deviceIndex].is_sync_in_connected () == false)
        {
            std::cout << "    Master Device.... \n";
            sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
            delay_off_master_usec = 0; // The master must have 0 as subordinate delay.
        }


        // Start the device
        //
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_2160P;  //_1080P // _1440P //_2160P //_3072P

        // This means that we'll only get captures that have both color and
        // depth images, so we don't need to check if the capture contains
        // a particular type of image.
        //
        config.synchronized_images_only = true;
        config.depth_delay_off_color_usec=0,
        config.wired_sync_mode=sync_mode;
        config.subordinate_delay_off_master_usec=delay_off_master_usec;
        config.disable_streaming_indicator=false;

        // Set color exposure time to manual. Recommended for synchronicity, otherwise
        // differences in auto exposure between devices may cause drift to bring the devices out of sync.
        mCameras[deviceIndex].set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_MANUAL,
            33330);


        // open up a file to print the calibration stuff onto 
        sprintf(filename, "/home/sdl/Documents/Azure-Kinect-Sensor-SDK/build/calibration-%s.bin", mCameras[deviceIndex].get_serialnum().c_str());
        outfile = fopen(filename, "wb");
        if (outfile == NULL) {
            fprintf(stderr, "\nError opened file\n");
            exit(1);
        }

        // get calibration information
        k4a_result_t result = k4a_device_get_calibration(mCameras[deviceIndex].handle(), config.depth_mode, config.color_resolution, &calibration);

        if (result == K4A_RESULT_SUCCEEDED) {
            std::cout << "Got calibration\n";
        }
        // write struct to file
        int flag = 0;
        flag = fwrite(&calibration, sizeof(struct _k4a_calibration_t), 1, outfile);
        if (flag) {
            printf("Contents of the structure written successfully\n");
        } else
            printf("Error Writing to File!");



        mCameras[deviceIndex].start_cameras(&config);
        std::cout << "    Serial Number: " << mCameras[deviceIndex].get_serialnum () << "\n";

        // close file
        fclose(outfile);

    }
}


void CloseDetectedDevices(uint32_t device_count)
{
    bool ret = true;
    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        std::cout << "Closing K4A device..." <<(int)deviceIndex<< std::endl;

        mCameras[deviceIndex].stop_cameras();
        std::cout<<(int)deviceIndex<<": " <<ret<<std::endl;
        mCameras[deviceIndex].close();
        std::cout<<(int)deviceIndex<<": " <<ret<<std::endl;
    }
}


bool IntializeEncoder(uint32_t preset)
{
    strm = LZMA_STREAM_INIT;
    lzma_ret ret = lzma_easy_encoder(&strm, preset, LZMA_CHECK_CRC64);

    if (ret == LZMA_OK) // Return successfully if the initialization went fine.
        return true;


    // Something went wrong. The possible errors are documented in
    // lzma/container.h (src/liblzma/api/lzma/container.h in the source
    // package or e.g. /usr/include/lzma/container.h depending on the
    // install prefix).
    const char *msg;
    switch (ret) {
    case LZMA_MEM_ERROR:
        msg = "Memory allocation failed";
        break;

    case LZMA_OPTIONS_ERROR:
        msg = "Specified preset is not supported";
        break;

    case LZMA_UNSUPPORTED_CHECK:
        msg = "Specified integrity check is not supported";
        break;

    default:
        // This is most likely LZMA_PROG_ERROR indicating a bug in
        // this program or in liblzma. It is inconvenient to have a
        // separate error message for errors that should be impossible
        // to occur, but knowing the error code is important for
        // debugging. That's why it is good to print the error code
        // at least when there is no good error message to show.
        msg = "Unknown error, possibly a bug";
        break;
    }

    fprintf(stderr, "Error initializing the encoder: %s (error code %u)\n",
            msg, ret);
    return false;

}




void compress(const uint8_t* buffer, size_t bsize) {
    const char* outputFile = "Image-compressed.xz";
    // Initialize the encoder
    //lzma_stream strm = LZMA_STREAM_INIT;
    lzma_ret ret = lzma_easy_encoder(&strm, LZMA_PRESET_DEFAULT, LZMA_CHECK_CRC64);
    if (ret != LZMA_OK) {
        //fprintf(stderr, "Error initializing the encoder: %s\n", lzma_strerror(ret));
        return;
    }

    // Set the input buffer
    strm.next_in = buffer;
    strm.avail_in = bsize;

    // Open the output file
    FILE* file = fopen(outputFile, "wb");
    if (file == NULL) {
        fprintf(stderr, "Error opening the output file\n");
        lzma_end(&strm);
        return;
    }

    // Create an output buffer
    uint8_t* compressedBuffer = (uint8_t*)malloc(CHUNK_SIZE);
    if (compressedBuffer == NULL) {
        fprintf(stderr, "Memory allocation error\n");
        fclose(file);
        lzma_end(&strm);
        return;
    }

    // Compress the data in chunks and write to the output file
    size_t compressedSize = 0;
    do {
        // Set the output buffer
        strm.next_out = compressedBuffer;
        strm.avail_out = CHUNK_SIZE;

        // Compress the chunk
        ret = lzma_code(&strm, LZMA_FINISH);
        if (ret != LZMA_OK && ret != LZMA_STREAM_END) {
            //fprintf(stderr, "Compression error: %s\n", lzma_strerror(ret));
            free(compressedBuffer);
            fclose(file);
            lzma_end(&strm);
            return;
        }

        // Calculate the compressed size
        size_t chunkSize = CHUNK_SIZE - strm.avail_out;
        compressedSize += chunkSize;

        // Write the compressed chunk to the output file
        fwrite(compressedBuffer, 1, chunkSize, file);

    } while (ret != LZMA_STREAM_END);

    // Cleanup
    free(compressedBuffer);
    fclose(file);
    lzma_end(&strm);
}
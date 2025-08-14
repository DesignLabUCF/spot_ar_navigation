/*
Uses OpenCV to record a video of two camera streams at once. The colormapping can be changes by pressing the 'w', 'j', or 'h' keys. Press 'ESC' to stop the program. run the program with 'r' if recording is desired.
*/

#include <stdio.h>
#include <fcntl.h> // open, O_RDWR
#include <opencv2/opencv.hpp>
// #include <unistd.h>    // close
// #include <sys/ioctl.h> // ioctl
// #include <asm/types.h> // videodev2.h
// #include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <opencv2/videoio.hpp>

#include <stdbool.h>    ///********************Fixes   _Bool????????????????????????????????? #JTK


extern "C"
{
#include "Client_API.h"
#include "EnumTypes.h"
#include "UART_Connector.h"

}

#define YUV 0
#define RAW16 1

using namespace cv;
using namespace std;

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLU "\x1B[34m"
#define MAG "\x1B[35m"
#define CYN "\x1B[36m"
#define WHT "\x1B[37m"
#define RESET "\x1B[0m"

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

// Global variables to keep this simple
int width;
int height;

// Types of sensors supported
enum sensor_types
{
    Boson320,
    Boson640
};


/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int main(int argc, char **argv)
{
    bool record = false;

    for (int i = 0; i < argc; i++)
    {
        // Check if RAW16 video is desired
        if (argv[i][0] == 'r')
        {
            record = true;
        }
    }

        // testing bosonSDK
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

        // Step 2: Get Camera Serial Number
        printf("CameraSN: ");
        uint32_t camera_sn;
        result = bosonGetCameraSN(&camera_sn);
        if (result)
        {
            printf("Failed CameraSN with status 0x%08X, exiting.\n", result);
            Close();
            return 1;
        }
        printf("%d \n", camera_sn);
        printf("SUCCESS\n\n");

        VideoCapture cap(0, CAP_V4L);
        VideoCapture cap2(2, CAP_V4L);
        VideoCapture cap3(4, CAP_V4L);
        VideoCapture cap4(6, CAP_V4L);
        VideoCapture cap5(8, CAP_V4L);
        VideoCapture cap6(10, CAP_V4L);

        cout << ">>>> Press 'h' for COLORMAP_HOT" << endl;
        cout << ">>>> Press 'j' for COLORMAP_JET" << endl;
        cout << ">>>> Press 'w' for default" << endl;
        cout << ">>>> Press ESC for to escape" << endl;

        // connect ti flir serial virtual COM Port

        ////set high gain
        // bosonSetGainMode(0);
        ////get serial number
        // auto const serialNumber = bosonGetCameraSN();

        // check for camera
        if (!cap.isOpened())
        {
            cout << "Error opening video stream or file" << endl;
            return -1;
        }
        // check for camera
        if (!cap2.isOpened())
        {
            cout << "Error opening video stream or file from second camera" << endl;
            return -1;
        }
        // check for camera
        if (!cap3.isOpened())
        {
            cout << "Error opening video stream or filefrom 3rd camera" << endl;
            return -1;
        }
        // check for camera
        if (!cap4.isOpened())
        {
            cout << "Error opening video stream or file from fourth camera" << endl;
            return -1;
        }
        // check for camera
        if (!cap5.isOpened())
        {
            cout << "Error opening video stream or file from fifth camera" << endl;
            return -1;
        }
        // check for camera
        if (!cap6.isOpened())
        {
            cout << "Error opening video stream or file from sixth camera" << endl;
            return -1;
        }

        // gets the resolutions and creates a video writer object
        int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

        //initialize but don't use unless record is checked
        VideoWriter video("sixstreams.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 60, Size(frame_width * 3, frame_height *2));

        int colorMap = -1;

        while (1)
        {
            Mat frame; // to store the final concat version
            Mat frametmp;

            vector<Mat> imglist;
            vector<Mat> imglist2;

            Mat frame1;
            Mat frame2;
            Mat frame3;
            Mat frame4;
            Mat frame5;
            Mat frame6;

            Mat coloredFrame;

            cap >> frame1;
            cap2 >> frame2;
            cap3 >> frame3;
            cap4 >> frame4;
            cap5 >> frame5;
            cap6 >> frame6;


            if (frame1.empty() || frame2.empty() || frame3.empty() || frame4.empty() || frame5.empty() || frame6.empty())
            {
                break;
            }

            imglist.push_back(frame1);
            imglist.push_back(frame2);
            imglist.push_back(frame3);

            imglist2.push_back(frame4);
            imglist2.push_back(frame5);
            imglist2.push_back(frame6);

            hconcat(imglist, frame);
            hconcat(imglist2, frametmp);
            vconcat(frame, frametmp, frame);

            // asethetic changes
            if (colorMap != -1)
            {
                if (colorMap == 2)
                {
                    applyColorMap(frame, coloredFrame, COLORMAP_JET);
                }
                else if (colorMap == 11)
                {
                    applyColorMap(frame, coloredFrame, COLORMAP_HOT);
                }

                // saving video
                // saving video
                if (record) {
                    video.write(coloredFrame);
                }

                // display video
                imshow("Minerva IR Cameras", coloredFrame);
            }
            else
            {
                // saving video
                if (record) {
                    video.write(frame);
                }

                // display video
                imshow("Minerva IR Cameras", frame);
            }

            // exit video recording
            char c = (char)waitKey(1);
            if (c == 104)
            {
                colorMap = 11;
            }
            else if (c == 106)
            {
                colorMap = 2;
            }
            else if (c == 119)
            {
                colorMap = -1;
            }
            else if (c == 27)
            {
                break;
            }
        }

        // when everything is done release the video capture object and then close all frames

        cap.release();
        if (record) {
            video.release();
        }
        destroyAllWindows();
        Close();

        return 0;
    }
#include "UART_Connector_wrapper.h"
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <fstream>
#include <sys/stat.h>
#include "Frame.h"
#include <queue>
using namespace std::chrono;

class Boson_Camera {
    private:
        UART_Connector_wrapper cam; //the flir camera component
        cv::VideoCapture cap; // opencv videocature component
        uint32_t camera_sn;
        int width = 640;
        int height = 512;
        int totalSize = width * height;
        std::queue<Frame> frameQueue;
        uint64_t classFrameCount = 0;
    public:
        Boson_Camera(int32_t camPort, int capPort);
        ~Boson_Camera() {
            cam.Close();
            cap.release();
            printf("\nCamera closed\n");
        }
        void PrintToFile(long frame, float *temperatureInCelsius);
        void ReadWriteFrame(long frameCount); 
        Frame CaptureFrame(); // requires a WriteFrame thread to run in order to empty out the queue as images are added to it
        void WriteFrame(); // requires CaptureFrame to be used for this method to do anything
        cv::Mat AGC_Basic_Linear(cv::Mat input_16, int height, int width);
        void Stream();
        float getSensorTemp();
        uint32_t getFrameRate();
        bool isStreaming = true; // Streaming flag that must be turned to false if using the WriteFrame method
};
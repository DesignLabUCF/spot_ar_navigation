#ifndef Frame_H
#define Frame_H

#include "UART_Connector_wrapper.h"
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <fstream>
#include <sys/stat.h>

class Frame {
    private:
        cv::Mat image; 
        uint16_t fpa;
    public:
        Frame(cv::Mat Newimage, uint16_t Newfpa){
            image = Newimage;
            fpa = Newfpa;
        }
        ~Frame() {
            
        }
        cv::Mat getImage() {
            return image;
        }
        uint16_t getFPA() {
            return fpa;
        }
};

#endif
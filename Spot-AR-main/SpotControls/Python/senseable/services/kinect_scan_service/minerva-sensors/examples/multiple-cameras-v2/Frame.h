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
        uint32_t serialnumber;
    public:
        Frame(){
            image = NULL;
            fpa = NULL;
            serialnumber = NULL;
            ms = NULL;
        }
        Frame(cv::Mat Newimage, uint16_t Newfpa, uint32_t NewSN, uint64_t NewMS){
            image = Newimage;
            fpa = Newfpa;
            serialnumber = NewSN;
            ms = NewMS;
        }
        ~Frame() {
            image.release();  
        }
        cv::Mat getImage() {
            return image;
        }
        uint16_t getFPA() {
            return fpa;
        }
        uint32_t getSerialNum(){
            return serialnumber;
        }
        uint64_t ms;
};

#endif
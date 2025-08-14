#ifndef Frame_H
#define Frame_H

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sys/stat.h>
#include <k4a/k4a.hpp>

class Frame {
    private:
        //uint32_t serialnumber;
        std::string serialnumber;
    public:
        Frame(const k4a::image NewRGBcapture, const k4a::image NewDcapture,const k4a::image NewIRcapture,
            const std::string NewSN){
            rgbcapture = NewRGBcapture;
            depthcapture = NewDcapture;
            ircapture = NewIRcapture;
            serialnumber = NewSN;
        }

        ~Frame() {
            
        }


        std::string getSerialNum(){
            return serialnumber;
        }


        k4a::image rgbcapture;
        k4a::image depthcapture;
        k4a::image ircapture;
};

#endif
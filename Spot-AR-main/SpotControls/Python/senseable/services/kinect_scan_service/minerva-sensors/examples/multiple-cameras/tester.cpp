#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <string>

#include <stdbool.h>    ///********************Fixes   _Bool????????????????????????????????? #JTK

// random number
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "UART_Connector_wrapper.h"


extern "C" {
    #include "Client_API.h"
    #include "EnumTypes.h"
    #include "UART_Connector.h"



    // #include "UART_HalfDuplex.h"
    // #include "flirChannels.h"
    // #include "flirCRC.h"
    // #include "rs232.h"

}


int main(int argc, char** argv )
{
    
    FLR_RESULT result;
    UART_Connector_wrapper myCam1;
    UART_Connector_wrapper myCam2;


       // Intitialize the Boson Camera
    result = myCam1.Initialize(16, 921600); // /dev/ttyACM0, 921600 baud
    printf("Initialize: ");
    if (result)
    {
        printf("Failed to initialize, exiting.\n");
        //Close();
        return 1;
    }
    printf("0x%08X\n", result);
    printf("SUCCESS\n\n");

  

    result = myCam2.Initialize(17, 921600); // /dev/ttyACM0, 921600 baud
    printf("Initialize: ");
    if (result)
    {
        printf("Failed to initialize, exiting.\n");
        //Close();
        return 1;
    }
    printf("0x%08X\n", result);
    printf("SUCCESS\n\n");



        // retrieve the serial number form camera in order to look up respective bin file
        printf("CameraSN: ");
        uint32_t camera_sn;
        result = myCam1.bosonGetCameraSN(&camera_sn);
        if (result)
        {
            printf("Failed CameraSN with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("%d \n", camera_sn);
        printf("SUCCESS\n\n");


        // retrieve the serial number form camera in order to look up respective bin file
        printf("CameraSN: ");
           result = myCam2.bosonGetCameraSN(&camera_sn);
        if (result)
        {
            printf("Failed CameraSN with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("%d \n", camera_sn);
        printf("SUCCESS\n\n");

        result = myCam1.bosonSetGainMode(FLR_BOSON_HIGH_GAIN);
        if (result)
        {
            printf("Failed GAIN with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("SUCCESS\n\n");

        result = myCam2.bosonSetGainMode(FLR_BOSON_HIGH_GAIN);
        if (result)
        {
            printf("Failed GAIN with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("SUCCESS\n\n");


        // runFFC for both cameras
        result = myCam1.bosonRunFFC();
        if (result)
        {
            printf("Failed FFC with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("FFC: SUCCESS\n\n");

        result = myCam2.bosonRunFFC();
        if (result)
        {
            printf("Failed FFC with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("FFC: SUCCESS\n\n");

        // get FPA
        uint16_t fpa;
        uint16_t fpa1;
        result = myCam1.roicGetFPATemp(&fpa);
        if (result)
        {
            printf("Failed FPA with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("FPA: %d SUCCESS\n\n", fpa);

        result = myCam2.roicGetFPATemp(&fpa1);
        if (result)
        {
            printf("Failed FFC with status 0x%08X, exiting.\n",result);
            //Close();
            return 1;
        }
        printf("FPA: %d SUCCESS\n\n", fpa1);

        int16_t fpa_temp = 0;
        float cpu_temp = 0;
        result = myCam1.bosonlookupFPATempDegCx10(&fpa_temp);
        if (result)
            {
                    printf("Failed FPATemp with status 0x%08X, exiting.\n",result);
                    Close();
                    return 1;
            }
        cpu_temp = fpa_temp / 10.0;
        printf("%.2f\n", cpu_temp);

        uint32_t frameRate = 0;
        myCam1.sysctrlGetCameraFrameRate(&frameRate);

        myCam1.Close();
        myCam2.Close();

    return 1;
}
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <k4a/k4a.hpp>
#include <sstream>


#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


void PrintDetectedDevices(uint32_t device_count)
{
    k4a_device_t device = NULL;

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
            throw std::runtime_error(SSTR("Failed to open device: "<<deviceIndex<< "\n"));


        size_t serial_number_length = 0;

        if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
        {
            k4a_device_close(device);
            device = NULL;
            throw std::runtime_error(SSTR("Failed to get serial number length: "<<deviceIndex<< "\n"));
            continue;
        }

        char *serial_number = NULL;
        serial_number = (char*)malloc(serial_number_length);
        if (serial_number == NULL)
        {
            k4a_device_close(device);
            device = NULL;
            throw std::runtime_error(SSTR("Failed to allocate "<< serial_number_length<<"bytes memory for serial number: "<<deviceIndex<< "\n"));
            continue;
        }

        if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
        {
            free(serial_number);
            serial_number = NULL;
            k4a_device_close(device);
            device = NULL;
            throw std::runtime_error(SSTR("Failed to get serial number: "<<deviceIndex<< "\n"));
            continue;
        }

        std::cout <<(int)deviceIndex <<": Device " << device.get_serialnum () <<  "\n";

       // if (device.SyncOutJackConnected == True && device.SyncInJackConnected == False)
       //     std::cout <<"Device Main!\n";

        k4a_device_close(device);
        device = NULL;
        free(serial_number);
        serial_number = NULL;
    }
}


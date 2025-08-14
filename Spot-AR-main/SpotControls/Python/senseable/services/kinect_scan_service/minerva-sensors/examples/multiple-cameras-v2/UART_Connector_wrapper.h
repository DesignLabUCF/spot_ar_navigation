#pragma once
#ifndef UART_CONNECTOR_WRAPPER_H
#define UART_CONNECTOR_WRAPPER_H
#include <stdbool.h>  
#include <stdint.h>
#include "ReturnCodes.h"
#include <stdio.h>
#include "serialPortAdapter.h"

#include "FSLP.h"
#include "flirCRC.h"
#include "flirChannels.h"
#include "timeoutLogic.h"
#include "EnumTypes.h"


#include "Serializer_Struct.h"
#include "FunctionCodes.h"
#include "Client_Interface.h"



#ifdef linux
#define DO_ERROR_TRACE(x...)    printf(x)
#define DO_DEBUG_TRACE(x...)    //printf(x)
#define DO_VERBOSE_TRACE(x...)  //printf(x)
#endif

#ifdef _MSC_VER
#define DO_ERROR_TRACE(...)    printf(__VA_ARGS__)
#define DO_DEBUG_TRACE(...)    //printf(x)
#define DO_VERBOSE_TRACE(...)  //printf(x)
#endif



//#include "UART_Connector.h"

#ifdef _WIN32
#define FLR_EXPORT __declspec(dllexport)
#else
#define FLR_EXPORT
#endif

#ifdef _WIN32
#define FLR_IMPORT __declspec( dllimport )
#else
#define FLR_IMPORT
#endif


class UART_Connector_wrapper {

public:
	UART_Connector_wrapper() {};
	~UART_Connector_wrapper() {};

	unsigned int mWidth;
	unsigned int mHeight;
    uint8_t isInitialized;
    int myPort;
    int readTimeout;
    uint8_t CommandChannel; 

    FLR_EXPORT FLR_RESULT Initialize(int32_t port_num, int32_t baud_rate);
    FLR_EXPORT FLR_RESULT bosonGetCameraSN(uint32_t *data);
    FLR_RESULT CLIENT_pkgBosonGetCameraSN(uint32_t *data);
    FLR_RESULT CLIENT_interface_readFrame(uint8_t *readData, uint32_t *readBytes);

    FLR_EXPORT void FSLP_send_to_camera(int32_t port_num, uint8_t channel_ID, uint32_t sendBytes, uint8_t *sendPayload);//, uint32_t *receiveBytes, uint8_t *receivePayload);
    // FLR_EXPORT void read_command(int32_t port_num, uint8_t channel_ID, uint32_t sendBytes, uint8_t *sendPayload, uint32_t *receiveBytes, uint8_t *receivePayload);
    FLR_EXPORT int32_t FSLP_read_frame(int32_t port_num,uint8_t channel_ID, uint16_t start_byte_ms,uint32_t *receiveBytes, uint8_t *receiveBuffer);
    FLR_EXPORT void FSLP_read_unframed(int32_t port_num, uint16_t start_byte_ms,uint32_t *receiveBytes, uint8_t *receiveBuffer);
    FLR_EXPORT int32_t FSLP_check_data_ready(int32_t port_num, uint8_t *channel_ID, uint16_t start_byte_ms, uint32_t *receiveBytes, const uint8_t **receiveBuffer);

    FLR_EXPORT FLR_RESULT bosonSetGainMode(const FLR_BOSON_GAINMODE_E gainMode);
    FLR_EXPORT FLR_RESULT bosonRunFFC();
    FLR_EXPORT FLR_RESULT roicGetFPATemp(uint16_t *data);
    FLR_RESULT CLIENT_pkgRoicGetFPATemp(uint16_t *data);
    FLR_RESULT CLIENT_pkgBosonRunFFC();
    FLR_RESULT CLIENT_pkgBosonSetGainMode(const FLR_BOSON_GAINMODE_E gainMode);
    FLR_EXPORT FLR_RESULT bosonlookupFPATempDegCx10(int16_t *data);
    FLR_RESULT CLIENT_pkgBosonlookupFPATempDegCx10(int16_t *data);

    FLR_RESULT sysctrlGetCameraFrameRate(uint32_t *frameRate);
    FLR_RESULT CLIENT_pkgSysctrlGetCameraFrameRate(uint32_t *frameRate);





 int32_t create_frame(uint8_t *frame_buf, uint8_t channel_ID, uint8_t *payload, uint32_t payload_len);

void initialize_channels();
int32_t get_channel(uint8_t channel_ID, CHANNEL_T **return_channel);
void get_unframed(CHANNEL_T **return_channel);
void add_byte(uint8_t inbyte,CHANNEL_T *channel_ptr);
int32_t get_byte(uint8_t *outbyte,CHANNEL_T *channel_ptr);

int write_frame(int32_t port_num,uint8_t *frame_buf, uint32_t len);
FLR_RESULT CLIENT_interface_writeFrame(uint8_t *writeData, uint32_t writeBytes);
void SendToCamera( uint8_t channelID,  uint32_t sendBytes, uint8_t *sendData);

FLR_RESULT CLIENT_dispatcher_Tx(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes);
FLR_RESULT CLIENT_dispatcher_Rx(uint32_t *seqNum, uint32_t *fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes);
FLR_RESULT CLIENT_dispatcher(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes);
FLR_RESULT CheckReadyDataCommandId(uint32_t receiveBytes, const uint8_t *receiveData, uint32_t *commandId);

  
    //FLR_IMPORT uint8_t FSLP_open_port(int32_t port_num, int32_t baud_rate);

    FLR_EXPORT void ReadTimeoutSet(unsigned int timeout);

    FLR_EXPORT void ReadFrame( uint8_t channelID, uint32_t *receiveBytes, uint8_t *receiveData);
    FLR_EXPORT void ReadUnframed(uint32_t *receiveBytes, uint8_t *receiveData);

    FLR_EXPORT void Close(void);
    FLR_EXPORT int32_t CheckDataReady(uint8_t *channel_ID, uint32_t *receiveBytes, const uint8_t **receiveData);

};
#endif 








#include <stdbool.h>  
#include "UART_Connector_wrapper.h"
#include "serialPortAdapter.h"
#include "serialPortAdapter.c"
#include "serial.h"
#include "serial.c"
#include "EnumTypes.h"
#include "Serializer_Struct.h"
#include "FunctionCodes.h"
#include "Serializer_BuiltIn.h"
#include "Serializer_BuiltIn.c"
#include "timeoutLogic.h"
#include "timeoutLogic.c"
#include "flirCRC.h"
#include "flirCRC.c"





#define DO_ERROR_TRACE(x...)    printf(x)
#define DO_DEBUG_TRACE(x...)    //printf(x)
#define DO_VERBOSE_TRACE(x...)  //printf(x)









#define ESCAPE_BYTE        0x9E
#define START_FRAME_BYTE   0x8E
#define END_FRAME_BYTE     0xAE
#define ESCAPED_ESCAPE_BYTE        0x91
#define ESCAPED_START_FRAME_BYTE   0x81
#define ESCAPED_END_FRAME_BYTE     0xA1

#define NUM_FRAMING_BYTES 3
#define FRAME_START_IDX 1
#define CRC_START_IDX 0

#define FRAME_BUF_SIZ      2048

#define FRAME_TIMEOUT_SEC   2.000
#define POLL_TIMEOUT_SEC   0.025
#define BYTE_TIMEOUT_SEC    0.005

static uint8_t in_frame_buf[FRAME_BUF_SIZ];
static uint8_t out_frame_buf[FRAME_BUF_SIZ];

enum frame_state_e {
    UNFRAMED = 0,
    CORRECT_FRAME = 1,
    OTHER_FRAME = 2,
};
typedef enum frame_state_e frame_state;

static uint8_t other_frame_ID;

static int16_t read_single_byte(int32_t port_num)
{
    return FSLP_read_byte_with_timeout(port_num, BYTE_TIMEOUT_SEC);
}

// defined by serialPortAdapter
// static inline int write_byte(int32_t port_num,int outc)

static void extract_payload(uint8_t* raw_payload_buf, uint32_t raw_payload_len, uint8_t* payload_buf, uint32_t* payload_len)
{
    uint32_t i;

    for(*payload_len=0, i=0; i < raw_payload_len; i++)
    {
        // if (raw_payload_buf[i] == ESCAPE_BYTE) i++;
        payload_buf[(*payload_len)++] = raw_payload_buf[i];
    }
}


void UART_Connector_wrapper::FSLP_send_to_camera(int32_t port_num, uint8_t channel_ID, uint32_t sendBytes, uint8_t *sendPayload)
{
    int success=0;//, i=0;

    int32_t out_len;//, in_payload_len, out_payload_len;
    out_len = create_frame(out_frame_buf,channel_ID, sendPayload, sendBytes);

    DO_DEBUG_TRACE("sendBytes = %u, out_len = %d\n",sendBytes, out_len);

    success = write_frame(port_num,out_frame_buf, out_len);

    DO_DEBUG_TRACE("frame send status %d.\n",success);

}


 int32_t UART_Connector_wrapper::create_frame(uint8_t *frame_buf, uint8_t channel_ID, uint8_t *payload, uint32_t payload_len)
{
    uint32_t i;
    int32_t out_len = 0;
    uint16_t crc_out = FLIR_CRC_INITIAL_VALUE;
    frame_buf[out_len++] = (uint8_t) START_FRAME_BYTE;
    frame_buf[out_len++] = (uint8_t) channel_ID;
    crc_out = ByteCRC16(channel_ID, (int) crc_out);
    // frame_buf[out_len++] = (uint8_t) 0x00;  // Removed Frame status byte[0]
    // crc_out = ByteCRC16(0x00, (int) crc_out);
    // frame_buf[out_len++] = (uint8_t) 0x00;  // Removed Frame status byte[1]
    // crc_out = ByteCRC16(0x00, (int) crc_out);

    if ((payload_len + 5) > FRAME_BUF_SIZ) // start + channel_ID + payload + crc_bytes[2] + end
        return (-1);

    for(i=0; i < payload_len; i++)
    {
        if ((payload[i] == START_FRAME_BYTE) || (payload[i] == END_FRAME_BYTE) || (payload[i] == ESCAPE_BYTE)){
            frame_buf[out_len++] = ESCAPE_BYTE;
            uint8_t c = payload[i];
            crc_out = ByteCRC16((int) c, (int) crc_out);
            switch (c)
            {
                case END_FRAME_BYTE:
                    c = ESCAPED_END_FRAME_BYTE;
                    break;
                case START_FRAME_BYTE:
                    c = ESCAPED_START_FRAME_BYTE;
                    break;
                case ESCAPE_BYTE:
                    c = ESCAPED_ESCAPE_BYTE;
                    break;
                default:
                    break;
            }
            frame_buf[out_len++] = c;

        }
        else
        {
            frame_buf[out_len++] = payload[i];
            crc_out = ByteCRC16((int) payload[i], (int) crc_out);
        }
        if ((out_len + 3) > FRAME_BUF_SIZ)
            return (-2);
    }

    // crc_out = calcFlirCRC16Bytes(out_len, frame_buf);
    uint8_t crcbyte = ((crc_out >> 8) & 0xFF);
    if ((crcbyte == START_FRAME_BYTE) || (crcbyte == END_FRAME_BYTE) || (crcbyte == ESCAPE_BYTE)) {
        frame_buf[out_len++] = ESCAPE_BYTE;
        uint8_t c = crcbyte;
        switch (c)
        {
            case END_FRAME_BYTE:
                c = ESCAPED_END_FRAME_BYTE;
                break;
            case START_FRAME_BYTE:
                c = ESCAPED_START_FRAME_BYTE;
                break;
            case ESCAPE_BYTE:
                c = ESCAPED_ESCAPE_BYTE;
                break;
            default:
                break;
        }
        frame_buf[out_len++] = c;
    }
    else
    {
        frame_buf[out_len++] = crcbyte;
    }

    crcbyte = (crc_out & 0xFF);
    if ((crcbyte == START_FRAME_BYTE) || (crcbyte == END_FRAME_BYTE) || (crcbyte == ESCAPE_BYTE)) {
        frame_buf[out_len++] = ESCAPE_BYTE;
        uint8_t c = crcbyte;
        switch (c)
        {
            case END_FRAME_BYTE:
                c = ESCAPED_END_FRAME_BYTE;
                break;
            case START_FRAME_BYTE:
                c = ESCAPED_START_FRAME_BYTE;
                break;
            case ESCAPE_BYTE:
                c = ESCAPED_ESCAPE_BYTE;
                break;
            default:
                break;
        }
        frame_buf[out_len++] = c;
    }
    else
    {
        frame_buf[out_len++] = crcbyte;
    }
    frame_buf[out_len++] = END_FRAME_BYTE;

    return(out_len);
}

int UART_Connector_wrapper::write_frame(int32_t port_num,uint8_t *frame_buf, uint32_t len)
{
    int i;

    DO_DEBUG_TRACE("Writing Frame: ");
    for(i=0; i<len; i++){
        DO_DEBUG_TRACE(" %02X",frame_buf[i]);
    }
    DO_DEBUG_TRACE("...\n");

    if (FSLP_write_buffer(port_num, frame_buf, len) == len){

    DO_DEBUG_TRACE("Success.");

        return 0;
    } else {

    DO_DEBUG_TRACE("Failed.");

        return -1;
    }
}

FLR_RESULT UART_Connector_wrapper::bosonlookupFPATempDegCx10(int16_t *data){
    FLR_RESULT returncode = CLIENT_pkgBosonlookupFPATempDegCx10(data);
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of lookupFPATempDegCx10()

// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgBosonlookupFPATempDegCx10(int16_t *data) {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 0;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 2;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;
    uint32_t commandCount = 0;

    
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, BOSON_LOOKUPFPATEMPDEGCX10, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    // read data from receiveData buffer
    { //Block to allow reuse of outVal
        if(inPtr >= (receiveData+receiveBytes)){
            return R_SDK_PKG_BUFFER_OVERFLOW;
        }
        int16_t outVal;
        byteToINT_16( (const uint8_t *) inPtr, &outVal);
        *data = (int16_t)outVal;
        inPtr+=2;
    }// end of data handling
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgBosonlookupFPATempDegCx10()


void UART_Connector_wrapper::SendToCamera( uint8_t channelID,  uint32_t sendBytes, uint8_t *sendData)
{
	if (!isInitialized) return;


	FSLP_send_to_camera( (int32_t) myPort, channelID, sendBytes, sendData);
}


FLR_RESULT UART_Connector_wrapper::Initialize(int32_t port_num, int32_t baud_rate)
{

            isInitialized = 0;
            myPort = 0;
            readTimeout = 1000;
			CommandChannel = 0x00; 

	if (isInitialized) return R_UART_PORT_ALREADY_OPEN;

	myPort = port_num;

	printf("Intializing your camera Ms. Emily... \n");
    //Initialize(port_num, baud_rate);
	
	
    if (FSLP_open_port(myPort, baud_rate)) return R_UART_PORT_FAILURE;

	isInitialized = 1;
	return FLR_COMM_OK; // 0 == success.
}

void UART_Connector_wrapper::Close()
{
	FLR_IMPORT void FSLP_close_port(int32_t port_num);
	FSLP_close_port((int32_t) myPort);
	isInitialized = 0;
	myPort = 0;
}


FLR_RESULT UART_Connector_wrapper::CLIENT_interface_writeFrame(uint8_t *writeData, uint32_t writeBytes)
{
    if(writeData == NULL)
        return FLR_BAD_ARG_POINTER_ERROR;

#ifdef USE_I2C_SLAVE_CP
    if(I2C_writeFrame(writeData, writeBytes) != FLR_OK)
        return FLR_COMM_ERROR_WRITING_COMM;
#else
    SendToCamera(CommandChannel, writeBytes, writeData);
#endif
    return FLR_OK;
}


FLR_RESULT UART_Connector_wrapper::CLIENT_interface_readFrame(uint8_t *readData, uint32_t *readBytes)
{


    if(readData == NULL || readBytes == NULL)
        return FLR_BAD_ARG_POINTER_ERROR;

#ifdef USE_I2C_SLAVE_CP
    if(I2C_readFrame(readData, readBytes) != FLR_OK)
        return FLR_COMM_ERROR_READING_COMM;
#else
    ReadFrame(CommandChannel, readBytes, readData);
#endif
    return FLR_OK;
}



void UART_Connector_wrapper::add_byte(uint8_t inbyte,CHANNEL_T *channel_ptr){
	uint32_t index;
	uint16_t start = (channel_ptr->start);
	if (channel_ptr->len != CHANNEL_BUF_SIZ){
		index = start + (channel_ptr->len);
		(channel_ptr->buff)[index] = inbyte;
		(channel_ptr->len)++;
	} else {
		(channel_ptr->buff)[start] = inbyte;
		(channel_ptr->start) = (start + 1)%CHANNEL_BUF_SIZ;
	}
}

int32_t UART_Connector_wrapper::get_byte(uint8_t *outbyte,CHANNEL_T *channel_ptr){
	//return remaining length if success, -1 if channel already empty
	if (channel_ptr->len == 0) {
		return -1;
	} else {
		*outbyte = (channel_ptr->buff)[(channel_ptr->start)];
		(channel_ptr->start)++;
		return --(channel_ptr->len);
	}
}


FLR_RESULT UART_Connector_wrapper::bosonGetCameraSN(uint32_t *data){
    FLR_RESULT returncode = CLIENT_pkgBosonGetCameraSN(data);
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of GetCameraSN()


FLR_RESULT UART_Connector_wrapper::CLIENT_dispatcher(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes)
{    uint32_t returnSequence;
    uint32_t cmdID;
    FLR_RESULT res = CLIENT_dispatcher_Tx(seqNum, fnID, sendData, sendBytes, receiveData, receiveBytes);
    if (res)
        return res;
    res = CLIENT_dispatcher_Rx(&returnSequence, &cmdID, sendData, sendBytes, receiveData, receiveBytes);
    if (res)
        return res;
    
    if (returnSequence ^ seqNum)
        return R_SDK_DSPCH_SEQUENCE_MISMATCH;
    
    if (cmdID ^ (uint32_t) fnID)
        return R_SDK_DSPCH_ID_MISMATCH;
    
    return R_SUCCESS;
}

void UART_Connector_wrapper::ReadFrame(uint8_t channelID, uint32_t *receiveBytes, uint8_t *receiveData)
{
	//FLR_IMPORT void FSLP_read_frame(int32_t port_num,uint8_t channel_ID, uint16_t start_byte_ms,uint32_t *receiveBytes, uint8_t *receiveBuffer);
	FSLP_read_frame((int32_t) myPort, channelID, readTimeout, receiveBytes, receiveData);
}



// End Module: agc
// Begin Module: boson
// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgBosonGetCameraSN(uint32_t *data) {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 0;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 4;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;

	uint32_t commandCount = 0;
    
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, BOSON_GETCAMERASN, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    // read data from receiveData buffer
    { //Block to allow reuse of outVal
        if(inPtr >= (receiveData+receiveBytes)){
            return R_SDK_PKG_BUFFER_OVERFLOW;
        }
        uint32_t outVal;
        byteToUINT_32( (const uint8_t *) inPtr, &outVal);
        *data = (uint32_t)outVal;
        inPtr+=4;
    }// end of data handling
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgBosonGetCameraSN()



// Asynchronous (MultiService compatible) transmit part
FLR_RESULT UART_Connector_wrapper::CLIENT_dispatcher_Tx(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes) {
    
    uint32_t i;
    
    // Allocated buffer with extra space for payload header
    uint8_t sendPayload[530];
    uint8_t *pyldPtr = (uint8_t *)sendPayload;
    
    // Write sequence number to first 4 bytes
    UINT_32ToByte(seqNum, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Write function ID to second 4 bytes
    UINT_32ToByte((const uint32_t) fnID, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Write 0xFFFFFFFF to third 4 bytes
    UINT_32ToByte(0xFFFFFFFF, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Copy sendData to payload buffer
    uint8_t *dataPtr = (uint8_t *)sendData;
    for(i = 0;i<sendBytes;i++) {
        *pyldPtr++ = *dataPtr++;
    }
    
    if(CLIENT_interface_writeFrame(sendPayload, sendBytes + 12) != FLR_OK)
        return FLR_COMM_ERROR_WRITING_COMM;
    
    return R_SUCCESS;
}
// Asynchronous (MultiService compatible) receive part
FLR_RESULT UART_Connector_wrapper::CLIENT_dispatcher_Rx(uint32_t *seqNum, uint32_t *fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes) {
    
    uint32_t i;
    
    // Allocated buffer with extra space for return data
    uint8_t receivePayload[530];
    uint8_t *inPtr = (uint8_t *)receivePayload;
    
    *receiveBytes+=12;
    if(CLIENT_interface_readFrame(receivePayload, receiveBytes) != FLR_OK)
        return FLR_COMM_ERROR_READING_COMM;
    
    if (*receiveBytes < 12) {
        if(CLIENT_interface_readFrame(receivePayload, receiveBytes) != FLR_OK)
            return FLR_COMM_ERROR_READING_COMM;
    }
    
    if (*receiveBytes < 12)
        return FLR_COMM_ERROR_READING_COMM;
    
    // Evaluate sequence bytes as UINT_32
    uint32_t returnSequence;
    byteToUINT_32( (const uint8_t *) inPtr, &returnSequence);
    inPtr += 4;
    
    // Ensure that received sequence matches sent sequence
    if(seqNum){
        *seqNum = returnSequence;
    }
    
    // Evaluate CMD ID bytes as UINT_32 
    uint32_t cmdID;
    byteToUINT_32( (const uint8_t *) inPtr, &cmdID);
    inPtr += 4;
    
    // Ensure that received CMD ID matches sent CMD ID
    if(fnID){
        *fnID = cmdID;
    }
    
    // Evaluate Payload Status bytes as UINT_32
    uint32_t pyldStatus;
    byteToUINT_32( (const uint8_t *) inPtr, &pyldStatus);
    inPtr += 4;
    
    const FLR_RESULT returncode = (FLR_RESULT) pyldStatus;
    // Check for any errorcode
    if(returncode != R_SUCCESS){
        return returncode;
    }
    
    // Now have Good Tx, Good Sequence, Good CMD ID, and Good Status.
    // inPtr at Data block, fill receiveData buffer with outPtr
    uint8_t *outPtr = (uint8_t *)receiveData;
    // decrement receiveBytes by 12 (len of header bytes)
    *receiveBytes-=12;
    
    uint32_t localvar = *receiveBytes; //shouldn't have to do this, but it works.
    for(i=0;i<localvar;i++) {
        *outPtr++ = *inPtr++;
    }
    
    return R_SUCCESS;
} // End CLIENT_dispatcher()


int32_t UART_Connector_wrapper::get_channel(uint8_t channel_ID, CHANNEL_T **return_channel){
	int i;
	for (i = 1; i < NUM_CHANNELS; i++){
		if (channel_ID == channel_list[i].channel) {
			*return_channel = &(channel_list[i]);
			return 0;
		}
	}
	*return_channel = 0;
	return -1;
}

void UART_Connector_wrapper::get_unframed(CHANNEL_T **return_channel){
	*return_channel = &(channel_list[0]);
	return;
}


// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgBosonSetGainMode(const FLR_BOSON_GAINMODE_E gainMode) {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 4;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 1;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;
    
    //write gainMode to sendData buffer
    { //Block to allow reuse of inVal
        if(outPtr >= (sendData+sendBytes)){
            return R_SDK_PKG_BUFFER_OVERFLOW;
        }
        const FLR_BOSON_GAINMODE_E inVal = gainMode;
        INT_32ToByte(inVal, (const uint8_t *) outPtr);
        outPtr += 4;
    }
    uint32_t commandCount = 0;
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, BOSON_SETGAINMODE, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgBosonSetGainMode()


FLR_RESULT UART_Connector_wrapper::bosonSetGainMode(const FLR_BOSON_GAINMODE_E gainMode){
    FLR_RESULT returncode = CLIENT_pkgBosonSetGainMode(gainMode);
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of SetGainMode()




// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgBosonRunFFC() {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 0;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 1;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;
    uint32_t commandCount = 0;
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, BOSON_RUNFFC, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgBosonRunFFC()

FLR_RESULT UART_Connector_wrapper::bosonRunFFC(){
    FLR_RESULT returncode = CLIENT_pkgBosonRunFFC();
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of RunFFC()


// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgRoicGetFPATemp(uint16_t *data) {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 0;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 2;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;
    uint32_t commandCount = 0;
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, ROIC_GETFPATEMP, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    // read data from receiveData buffer
    { //Block to allow reuse of outVal
        if(inPtr >= (receiveData+receiveBytes)){
            return R_SDK_PKG_BUFFER_OVERFLOW;
        }
        uint16_t outVal;
        byteToUINT_16( (const uint8_t *) inPtr, &outVal);
        *data = (uint16_t)outVal;
        inPtr+=2;
    }// end of data handling
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgRoicGetFPATemp()


FLR_RESULT UART_Connector_wrapper::roicGetFPATemp(uint16_t *data){
    FLR_RESULT returncode = CLIENT_pkgRoicGetFPATemp(data);
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of GetFPATemp()

FLR_RESULT UART_Connector_wrapper::sysctrlGetCameraFrameRate(uint32_t *frameRate){
    FLR_RESULT returncode = CLIENT_pkgSysctrlGetCameraFrameRate(frameRate);
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    return R_SUCCESS;
} // End of GetCameraFrameRate()

// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT UART_Connector_wrapper::CLIENT_pkgSysctrlGetCameraFrameRate(uint32_t *frameRate) {
    // Allocate buffers with space for marshalled data
    uint32_t sendBytes = 0;
    uint8_t sendData[sendBytes];
    uint32_t receiveBytes = 4;
    uint8_t receiveData[receiveBytes];
    uint8_t *outPtr = (uint8_t *)sendData;
    uint32_t commandCount = 0;
    
    FLR_RESULT returncode = CLIENT_dispatcher(commandCount++, SYSCTRL_GETCAMERAFRAMERATE, sendData, sendBytes, receiveData, &receiveBytes);
    
    // Check for any errorcode
    if((uint32_t) returncode){
        return returncode;
    }
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    // read frameRate from receiveData buffer
    { //Block to allow reuse of outVal
        if(inPtr >= (receiveData+receiveBytes)){
            return R_SDK_PKG_BUFFER_OVERFLOW;
        }
        uint32_t outVal;
        byteToUINT_32( (const uint8_t *) inPtr, &outVal);
        *frameRate = (uint32_t)outVal;
        inPtr+=4;
    }// end of frameRate handling
    
    return R_SUCCESS;
    
}// End of CLIENT_pkgSysctrlGetCameraFrameRate()



int32_t UART_Connector_wrapper::FSLP_read_frame(int32_t port_num,uint8_t channel_ID, uint16_t start_byte_ms,uint32_t *receiveBytes, uint8_t *receiveBuffer)
{
    int32_t byte_idx = 0;
    int32_t tempval = 0;
    frame_state am_in_frame = UNFRAMED;
    uint8_t found_complete_frame=0, in_escape=0;
    uint8_t c;
    // uint8_t failedcrc = 0;
    uint16_t calc_crc;

    if (get_channel(channel_ID, &chan_ptr))
        DO_ERROR_TRACE("Channel not found for channel_ID [0x%02x]\n", channel_ID);
    get_unframed(&unframed_ptr);

    struct timespec start_t,current_t;
    double elapsed_sec;
    clock_gettime(CLOCK_MONOTONIC, &start_t);
    double timeout = (double)start_byte_ms/1000.0;
    *receiveBytes =  0;

    if (chan_ptr && (chan_ptr->len > 0)) {

        int i,j;
        DO_DEBUG_TRACE("POLL buffer start = %d, len = %d \n", chan_ptr->start, chan_ptr->len);
        DO_DEBUG_TRACE("current buffer[20] : ");
        for (j = 0; j < 20; j++){
            i = ( (chan_ptr->start + j ) % CHANNEL_BUF_SIZ);
            DO_DEBUG_TRACE(" %02X", chan_ptr->buff[i]);
        }
        DO_DEBUG_TRACE("\n");

        while (chan_ptr->len > 0) {

            // if (i==poll_idx_lo) DO_DEBUG_TRACE("Entered PROCESS_BUFF_%02d\n",chan_ptr->channel);

            int32_t ret = get_byte(&c, chan_ptr);
            if (ret < 0){
                DO_ERROR_TRACE("Failed to get_byte\n");
                DO_VERBOSE_TRACE("Breaking the pre-received path\n");
                break;
            } else {
                DO_VERBOSE_TRACE("Next byte: [0x%02x]\n", c);
            }

            if (c == START_FRAME_BYTE)
            {
                DO_DEBUG_TRACE("     POLL_RX_BUF StartByte \n ");

                timeout = FRAME_TIMEOUT_SEC;
                DO_VERBOSE_TRACE("Setting timeout to FRAME_TIMEOUT_SEC\n");
                byte_idx = 0;
                am_in_frame = CORRECT_FRAME;
                DO_VERBOSE_TRACE("Am in CORRECT_FRAME\n");
                DO_VERBOSE_TRACE("Continuing (to get_byte stage)\n");
                continue;
            }
            if (am_in_frame) {

                if (c == ESCAPE_BYTE)
                {
                    in_escape = 1;
                    int32_t ret = get_byte(&c,chan_ptr);
                    if (ret < 0){
                        DO_ERROR_TRACE("Failed to get_byte\n");
                        DO_VERBOSE_TRACE("Breaking the pre-received path\n");
                        break;
                    }
                    switch (c){
                        case ESCAPED_END_FRAME_BYTE:
                            c = END_FRAME_BYTE;
                            break;
                        case ESCAPED_START_FRAME_BYTE:
                            c = START_FRAME_BYTE;
                            break;
                        case ESCAPED_ESCAPE_BYTE:
                            c = ESCAPE_BYTE;
                            break;
                        default:
                            break;
                    }

                    in_frame_buf[byte_idx++] = c;
                    DO_DEBUG_TRACE(" %02X",c);
                    DO_VERBOSE_TRACE("in_frame_buf[%d] [%02X]",(byte_idx - 1), c);
                    DO_VERBOSE_TRACE("Continuing (to get_byte stage)\n");
                    continue;
                }

                if (c == END_FRAME_BYTE)
                {
                    DO_VERBOSE_TRACE("Am in UNFRAMED\n");
                    am_in_frame = UNFRAMED;
                    /*
                     * Check CRC: if good end loop.  If bad, start looking for start byte.
                     * Check Frame Status:  If zero, end loop.  If non-zero, start looking for start byte.
                     */

                     /*
                      * byte_idx - 3 = total frame length - channel_ID byte - crc_bytes[2]
                      * &(frame_buf[1]) is first byte after channel_ID byte
                      */
                    calc_crc = calcFlirCRC16Bytes((byte_idx - 2), &(in_frame_buf[CRC_START_IDX]));

                    if ( (((calc_crc >> 8) &0xFF) != in_frame_buf[byte_idx - 2]) || ((calc_crc &0xFF) != in_frame_buf[byte_idx - 1]) )
                    {
                        DO_ERROR_TRACE("\nFailed packet integrity check (calc) %02X%02X !=  (recd) %02X%02X\n",((calc_crc >> 8) &0xFF),(calc_crc&0xFF),in_frame_buf[byte_idx - 2],in_frame_buf[byte_idx - 1]);
                        DO_ERROR_TRACE("RAW Receive Packet: ");
                        for (i = 0; i < byte_idx; i++){
                            DO_ERROR_TRACE(" %02X",in_frame_buf[i]);
                        }
                        DO_ERROR_TRACE("\n");
                        // DO_ERROR_TRACE("     Failed time %f:\n",elapsed_sec);
                        // failedcrc = 1;
                        byte_idx = 0;

                        timeout = POLL_TIMEOUT_SEC;
                        DO_VERBOSE_TRACE("Setting timeout to POLL_TIMEOUT_SEC\n");
                        DO_VERBOSE_TRACE("Continuing (to get_byte stage)\n");
                        continue;
                    } else {
                        DO_DEBUG_TRACE("\n");
                    }

                    found_complete_frame = 1;
                    DO_VERBOSE_TRACE("Found a complete frame - breaking the pre-received path\n");
                    break;
                }


                in_frame_buf[byte_idx++] = c;
                DO_VERBOSE_TRACE("in_frame_buf[%d] [0x%02x]\n", (byte_idx - 1), c);
                DO_DEBUG_TRACE(" %02X",c);

                DO_VERBOSE_TRACE("Continuing (to get_byte stage)\n");
                continue;
            }
        }
    }

    if (found_complete_frame) {

        DO_DEBUG_TRACE("Found Complete Frame in channel_buf_%02d\n",chan_ptr->channel);
        DO_DEBUG_TRACE("current buffer start = %d, len = %d \n",chan_ptr->start,chan_ptr->len);

        extract_payload(&(in_frame_buf[FRAME_START_IDX]), (byte_idx - NUM_FRAMING_BYTES), receiveBuffer, receiveBytes);
        return 0;
    }

    DO_DEBUG_TRACE("Entering serial loop...\n");

    clock_gettime(CLOCK_MONOTONIC, &start_t);
    while(1)
    {
        // if (failedcrc){
            // DO_ERROR_TRACE("AFTER FAIL: %f\n",elapsed_sec);
        // }
        if (byte_idx == FRAME_BUF_SIZ)
        {
            // Buffer overrun
            DO_ERROR_TRACE("Buffer overrun - exitting\n");
            return -1;
        }

        clock_gettime(CLOCK_MONOTONIC, &current_t);
        elapsed_sec = diff_timespec(&current_t, &start_t);

        // The whole thing has been taking too long
        if (elapsed_sec >= timeout){
            DO_DEBUG_TRACE("Frame reading has been taking too long [%f]\n", elapsed_sec);
            DO_ERROR_TRACE("Timeout: %f s\n", elapsed_sec);

            if (am_in_frame == CORRECT_FRAME) {
                int i;
                DO_VERBOSE_TRACE("Timeout while in CORRECT_FRAME: %f s\n", elapsed_sec);

                DO_ERROR_TRACE("Received frame fragment - len:[%d] : ", byte_idx);
                for (i = 0; i < byte_idx; i++){
                    DO_ERROR_TRACE(" %02X", in_frame_buf[i]);
                }
                DO_ERROR_TRACE("\n");
            } else {
                DO_VERBOSE_TRACE("Timeout while in %s: %f s\n", (am_in_frame) ? "OTHER_FRAME" : "UNFRAMED", elapsed_sec);
            }

            *receiveBytes =  0;
            am_in_frame = UNFRAMED;
            return -1;
        }

        if (in_escape && (am_in_frame == CORRECT_FRAME)) {
            //carryover escape character from buffer
            tempval = ESCAPE_BYTE;
        } else {
            //normal operations
            tempval = read_single_byte(port_num);
        }

        if (tempval<0) {
            clock_gettime(CLOCK_MONOTONIC, &current_t);
            elapsed_sec = diff_timespec(&current_t, &start_t);
            DO_VERBOSE_TRACE("Timeout in read_byte; current frame read time %f\n", elapsed_sec);
            DO_VERBOSE_TRACE("Continuing...\n");
            continue; //byte read timeout.
        } else {
            c = (uint8_t) tempval;
            DO_VERBOSE_TRACE("Next byte: [0x%02x]\n", c);
        }

        if ((c & 0xFF) == (START_FRAME_BYTE & 0xFF))
        {
            DO_DEBUG_TRACE("     StartByte time %f:\n     ",elapsed_sec);

            DO_VERBOSE_TRACE("Setting timeout to FRAME_TIMEOUT_SEC=%f\n",FRAME_TIMEOUT_SEC);
            timeout = FRAME_TIMEOUT_SEC;
            DO_VERBOSE_TRACE("Resetting start time from %f to %f\n",start_t,current_t);
            start_t = current_t;
            
            byte_idx = 0;
            do {
                clock_gettime(CLOCK_MONOTONIC, &current_t);
                elapsed_sec = diff_timespec(&current_t, &start_t);
                if (elapsed_sec > timeout) {
                    DO_DEBUG_TRACE("Abort waiting for channel ID at %f of %f seconds\n",elapsed_sec,timeout);
                    break;
                }
                tempval = read_single_byte(port_num);
                if ( (tempval & 0xFF) == (START_FRAME_BYTE & 0xFF) )
                {
                    DO_DEBUG_TRACE("Weird Byte 0x%X\n",tempval);
                    tempval = -1;
                }
            } while (tempval < 0);
            if (tempval < 0) {
                DO_VERBOSE_TRACE("Timeout in read_byte\n");
                DO_VERBOSE_TRACE("Continuing...\n");
                continue;
            }

            c = (uint8_t) tempval & 0xFF;
            uint8_t needs_escape = 0;
            if (c == ESCAPE_BYTE){
                DO_DEBUG_TRACE("ESCAPE char before channel byte\n");
                needs_escape = 1;
                do {
                    clock_gettime(CLOCK_MONOTONIC, &current_t);
                    elapsed_sec = diff_timespec(&current_t, &start_t);
                    if (elapsed_sec > timeout) {
                        DO_DEBUG_TRACE("Abort waiting for escaped channel ID at %f of %f seconds\n",elapsed_sec,timeout);
                        break;
                    }
                    tempval = read_single_byte(port_num);
                    if ( (tempval & 0xFF) == (START_FRAME_BYTE & 0xFF) )
                        {
                            DO_DEBUG_TRACE("Escaped channel Byte 0x%X\n",tempval);
                            tempval = -1;
                        }
                } while (tempval < 0);
                if (tempval < 0) {
                    DO_VERBOSE_TRACE("Timeout in read_byte\n");
                    DO_VERBOSE_TRACE("Continuing...\n");
                    continue;
                }
                c = (uint8_t) tempval & 0xFF;
            }

            if (c == channel_ID) {
                //found correct frame
                DO_VERBOSE_TRACE("Am in CORRECT_FRAME\n");
                am_in_frame = CORRECT_FRAME;
                in_frame_buf[byte_idx++] = c;
                DO_VERBOSE_TRACE("in_frame_buf[%d] [0x%02x]\n", (byte_idx - 1), c);
            } else {
                DO_VERBOSE_TRACE("Am in OTHER_FRAME (0x%02x)\n", c);
                am_in_frame = OTHER_FRAME;
                other_frame_ID = c;
                get_channel(other_frame_ID, &chan_ptr);
                if (!chan_ptr){
                    DO_ERROR_TRACE("Incoming frame's channel (0x%02x) not found\n", other_frame_ID);
                    chan_ptr = unframed_ptr;
                    am_in_frame = UNFRAMED;
                    DO_VERBOSE_TRACE("Am in UNFRAMED\n");
                    continue;
                }
                add_byte(START_FRAME_BYTE,chan_ptr);
                if(needs_escape)
                    add_byte(ESCAPE_BYTE,chan_ptr);
                add_byte(c,chan_ptr);
                DO_VERBOSE_TRACE("Add START + (ESCAPE) + [0x%02x]\n", c);
            }

            DO_VERBOSE_TRACE("Continuing...\n");
            continue;
        }

        if (am_in_frame == CORRECT_FRAME) {
            if ((c & 0xFF) == (ESCAPE_BYTE & 0xFF))
            {
                in_escape = 1;

                do {
                    clock_gettime(CLOCK_MONOTONIC, &current_t);
                    elapsed_sec = diff_timespec(&current_t, &start_t);
                    if (elapsed_sec>timeout) {
                        DO_DEBUG_TRACE("Abort waiting for escaped normal byte at %f of %f seconds\n",elapsed_sec,timeout);
                        break;
                    }
                    tempval = read_single_byte(port_num);
                } while (tempval < 0);
                if (tempval < 0) {
                    DO_VERBOSE_TRACE("Timeout in read_byte\n");
                    DO_VERBOSE_TRACE("Continuing...\n");
                    continue;
                }

                c = (uint8_t) tempval;
                switch (c){
                    case ESCAPED_END_FRAME_BYTE:
                        c = END_FRAME_BYTE;
                        break;
                    case ESCAPED_START_FRAME_BYTE:
                        c = START_FRAME_BYTE;
                        break;
                    case ESCAPED_ESCAPE_BYTE:
                        c = ESCAPE_BYTE;
                        break;
                    default:
                        break;
                }
                in_frame_buf[byte_idx++] = c;
                in_escape = 0;
                DO_VERBOSE_TRACE("in_frame_buf[%d] [0x%02x]\n", (byte_idx - 1), c);
                DO_DEBUG_TRACE(" %02X",c);

                DO_VERBOSE_TRACE("Continuing...\n");
                continue;
            }

            if ((c & 0xFF) == (END_FRAME_BYTE & 0xFF))
            {
                // DO_ERROR_TRACE("\n     EndByte time %f:\n",elapsed_sec);
                // frame_buf[byte_idx++] = c;

                /*
                 * Check CRC: if good end loop.  If bad, start looking for start byte.
                 * Check Frame Status:  If zero, end loop.  If non-zero, start looking for start byte.
                 */

                 /*
                  * byte_idx - 3 = total frame length - channel_ID byte - crc_bytes[2]
                  * &(frame_buf[1]) is first byte after channel_ID byte
                  */

                DO_VERBOSE_TRACE("End frame byte\n");
                am_in_frame = UNFRAMED;
                DO_VERBOSE_TRACE("Am in UNFRAMED\n");

                calc_crc = calcFlirCRC16Bytes((byte_idx - 2), &(in_frame_buf[CRC_START_IDX]));

                if ( (((calc_crc >> 8) &0xFF) != in_frame_buf[byte_idx - 2]) || ((calc_crc &0xFF) != in_frame_buf[byte_idx - 1]) )
                {
                    int i;
                    DO_ERROR_TRACE("Failed POLL packet integrity check (calc) %02X%02X !=  (recd) %02X%02X\n",((calc_crc >> 8) &0xFF),(calc_crc&0xFF),in_frame_buf[byte_idx - 2],in_frame_buf[byte_idx - 1]);
                    DO_ERROR_TRACE("RAW Receive Packet: ");
                    for (i = 0; i < byte_idx; i++){
                        DO_ERROR_TRACE(" %02X", in_frame_buf[i]);
                    }
                    DO_ERROR_TRACE("\n");
                    // DO_ERROR_TRACE("     Failed time %f:\n",elapsed_sec);
                    // failedcrc = 1;
                    byte_idx = 0;
                    clock_gettime(CLOCK_MONOTONIC, &start_t);
                    timeout= (double)start_byte_ms/1000.0;
                    DO_VERBOSE_TRACE("Setting timeout to [%f]\n", timeout);
                    DO_VERBOSE_TRACE("Continuing...\n");
                    continue;
                } else {
                    DO_DEBUG_TRACE("\n");
                }

                DO_VERBOSE_TRACE("Found complete frame\n");
                found_complete_frame = 1;
                DO_VERBOSE_TRACE("Breaking the serial loop path\n");
                break;
            }

            in_frame_buf[byte_idx++] = c;
            DO_VERBOSE_TRACE("in_frame_buf[%d] [0x%02x]\n", (byte_idx - 1), c);

            DO_DEBUG_TRACE(" %02X",c);

        } else if (am_in_frame == OTHER_FRAME) {
            if ((c & 0xFF) == (ESCAPE_BYTE & 0xFF)) {
                in_escape = 1;
                add_byte(c,chan_ptr);
                DO_VERBOSE_TRACE("Add [0x%02x]\n", c);
                do {
                    clock_gettime(CLOCK_MONOTONIC, &current_t);
                    elapsed_sec = diff_timespec(&current_t, &start_t);
                    if (elapsed_sec>timeout) {
                        break;
                    }
                    tempval = read_single_byte(port_num);
                } while (tempval < 0);
                if (tempval < 0) {
                    DO_VERBOSE_TRACE("Timeout in read_byte\n");
                    DO_VERBOSE_TRACE("Continuing...\n");
                    continue;
                }

                c = (uint8_t) tempval;
                add_byte(c,chan_ptr);
                DO_VERBOSE_TRACE("Add [0x%02x]\n", c);
                in_escape = 0;

            } else if ((c & 0xFF) == (END_FRAME_BYTE & 0xFF)) {
                add_byte(c,chan_ptr);
                DO_VERBOSE_TRACE("Add [0x%02x]\n", c);
                am_in_frame = UNFRAMED;
                DO_VERBOSE_TRACE("End frame byte\n");
                DO_VERBOSE_TRACE("Am in UNFRAMED\n");

                found_complete_frame = 1;
                break;

            } else {
                add_byte(c,chan_ptr);
                DO_VERBOSE_TRACE("Add [0x%02x]\n", c);
            }

            DO_DEBUG_TRACE("(%02d:%02X)",chan_ptr->channel,c);

            DO_VERBOSE_TRACE("Continuing...\n");
            continue;

        } else { //Unframed data

            DO_DEBUG_TRACE("(U:%02X)",c);
            add_byte(c,unframed_ptr);
            DO_VERBOSE_TRACE("Add [0x%02x] to unframed\n", c);
        }

    }

    if (found_complete_frame) {

        if (channel_ID == 0xff) {
            *receiveBuffer = other_frame_ID;
            *receiveBytes = chan_ptr->len;
            DO_VERBOSE_TRACE("Frame completed in 0xff mode\n");
        } else {
            extract_payload(&(in_frame_buf[FRAME_START_IDX]), (byte_idx - NUM_FRAMING_BYTES), receiveBuffer, receiveBytes);
        }

        return 0;
    } else {
        *receiveBytes =  0;
        return 0;
    }
}

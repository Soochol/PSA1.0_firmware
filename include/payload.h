#ifndef _PAYLOAD_H
#define _PAYLOAD_H

#include "globals.h"
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.12.0)

#ifdef __cplusplus
extern "C"
{
#endif
    // uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

extern "C" {
#include "crypto-original-base64.h"
}


#define PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE 512 // maximum size of payload block per transmit

class PayloadConvert {
    public:
        PayloadConvert();
        ~PayloadConvert();
        void initialize();
        void appendToBuffer(const char* key, const char* value);
        void appendToBuffer(const char* key, int value);
        void reset(void);
        char *getBuffer(void);

        void addDeviceId();
        void setTitle(int8_t title);
        void addSensorData();
        void addDeviceData();
        void addDeviceStates();
        void addUserId();
    private:
        char *buffer;
        size_t bufferIndex;
};

// uint8_t add_gaussian_noise(uint8_t original_value, float std_dev);  // Removed - not implemented

// Use unified buffer size from globals.h
// #define USART_MESSAGE_MAXIMUM_SIZE 256  // Removed - use USART_MESSAGE_MAXIMUM_LENGTH from globals.h
class usartMessage {
    public:
        usartMessage();
        ~usartMessage();
        void initialize();
        void appendByte(byte value);
        void reset(void);

        const char* getBuffer() const { return buffer; } // To get the pointer to the message data
        size_t getCurrentSize() const { return bufferIndex; } // To get the current length of the message

    private:
        char *buffer;
        size_t bufferIndex;
};



#endif
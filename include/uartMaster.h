#ifndef _ESP_USART_MASTER_H
#define _ESP_USART_MASTER_H

#include "globals.h"
#include "tinyml.h"
#include "payload.h"

#define ACK             0x06
#define NACK            0x15

#define STM             0x02    // start byte
#define ETX             0x03    // end byte

#define MSG_REQUEST     0x20    // Request message (any initiator)
#define MSG_RESPONSE    0x02    // Response/ACK message (any responder)

#define initTempSleep   0x10
#define initTempWaiting 0x11
#define initTempForceUp 0x12
#define initTempHeatPad 0x13
// #define initPWMFan      0x14
#define initPWMCoolFan  0x15
#define initTout        0x16
#define initDelay       0x17

#define reqTempSleep    0x30
#define reqTempWaiting  0x31
#define reqTempForceUp  0x32
#define reqTempHeatPad  0x33    // DEPRECATED
#define reqUpperTemp    0x34
#define reqPWMCoolFan   0x35
#define reqTimeout      0x36
#define reqSpk          0x37
#define reqDelay        0x38

#define ctrlReset       0x50
#define ctrlMode        0x51
#define ctrlSpkVol      0x52
#define ctrlFanOn       0x53
#define ctrlFanPWM      0x54
#define ctrlCoolFanOn   0x55
#define ctrlCoolFanPWM  0x56
#define ctrlHeatPadOn   0x57
#define ctrlHeatPadTemp 0x58
#define ctrlPose        0x59

#define statMessage     0x70

#define evtInitStart    0x80
#define evtInitResult   0x81
#define evtMode         0x82

#define errInit         0x90

// User-friendly enum definitions
enum class DeviceMode : uint8_t {
    SLEEP = 0,
    WAITING = 1, 
    FORCE_UP = 2,
    FORCE_ON = 3,
    FORCE_DOWN = 4,
    IMU = 5,
    ERROR = 6
};

// Command state management for ESP->STM communication
enum class CommandState : uint8_t {
    IDLE = 0,
    SENT = 1,
    ACK_RECEIVED = 2,
    DATA_RECEIVED = 3,
    TIMEOUT = 4,
    ERROR = 5
};

enum class ResponseType : uint8_t {
    ACK_ONLY = 0,      // INIT and CONTROL commands expect only ACK
    DATA_RESPONSE = 1  // REQUEST commands expect data response
};

// Timeout constants (milliseconds)
#define COMMAND_TIMEOUT_MS       5000   // 5 seconds for ACK
#define REQUEST_TIMEOUT_MS       5000   // 5 seconds for data response
#define MAX_RETRY_ATTEMPTS       3      // Maximum retry attempts
#define RESPONSE_BUFFER_SIZE     64     // Maximum response data size

// Multi-command management constants
#define MAX_PENDING_COMMANDS     10      // Maximum simultaneous pending commands
#define MAX_CONSECUTIVE_TIMEOUTS 3      // Maximum consecutive timeouts before critical error
#define CRITICAL_ERROR_THRESHOLD 5000   // 5 seconds of communication failure = critical

// System state management
enum class SystemState : uint8_t {
    NORMAL = 0,
    COMMUNICATION_ERROR = 1,
    CRITICAL_ERROR = 2
};

// Command tracking structure
struct PendingCommand {
    uint8_t command;
    ResponseType expectedResponse;
    CommandState state;
    uint32_t sentTimestamp;
    uint8_t retryCount;
    uint8_t responseData[RESPONSE_BUFFER_SIZE];
    size_t responseLength;
    
    // Original command data for retry
    uint8_t originalData[RESPONSE_BUFFER_SIZE];
    size_t originalDataLength;
    
    void reset() {
        command = 0;
        expectedResponse = ResponseType::ACK_ONLY;
        state = CommandState::IDLE;
        sentTimestamp = 0;
        retryCount = 0;
        responseLength = 0;
        originalDataLength = 0;
        memset(responseData, 0, RESPONSE_BUFFER_SIZE);
        memset(originalData, 0, RESPONSE_BUFFER_SIZE);
    }
    
    bool isTimeoutExpired(uint32_t currentTime) const {
        uint32_t timeout = (expectedResponse == ResponseType::DATA_RESPONSE) ? 
                          REQUEST_TIMEOUT_MS : COMMAND_TIMEOUT_MS;
        // Fixed timeout regardless of retry count for faster STM communication failure detection
        return (currentTime - sentTimestamp) > timeout;
    }
    
    void backupOriginalData(const byte* data, size_t dataLen) {
        originalDataLength = (dataLen > RESPONSE_BUFFER_SIZE) ? RESPONSE_BUFFER_SIZE : dataLen;
        if (data && originalDataLength > 0) {
            memcpy(originalData, data, originalDataLength);
        } else {
            originalDataLength = 0;
        }
    }
};

enum class MessageState : uint8_t {
    WAITING_START = 0,
    READING_LENGTH,
    READING_DIRECTION,
    READING_COMMAND,
    READING_DATA,
    READING_CHECKSUM,
    READING_END,
    MESSAGE_COMPLETE
};

enum class CommandType : uint8_t {
    INIT = 0x10,
    REQUEST = 0x30,
    CONTROL = 0x50,
    STATUS = 0x70,
    EVENT = 0x80,
    ERROR = 0x90
};

// Structure for temperature configuration
struct TemperatureConfig {
    float targetTemp;
    float maxTemp;
    
    // Separate integer/decimal parts according to protocol
    uint8_t getIntegerPart() const { return (uint8_t)targetTemp; }
    uint8_t getDecimalPart() const { return (uint8_t)((targetTemp - (int)targetTemp) * 100); }
    uint8_t getMaxIntegerPart() const { return (uint8_t)maxTemp; }
    uint8_t getMaxDecimalPart() const { return (uint8_t)((maxTemp - (int)maxTemp) * 100); }
};

// Structured access for sensor data
struct SensorReading {
    // IMU data (Left/Right)
    int16_t leftGyro[3];    // X, Y, Z
    int16_t leftAccel[3];   // X, Y, Z  
    int16_t rightGyro[3];   // X, Y, Z
    int16_t rightAccel[3];  // X, Y, Z
    
    // Pressure sensors
    int16_t leftPressure;
    int16_t rightPressure;
    
    // Temperature sensors
    int16_t outsideTemp;
    int16_t boardTemp; 
    int16_t actuatorTemp;
    
    // Other sensors
    int16_t actuatorDisplacement;
    int16_t objectDistance;
    int16_t batteryVoltage;
    
    // IMU events
    uint8_t leftIMUEvent;
    uint8_t rightIMUEvent;

};

// Structure for message parsing
struct ProtocolMessage {
    uint8_t stm;
    uint8_t length;
    uint8_t direction;
    uint8_t command;
    uint8_t data[64];  // Maximum data size
    uint8_t dataLength;
    uint8_t checksum;
    uint8_t etx;
    
    bool isValid() const {
        return stm == STM && etx == ETX && direction == MSG_REQUEST;
    }
    
    CommandType getCommandType() const {
        return static_cast<CommandType>(command & 0xF0);
    }
};

// Existing functions
bool initUartMaster();

uint8_t calculateChecksum(const byte* data, size_t length);
bool checkMessage(const byte* incomingByteArray, size_t length);

void sendResponse(uint8_t receivedCommand);
void sendAckResponse(uint8_t receivedCommand);
void nackResponse(); // Note: NACK not defined in protocol, logs only

// New bidirectional communication functions
bool sendCommandAsync(uint8_t command, const byte* data = nullptr, size_t dataLen = 0);
// bool sendCommandWithAck(uint8_t command, const byte* data = nullptr, size_t dataLen = 0, uint32_t timeoutMs = COMMAND_TIMEOUT_MS);
// bool sendRequestWithResponse(uint8_t command, uint8_t* responseBuffer, size_t* responseLength, uint32_t timeoutMs = REQUEST_TIMEOUT_MS);
// bool waitForResponse(uint8_t expectedCommand, ResponseType responseType, uint32_t timeoutMs);
void handleInitResponse();
void handleRequestResponse();
void handleControlResponse();
bool isResponseExpected();
void processCommandTimeout();
CommandState getCommandState(uint8_t command);

// Error handling and diagnostic functions
const char* getCommandStateString(CommandState state);
// const char* getLastErrorString();
bool isCommandInProgress();
void clearPendingCommand();
uint32_t getLastCommandTimestamp();
uint8_t getCommandRetryCount();

// System state management functions
SystemState getCurrentSystemState();
bool isSystemInErrorState();
bool isCommunicationHealthy();
bool manualRecoveryFromCriticalError();

// Critical error handling functions
void handleCommunicationError(uint32_t currentTime);
void enterCriticalErrorState();
void emergencyShutdownActuators();
void setSystemErrorMode();
void notifySystemError();
void checkCommunicationRecovery();

// New user-friendly API functions
bool setSleepTemperature(float targetTemp);
bool setWaitingTemperature(float targetTemp);
bool setOperatingTemperature(float targetTemp);
bool setUpperTemperatureLimit(float limitTemp);

bool setDeviceMode(DeviceMode mode);
bool setFanState(bool enabled);
bool setFanSpeed(uint8_t speed_0_to_3);
bool setCoolingFanState(bool enabled);
bool setCoolingFanLevel(uint8_t level);
bool setHeatPadState(bool enabled);  // DEPRECATED - ctrlHeatPadOn not supported
bool setHeatPadLevel(uint8_t level);  // DEPRECATED - ctrlHeatPadTemp not supported
bool setSpeakerVolume(uint8_t volume_0_to_10);

SensorReading getCurrentSensorData();
DeviceMode getCurrentMode();
uint8_t getLastErrorCode();

// Async Request functions (Non-blocking)
bool requestSleepTemperature();
bool requestWaitingTemperature();
bool requestOperatingTemperature();
bool requestUpperTemperatureLimit();
// bool requestHeatPadLevel(); // DEPRECATED
bool requestCoolingFanLevel();
bool requestSpeakerVolume();
bool requestDetectionDelay();
bool requestAllParameters();

// System configuration getter functions
float getSleepTemperature();
float getWaitingTemperature();
float getOperatingTemperature();
float getUpperTemperatureLimit();
uint8_t getCoolingFanLevel();
uint8_t getMaxCoolingFanLevel();
uint8_t getSpeakerVolume();
uint16_t getForceUpTimeout();
uint16_t getForceOnTimeout();
uint16_t getForceDownTimeout();
uint16_t getWaitingTimeout();
uint8_t getPoseDetectionDelay();
uint8_t getObjectDetectionDelay();

// Utility functions
bool resetDevice();
bool isPoseDetectionMode();
bool isObjectDetectionMode();
bool setPoseDetectionMode(bool enabled);

// Command name lookup function
const char* getCommandName(uint8_t command);

// ESP→STM Communication Test Functions
void testESPtoSTMCommunication();

bool sendCommandSafe(byte command, const byte* data = nullptr, size_t dataLen = 0);
size_t buildMessage(uint8_t* buffer, byte command, const byte* data = nullptr, size_t dataLen = 0);
void resetParser();
void parseSensorData(const uint8_t* data, size_t length);

#endif
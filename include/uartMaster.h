/**
 * @file uartMaster.h
 * @brief ESP32-STM32 UART Communication Protocol for PSA (Postural Stability Assistant) Firmware
 * 
 * This module handles bidirectional communication between ESP32 and STM32 microcontrollers
 * in a postural stability assistant device. It manages device modes, sensor data collection,
 * temperature/fan control, and system state monitoring through a custom UART protocol.
 * 
 * Key Features:
 * - Asynchronous command/response protocol with response/timeout handling
 * - Device mode management (sleep, waiting, force modes, IMU mode)
 * - Real-time sensor data processing (IMU, pressure, temperature)
 * - Temperature and fan control system
 * - System health monitoring and error recovery
 * - TinyML integration for pose detection
 * - Korean protocol specification compliance with gyro angle and mode controls
 */

#ifndef _ESP_USART_MASTER_H
#define _ESP_USART_MASTER_H

#include "globals.h"
#include "tinyml.h"
#include "payload.h"

// =============================================================================
// Protocol Constants
// =============================================================================

// Message frame delimiters
#define STM             0x02    // Start byte - begins every message
#define ETX             0x03    // End byte - terminates every message

// Message direction indicators
#define MSG_REQUEST     0x20    // Request message (any initiator)
#define MSG_RESPONSE    0x02    // Response/ACK message (any responder)

// =============================================================================
// Command Codes by Category
// =============================================================================

// INIT Commands (0x10-0x22) - Initialize device parameters
#define initTempSleep           0x10    // Set sleep mode temperature
#define initTempWaiting         0x11    // Set waiting mode temperature
#define initTempForceUp         0x12    // Set force-up mode temperature
#define initTempLimit           0x14    // Set upper temperature limit
#define initPWMCoolFan          0x15    // Set cooling fan PWM level
#define initTout                0x16    // Set timeout values
#define initSpk                 0x17    // Set speaker volume
#define initDelay               0x18    // Set detection delay values
#define initGyroAct             0x19    // Set heating angle (IMU active angle)
#define initGyroRel             0x20    // Set cooling angle (IMU relative angle)
#define initMode                0x21    // Set mode change (0: AI mode, 1: IMU mode)
#define initWearableFanPWM      0x22    // Set wearable fan PWM speed (0-3)

// REQUEST Commands (0x30-0x42) - Query current parameter values
#define reqTempSleep            0x30    // Get sleep mode temperature
#define reqTempWaiting          0x31    // Get waiting mode temperature
#define reqTempForceUp          0x32    // Get force-up mode temperature
#define reqUpperTemp            0x34    // Get upper temperature limit
#define reqPWMCoolFan           0x35    // Get cooling fan PWM level
#define reqTimeout              0x36    // Get timeout values
#define reqSpk                  0x37    // Get speaker volume
#define reqDelay                0x38    // Get ForceDown delay value
#define reqGyroAct              0x39    // Get heating angle (IMU active angle)
#define reqGyroRel              0x40    // Get cooling angle (IMU relative angle)
#define reqMode                 0x41    // Get current mode (0: AI mode, 1: IMU mode)
#define reqPWMFan               0x42    // Get PWM fan speed

// CONTROL Commands (0x50-0x55) - Real-time device control
#define ctrlReset               0x50    // Reset device
#define ctrlMode                0x51    // Set device operating mode
#define ctrlSpkOn               0x52    // Turn speaker on/off
#define ctrlWearableFanOn       0x53    // Turn wearable fan on/off
#define ctrlCoolFanOn           0x55    // Turn cooling fan on/off

// STATUS Commands (0x70-0x79) - Status and sensor data
#define statMessage             0x70    // Sensor data message from STM

// EVENT Commands (0x80-0x89) - System events and notifications
#define evtInitStart            0x80    // Initialization started event
#define evtInitResult           0x81    // Initialization result event
#define evtMode                 0x82    // Device mode change event

// ERROR Commands (0x90-0x99) - Error notifications
#define errInit                 0x90    // Initialization error

// Error code bit masks for errInit (0x90)
#define ERR_IR_TEMP             0x0001  // IR temperature error
#define ERR_AMB_TEMP            0x0002  // Ambient temperature error
#define ERR_IMU                 0x0004  // IMU error
#define ERR_FAN                 0x0008  // Fan error
#define ERR_COOL_FAN            0x0010  // Cooling fan error
#define ERR_TOF                 0x0020  // TOF error
#define ERR_AUDIO               0x0040  // Audio error
#define ERR_FSR                 0x0080  // FSR error
#define ERR_SD_CARD             0x0100  // SD card error
#define ERR_MP3_FILE            0x0200  // MP3 file error

// =============================================================================
// Enum Definitions
// =============================================================================

/**
 * @brief Device operating modes for postural stability control
 */
enum class DeviceMode : uint8_t {
    SLEEP = 0,      // Device in sleep mode
    WAITING = 1,    // Waiting for user input/detection
    FORCE_UP = 2,   // Actively raising posture
    FORCE_ON = 3,   // Maintaining raised posture
    FORCE_DOWN = 4, // Lowering posture
    ERROR = 5       // STM in error state
};

/**
 * @brief Device operation control mode
 */
enum class OperationMode : uint8_t {
    AI_MODE = 0,    // ESP controls force up/down operations
    IMU_MODE = 1    // STM controls force up/down operations
};

/**
 * @brief Command state tracking for ESP->STM communication
 */
enum class CommandState : uint8_t {
    IDLE = 0,           // No command in progress
    SENT = 1,           // Command sent, awaiting response
    ACK_RECEIVED = 2,   // Acknowledgment received
    DATA_RECEIVED = 3,  // Data response received
    TIMEOUT = 4,        // Command timed out
    ERROR = 5           // Communication error
};

/**
 * @brief Expected response type for commands
 */
enum class ResponseType : uint8_t {
    ACK_ONLY = 0,       // INIT and CONTROL commands expect only ACK
    DATA_RESPONSE = 1   // REQUEST commands expect data response
};

/**
 * @brief Operating temperature level presets for force-up mode
 */
enum class OperatingTempLevel : uint8_t {
    FORCE_HIGH = 66,    // High temperature mode
    FORCE_NORMAL = 58,  // Normal temperature mode
    FORCE_LOW = 52      // Low temperature mode
};

/**
 * @brief Overall system health state
 */
enum class SystemState : uint8_t {
    NORMAL = 0,             // System operating normally
    COMMUNICATION_ERROR = 1, // Temporary communication issues
    CRITICAL_ERROR = 2      // Critical failure state
};

/**
 * @brief Command type categories for protocol organization
 */
enum class CommandType : uint8_t {
    INIT = 0x10,    // Initialization commands (0x10-0x29)
    REQUEST = 0x30, // Data request commands (0x30-0x49)
    CONTROL = 0x50, // Control commands (0x50-0x69)
    STATUS = 0x70,  // Status messages (0x70-0x79)
    EVENT = 0x80,   // Event notifications (0x80-0x89)
    ERROR = 0x90,   // Error notifications (0x90-0x99)
    UNKNOWN = 0xFF  // Unknown command type
};

/**
 * @brief Message parsing state machine states
 */
enum class MessageState : uint8_t {
    WAITING_START = 0,  // Looking for start byte
    READING_LENGTH,     // Reading length field
    READING_DIRECTION,  // Reading direction field
    READING_COMMAND,    // Reading command field
    READING_DATA,       // Reading data payload
    READING_CHECKSUM,   // Reading checksum
    READING_END,        // Reading end byte
    MESSAGE_COMPLETE    // Message parsing complete
};

// Protocol timeout constants (milliseconds)
#define COMMAND_TIMEOUT_MS       5000   // 5 seconds for ACK
#define REQUEST_TIMEOUT_MS       5000   // 5 seconds for data response
#define MAX_RETRY_ATTEMPTS       3      // Maximum retry attempts
#define RESPONSE_BUFFER_SIZE     64     // Maximum response data size
#define MAX_PENDING_COMMANDS     20     // Maximum simultaneous pending commands
#define MAX_CONSECUTIVE_TIMEOUTS 3      // Maximum consecutive timeouts before critical error
#define CRITICAL_ERROR_THRESHOLD 5000   // 5 seconds of communication failure = critical

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Temperature configuration structure
 */
struct TemperatureConfig {
    float targetTemp;   // Target temperature in Celsius
    float maxTemp;      // Maximum temperature limit in Celsius
    
    uint8_t getIntegerPart() const { return (uint8_t)targetTemp; }
    uint8_t getDecimalPart() const { return (uint8_t)((targetTemp - (int)targetTemp) * 100); }
    uint8_t getMaxIntegerPart() const { return (uint8_t)maxTemp; }
    uint8_t getMaxDecimalPart() const { return (uint8_t)((maxTemp - (int)maxTemp) * 100); }
};

/**
 * @brief Comprehensive sensor data structure
 */
struct SensorReading {
    // IMU sensor data for left and right foot sensors
    int16_t leftGyro[3];    // Left foot gyroscope (X, Y, Z axes)
    int16_t leftAccel[3];   // Left foot accelerometer (X, Y, Z axes)
    int16_t rightGyro[3];   // Right foot gyroscope (X, Y, Z axes)
    int16_t rightAccel[3];  // Right foot accelerometer (X, Y, Z axes)
    
    // Pressure sensor readings for weight distribution
    int16_t leftPressure;   // Left foot pressure sensor
    int16_t rightPressure;  // Right foot pressure sensor
    
    // Temperature sensors for thermal management
    int16_t outsideTemp;    // External ambient temperature
    int16_t boardTemp;      // PCB/electronics temperature
    int16_t actuatorTemp;   // Actuator motor temperature
    
    // Mechanical and electrical sensors
    int16_t actuatorDisplacement;   // Linear actuator position
    int16_t objectDistance;         // Distance sensor reading
    int16_t batteryVoltage;         // System battery voltage
    
    // IMU event flags for motion detection
    uint8_t leftIMUEvent;   // Left IMU motion event flags
    uint8_t rightIMUEvent;  // Right IMU motion event flags
};

/**
 * @brief Command tracking structure for asynchronous ESP->STM communication
 */
struct PendingCommand {
    uint8_t command;                                // Command code being tracked
    ResponseType expectedResponse;                  // Expected response type (ACK/DATA)
    CommandState state;                            // Current command state
    uint32_t sentTimestamp;                        // When command was sent (for timeout)
    uint8_t retryCount;                           // Number of retry attempts made
    uint8_t responseData[RESPONSE_BUFFER_SIZE];   // Received response data
    size_t responseLength;                        // Length of received response
    uint8_t originalData[RESPONSE_BUFFER_SIZE];   // Original command data
    size_t originalDataLength;                    // Original data length
    
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

/**
 * @brief Protocol message structure for UART communication
 */
struct ProtocolMessage {
    uint8_t stm;            // Start byte (0x02)
    uint8_t length;         // Message length
    uint8_t direction;      // Message direction (0x20=request, 0x02=response)
    uint8_t command;        // Command code
    uint8_t data[64];       // Payload data
    uint8_t dataLength;     // Actual data length
    uint8_t checksum;       // XOR checksum for verification
    uint8_t etx;            // End byte (0x03)
    
    bool isValid() const {
        return stm == STM && etx == ETX && direction == MSG_REQUEST;
    }
    
    bool isValidStrict(const char** errorMsg = nullptr) const {
        if (stm != STM) {
            if (errorMsg) *errorMsg = "Invalid STM start byte";
            return false;
        }
        if (direction != MSG_REQUEST && direction != MSG_RESPONSE) {
            if (errorMsg) *errorMsg = "Invalid direction";
            return false;
        }
        if (etx != ETX) {
            if (errorMsg) *errorMsg = "Invalid ETX end byte";
            return false;
        }
        return true;
    }
    
    CommandType getCommandType() const {
        if (command >= 0x10 && command <= 0x29) return CommandType::INIT;
        if (command >= 0x30 && command <= 0x49) return CommandType::REQUEST;
        if (command >= 0x50 && command <= 0x69) return CommandType::CONTROL;
        if (command >= 0x70 && command <= 0x79) return CommandType::STATUS;
        if (command >= 0x80 && command <= 0x89) return CommandType::EVENT;
        if (command >= 0x90 && command <= 0x99) return CommandType::ERROR;
        return CommandType::UNKNOWN;
    }
};

// =============================================================================
// Function Declarations
// =============================================================================

// -----------------------------------------------
// Core System Functions
// -----------------------------------------------
bool initUartMaster();
bool resetDevice();

// -----------------------------------------------
// Protocol and Message Handling
// -----------------------------------------------
uint8_t calculateChecksum(const byte* data, size_t length);
bool checkMessage(const byte* incomingByteArray, size_t length);
void sendResponse(uint8_t receivedCommand);
void sendAckResponse(uint8_t receivedCommand);
bool sendCommandSafe(byte command, const byte* data = nullptr, size_t dataLen = 0);
size_t buildMessage(uint8_t* buffer, byte command, const byte* data = nullptr, size_t dataLen = 0);
void resetParser();
void parseSensorData(const uint8_t* data, size_t length);
void logSensorDataFormatted(const SensorReading& reading);

// -----------------------------------------------
// Asynchronous Communication Management
// -----------------------------------------------
bool sendCommandAsync(uint8_t command, const byte* data = nullptr, size_t dataLen = 0);
bool sendCommandFireAndForget(uint8_t command, const byte* data = nullptr, size_t dataLen = 0);
void handleInitResponse();
void handleRequestResponse();
void handleControlResponse();
bool isResponseExpected();
void processCommandTimeout();
CommandState getCommandState(uint8_t command);

// -----------------------------------------------
// Error Handling and System Diagnostics
// -----------------------------------------------
const char* getCommandStateString(CommandState state);
bool isCommandInProgress();
void clearPendingCommand();
uint32_t getLastCommandTimestamp();
uint8_t getCommandRetryCount();
SystemState getCurrentSystemState();
bool isSystemInErrorState();
bool isCommunicationHealthy();
void handleCommunicationError(uint32_t currentTime);
void enterCriticalErrorState();
void emergencyShutdown();
void notifySystemError();

// -----------------------------------------------
// Device Configuration API (INIT Commands)
// -----------------------------------------------
bool initSleepTemperature(uint8_t tempInt, uint8_t tempDec);
bool initWaitingTemperature(uint8_t tempInt, uint8_t tempDec);
bool initOperatingTemperature(uint8_t tempInt, uint8_t tempDec);
bool initOperatingTemperature(OperatingTempLevel level);
bool initUpperTemperatureLimit(uint8_t tempInt, uint8_t tempDec);
bool initTimeoutConfiguration(uint16_t forceUpTimeout, uint16_t forceOnTimeout, uint16_t forceDownTimeout, uint16_t waitingTimeout);
bool initCoolingFanPWM(uint8_t currentLevel, uint8_t maxLevel);
bool initForeDownDelay(uint8_t forceDownDelay);
bool initSpeakerVolume(uint8_t volume_0_to_10);
bool initGyroActiveAngle(uint8_t activeAngle);
bool initGyroRelativeAngle(uint8_t relativeAngle);
bool initDeviceMode(uint8_t mode);
bool initWearableFanSpeed(uint8_t fanSpeed);

// -----------------------------------------------
// Device Control API (CONTROL Commands)
// -----------------------------------------------
bool setOperationMode(uint8_t mode);
bool updateStmState(uint8_t modeValue);
bool updateStmState(DeviceMode mode);
bool updateEspState(SystemState newState);
bool setDeviceMode(DeviceMode mode);
bool setWearableFanState(bool enabled);
bool setCoolingFanState(bool enabled);
bool setSpeakerState(bool enabled);

// -----------------------------------------------
// Data Retrieval API (REQUEST Commands)
// -----------------------------------------------
bool requestSleepTemperature();
bool requestWaitingTemperature();
bool requestOperatingTemperature();
bool requestUpperTemperatureLimit();
bool requestTimeoutConfiguration();
bool requestCoolingFanLevel();
bool requestSpeakerVolume();
bool requestDetectionDelay();
bool requestGyroActiveAngle();
bool requestGyroRelativeAngle();
bool requestCurrentMode();
bool requestPWMFanSpeed();
bool requestAllParameters();

// -----------------------------------------------
// Data Getters (Cached Values)
// -----------------------------------------------
SensorReading getCurrentSensorData();
DeviceMode getCurrentMode();
OperationMode getCurrentOperationMode();
bool isAIModeActive();
uint8_t getLastErrorCode();
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
uint8_t getForceDownDelay();
uint8_t getGyroActiveAngle();
uint8_t getGyroRelativeAngle();
uint8_t getCurrentModeValue();
uint8_t getPWMFanSpeed();

// -----------------------------------------------
// Utility Functions
// -----------------------------------------------
bool isPoseDetectionMode();
bool isObjectDetectionMode();
const char* getCommandName(uint8_t command);

#endif
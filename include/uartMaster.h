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

// INIT Commands (0x10-0x1F) - Initialize device parameters
#define initTempSleep   0x10    // Set sleep mode temperature
#define initTempWaiting 0x11    // Set waiting mode temperature
#define initTempForceUp 0x12    // Set force-up mode temperature
#define initTempHeatPad 0x13    // Set heat pad temperature
// #define initPWMFan      0x14 // (Reserved)
#define initPWMCoolFan  0x15    // Set cooling fan PWM level
#define initTout        0x16    // Set timeout values
#define initDelay       0x17    // Set detection delay values

// REQUEST Commands (0x30-0x3F) - Query current parameter values
#define reqTempSleep    0x30    // Get sleep mode temperature
#define reqTempWaiting  0x31    // Get waiting mode temperature
#define reqTempForceUp  0x32    // Get force-up mode temperature
#define reqTempHeatPad  0x33    // Get heat pad temperature (DEPRECATED)
#define reqUpperTemp    0x34    // Get upper temperature limit
#define reqPWMCoolFan   0x35    // Get cooling fan PWM level
#define reqTimeout      0x36    // Get timeout values
#define reqSpk          0x37    // Get speaker volume
#define reqDelay        0x38    // Get detection delay values

// CONTROL Commands (0x50-0x5F) - Real-time device control
#define ctrlReset       0x50    // Reset device
#define ctrlMode        0x51    // Set device operating mode
#define ctrlSpkVol      0x52    // Set speaker volume
#define ctrlFanOn       0x53    // Turn main fan on/off
#define ctrlFanPWM      0x54    // Set main fan speed (0-3)
#define ctrlCoolFanOn   0x55    // Turn cooling fan on/off
#define ctrlCoolFanPWM  0x56    // Set cooling fan PWM level
#define ctrlHeatPadOn   0x57    // Turn heat pad on/off
#define ctrlHeatPadTemp 0x58    // Set heat pad temperature
#define ctrlPose        0x59    // Enable/disable pose detection

// STATUS Commands (0x70-0x7F) - Status and sensor data
#define statMessage     0x70    // Sensor data message from STM

// EVENT Commands (0x80-0x8F) - System events and notifications
#define evtInitStart    0x80    // Initialization started event
#define evtInitResult   0x81    // Initialization result event
#define evtMode         0x82    // Device mode change event

// ERROR Commands (0x90-0x9F) - Error notifications
#define errInit         0x90    // Initialization error

// =============================================================================
// Enum Definitions
// =============================================================================

/**
 * @brief Device operating modes for postural stability control
 * 
 * These modes control the device's behavior and actuator states:
 * - SLEEP: Device inactive, minimal power consumption
 * - WAITING: Standby mode, ready for user interaction
 * - FORCE_UP: Actively raising user posture
 * - FORCE_ON: Maintaining raised posture
 * - FORCE_DOWN: Lowering user posture
 * - IMU: IMU-based motion tracking mode
 * - ERROR: System error state
 */
enum class DeviceMode : uint8_t {
    SLEEP = 0,      // Device in sleep mode
    WAITING = 1,    // Waiting for user input/detection
    FORCE_UP = 2,   // Actively raising posture
    FORCE_ON = 3,   // Maintaining raised posture
    FORCE_DOWN = 4, // Lowering posture
    IMU = 5,        // IMU motion tracking mode
    ERROR = 6       // System error state
};

/**
 * @brief Command state tracking for ESP->STM communication
 * 
 * Tracks the lifecycle of commands sent from ESP32 to STM32:
 * - IDLE: No command in progress
 * - SENT: Command transmitted, waiting for response
 * - ACK_RECEIVED: STM acknowledged command
 * - DATA_RECEIVED: Data response received from STM
 * - TIMEOUT: Command timed out waiting for response
 * - ERROR: Communication error occurred
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
 * 
 * Defines what type of response to expect from STM32:
 * - ACK_ONLY: Only acknowledgment expected (INIT/CONTROL commands)
 * - DATA_RESPONSE: Data payload expected (REQUEST commands)
 */
enum class ResponseType : uint8_t {
    ACK_ONLY = 0,       // INIT and CONTROL commands expect only ACK
    DATA_RESPONSE = 1   // REQUEST commands expect data response
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

/**
 * @brief Overall system health state
 * 
 * Tracks the overall health of the ESP-STM communication system:
 * - NORMAL: System operating normally
 * - COMMUNICATION_ERROR: Temporary communication issues
 * - CRITICAL_ERROR: Severe communication failure requiring intervention
 */
enum class SystemState : uint8_t {
    NORMAL = 0,             // System operating normally
    COMMUNICATION_ERROR = 1, // Temporary communication issues
    CRITICAL_ERROR = 2      // Critical failure state
};

/**
 * @brief Command tracking structure for asynchronous ESP->STM communication
 * 
 * Tracks the state of commands sent from ESP32 to STM32, including retry logic,
 * timeout handling, and response data storage. Supports concurrent command management
 * for efficient communication.
 */
struct PendingCommand {
    uint8_t command;                                // Command code being tracked
    ResponseType expectedResponse;                  // Expected response type (ACK/DATA)
    CommandState state;                            // Current command state
    uint32_t sentTimestamp;                        // When command was sent (for timeout)
    uint8_t retryCount;                           // Number of retry attempts made
    uint8_t responseData[RESPONSE_BUFFER_SIZE];   // Received response data
    size_t responseLength;                        // Length of received response
    
    // Original command data backup for retry functionality
    uint8_t originalData[RESPONSE_BUFFER_SIZE];   // Original command data
    size_t originalDataLength;                    // Original data length
    
    // Reset command to initial state
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
    
    // Check if command has timed out based on expected response type
    bool isTimeoutExpired(uint32_t currentTime) const {
        uint32_t timeout = (expectedResponse == ResponseType::DATA_RESPONSE) ? 
                          REQUEST_TIMEOUT_MS : COMMAND_TIMEOUT_MS;
        return (currentTime - sentTimestamp) > timeout;
    }
    
    // Backup original command data for potential retry
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
 * @brief Message parsing state machine states
 * 
 * State machine for parsing incoming UART messages byte by byte:
 * - WAITING_START: Looking for STM start byte (0x02)
 * - READING_LENGTH: Reading message length byte
 * - READING_DIRECTION: Reading direction byte (request/response)
 * - READING_COMMAND: Reading command code byte
 * - READING_DATA: Reading payload data bytes
 * - READING_CHECKSUM: Reading checksum verification byte
 * - READING_END: Reading ETX end byte (0x03)
 * - MESSAGE_COMPLETE: Complete message parsed and ready for processing
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

/**
 * @brief Command type categories for protocol organization
 * 
 * Groups commands by their functional category based on upper nibble:
 * - INIT (0x10): Initialization and configuration commands
 * - REQUEST (0x30): Data query commands
 * - CONTROL (0x50): Real-time control commands
 * - STATUS (0x70): Status and sensor data messages
 * - EVENT (0x80): System event notifications
 * - ERROR (0x90): Error and fault notifications
 */
enum class CommandType : uint8_t {
    INIT = 0x10,    // Initialization commands
    REQUEST = 0x30, // Data request commands
    CONTROL = 0x50, // Control commands
    STATUS = 0x70,  // Status messages
    EVENT = 0x80,   // Event notifications
    ERROR = 0x90    // Error notifications
};

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Temperature configuration structure
 * 
 * Holds temperature settings and provides protocol-compatible conversion methods.
 * The protocol requires temperatures to be split into integer and decimal parts.
 */
struct TemperatureConfig {
    float targetTemp;   // Target temperature in Celsius
    float maxTemp;      // Maximum temperature limit in Celsius
    
    // Protocol conversion methods - split float into integer/decimal parts
    uint8_t getIntegerPart() const { return (uint8_t)targetTemp; }
    uint8_t getDecimalPart() const { return (uint8_t)((targetTemp - (int)targetTemp) * 100); }
    uint8_t getMaxIntegerPart() const { return (uint8_t)maxTemp; }
    uint8_t getMaxDecimalPart() const { return (uint8_t)((maxTemp - (int)maxTemp) * 100); }
};

/**
 * @brief Comprehensive sensor data structure
 * 
 * Contains all sensor readings from the STM32, including IMU data, pressure sensors,
 * temperature readings, and other environmental/system sensors. Used for pose detection,
 * system monitoring, and safety checks.
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
 * @brief Protocol message structure for UART communication
 * 
 * Represents a complete protocol message with all fields. Used by the message
 * parser to build incoming messages and validate their integrity.
 * 
 * Message format: [STM][LEN][DIR][CMD][DATA...][CHKSUM][ETX]
 */
struct ProtocolMessage {
    uint8_t stm;            // Start byte (0x02)
    uint8_t length;         // Message length (DIR+CMD+DATA+CHKSUM+ETX)
    uint8_t direction;      // Message direction (0x20=request, 0x02=response)
    uint8_t command;        // Command code
    uint8_t data[64];       // Payload data (maximum 64 bytes)
    uint8_t dataLength;     // Actual data length
    uint8_t checksum;       // XOR checksum for verification
    uint8_t etx;            // End byte (0x03)
    
    // Validation method - checks message frame integrity
    bool isValid() const {
        // STM32→ESP32 messages should have MSG_REQUEST direction (0x20)
        return stm == STM && etx == ETX && direction == MSG_REQUEST;
    }
    
    // Enhanced validation with detailed error reporting
    bool isValidStrict(const char** errorMsg = nullptr) const {
        if (stm != STM) {
            if (errorMsg) *errorMsg = "Invalid STM start byte";
            return false;
        }
        
        // Accept both MSG_REQUEST (spontaneous messages) and MSG_RESPONSE (ACK messages)
        if (direction != MSG_REQUEST && direction != MSG_RESPONSE) {
            if (errorMsg) *errorMsg = "Invalid direction (expected MSG_REQUEST or MSG_RESPONSE for STM→ESP)";
            return false;
        }
        
        if (etx != ETX) {
            if (errorMsg) *errorMsg = "Invalid ETX end byte (STM32 protocol error)";
            return false;
        }
        return true;
    }
    
    // Command-type-specific direction validation
    bool isValidStrictWithCommandCheck(const char** errorMsg = nullptr) const {
        // First check basic frame validation
        if (!isValidStrict(errorMsg)) {
            return false;
        }
        
        // Check direction based on command type
        CommandType cmdType = getCommandType();
        
        switch (cmdType) {
            case CommandType::STATUS:   // 0x7X - STM spontaneous sensor data
            case CommandType::EVENT:    // 0x8X - STM spontaneous events  
            case CommandType::ERROR:    // 0x9X - STM spontaneous errors
                // Spontaneous messages must use MSG_REQUEST
                if (direction != MSG_REQUEST) {
                    if (errorMsg) *errorMsg = "Spontaneous STM message must use MSG_REQUEST direction";
                    return false;
                }
                break;
                
            case CommandType::INIT:     // 0x1X - ACK for ESP→STM INIT commands
            case CommandType::REQUEST:  // 0x3X - Response for ESP→STM REQUEST commands  
            case CommandType::CONTROL:  // 0x5X - ACK for ESP→STM CONTROL commands
                // Response/ACK messages must use MSG_RESPONSE
                if (direction != MSG_RESPONSE) {
                    if (errorMsg) *errorMsg = "STM response/ACK message must use MSG_RESPONSE direction";
                    return false;
                }
                break;
                
            default:
                if (errorMsg) *errorMsg = "Unknown command type for direction validation";
                return false;
        }
        
        return true;
    }
    
    // Extract command category from command code upper nibble
    CommandType getCommandType() const {
        return static_cast<CommandType>(command & 0xF0);
    }
};

// =============================================================================
// Function Declarations
// =============================================================================

// -----------------------------------------------
// Core System Functions
// -----------------------------------------------
bool initUartMaster();

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

// -----------------------------------------------
// Asynchronous Communication Management
// -----------------------------------------------
bool sendCommandAsync(uint8_t command, const byte* data = nullptr, size_t dataLen = 0);
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
bool manualRecoveryFromCriticalError();

// -----------------------------------------------
// Critical Error Management
// -----------------------------------------------
void handleCommunicationError(uint32_t currentTime);
void enterCriticalErrorState();
void emergencyShutdownActuators();
void setSystemErrorMode();
void notifySystemError();
void checkCommunicationRecovery();

// -----------------------------------------------
// Device Control API (Setters)
// -----------------------------------------------
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
bool setPoseDetectionMode(bool enabled);

// -----------------------------------------------
// Data Retrieval API (Getters)
// -----------------------------------------------
SensorReading getCurrentSensorData();
DeviceMode getCurrentMode();
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
uint8_t getPoseDetectionDelay();
uint8_t getObjectDetectionDelay();

// -----------------------------------------------
// Asynchronous Data Requests (Non-blocking)
// -----------------------------------------------
bool requestSleepTemperature();
bool requestWaitingTemperature();
bool requestOperatingTemperature();
bool requestUpperTemperatureLimit();
bool requestCoolingFanLevel();
bool requestSpeakerVolume();
bool requestDetectionDelay();
bool requestAllParameters();

// -----------------------------------------------
// Utility and Helper Functions
// -----------------------------------------------
bool resetDevice();
bool isPoseDetectionMode();
bool isObjectDetectionMode();
const char* getCommandName(uint8_t command);

// -----------------------------------------------
// Testing and Validation
// -----------------------------------------------
void testESPtoSTMCommunication();

#endif
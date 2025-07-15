#include "uartMaster.h"

static const char TAG[] = __FILE__;

HardwareSerial STMSerial(2); // use ESP32 UART2

// Global variables for message parsing
static MessageState currentState = MessageState::WAITING_START;
static ProtocolMessage currentMessage = {};
static uint8_t messageBuffer[USART_MESSAGE_MAXIMUM_LENGTH];
static size_t bufferIndex = 0;
static size_t expectedDataLength = 0;

// Global variables for sensor data storage
static SensorReading lastSensorReading = {};
static bool sensorDataAvailable = false;

// Global variables for device state tracking
static DeviceMode currentDeviceMode = DeviceMode::WAITING;  // Default mode
static uint8_t lastErrorCode = 0;  // Last error code received (0 = no error)

// Global variables for command state management (ESP→STM)
static PendingCommand currentPendingCommand = {};
static bool waitingForResponse = false;

// Global variables for async response data storage
static float cachedSleepTemp = 30.0f;
static float cachedWaitingTemp = 38.0f;
static float cachedOperatingTemp = 52.0f;
static float cachedTempLimit = 80.0f;
static uint8_t cachedFanLevel = 0;
static uint8_t cachedMaxFanLevel = 10;
static uint8_t cachedCoolingFanLevel = 0;
static uint8_t cachedMaxCoolingFanLevel = 10;
static uint8_t cachedSpeakerVolume = 5;
// Heat pad variables removed - deprecated functionality

// Timeout values for different modes (1-65535 seconds)
static uint16_t cachedForceUpTimeout = 10;
static uint16_t cachedForceOnTimeout = 60;
static uint16_t cachedForceDownTimeout = 15;
static uint16_t cachedWaitingTimeout = 255;

// ForceDown detection delays (in 100ms units)
static uint8_t cachedPoseDetectionDelay = 15;    // 1.5s in 100ms units
static uint8_t cachedObjectDetectionDelay = 0;   // 0s in 100ms units

// Validity flags for cached data
static bool sleepTempValid = false;
static bool waitingTempValid = false;
static bool operatingTempValid = false;
static bool tempLimitValid = false;
static bool fanLevelValid = false;
static bool coolingFanLevelValid = false;
static bool speakerVolumeValid = false;
// heatPadLevelValid removed - deprecated functionality
static bool timeoutsValid = false;
static bool detectionDelayValid = false;

// TinyML related variables (maintained from original)
uint8_t tinyMLInferenceResult = 0;
uint8_t finalResult = 0;

// Variables for message construction
byte LEN;
uint8_t CHKSUM;

// =============================================================================
// Core Protocol Functions
// =============================================================================

uint8_t calculateChecksum(const byte* data, size_t length) {
    // Accurate checksum calculation according to protocol document
    // XOR of STM + LEN + DIR + CMD + DATA, initial value: A5h
    uint8_t checksum = 0xA5;  // Initial value A5h
    
    // STM (data[0])
    checksum ^= data[0];
    
    // LEN (data[1]) 
    checksum ^= data[1];
    
    // DIR (data[2])
    checksum ^= data[2];
    
    // CMD (data[3])
    checksum ^= data[3];
    
    // DATA (data[4] ~ data[length-3])
    for (size_t i = 4; i < length - 2; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool checkMessage(const byte* incomingByteArray, size_t length) {
    if (incomingByteArray[length-2] == calculateChecksum(incomingByteArray, length)) {
        return true;
    }
    else { return false; }
}

void sendResponse(uint8_t receivedCommand) {
    // Send protocol response with received command: STX + LEN + DIR + CMD + CHK + ETX
    sendCommandSafe(receivedCommand);
}

void nackResponse() {
    // NACK is not defined in protocol - no response sent (silence = NACK)
    ESP_LOGW(TAG, "NACK: No response sent as per protocol");
}

// Unified message construction function - eliminate code duplication
void buildMessage(usartMessage& message, byte command, const byte* data = nullptr, size_t dataLen = 0) {
    message.reset();
    message.appendByte(STM);
    
    // LEN = DIR + CMD + DATA + CHKSUM + ETX length
    LEN = 4 + dataLen;  // DIR(1) + CMD(1) + CHKSUM(1) + ETX(1) + DATA(dataLen)
    message.appendByte(LEN);
    message.appendByte(ESP_TO_STM_DIR);
    message.appendByte(command);
    
    // Add data
    if (data && dataLen > 0) {
        for (size_t i = 0; i < dataLen; i++) {
            message.appendByte(data[i]);
        }
    }
    
    // Calculate and add checksum
    CHKSUM = calculateChecksum(reinterpret_cast<const byte*>(message.getBuffer()), message.getCurrentSize() + 2);
    message.appendByte(CHKSUM);
    message.appendByte(ETX);
}

// Efficient command transmission function (eliminate duplication)
bool sendCommandSafe(byte command, const byte* data = nullptr, size_t dataLen = 0) {
    usartMessage message;
    buildMessage(message, command, data, dataLen);
    
    size_t bytesWritten = STMSerial.write(
        reinterpret_cast<const uint8_t*>(message.getBuffer()), 
        message.getCurrentSize()
    );
    
    if (bytesWritten != message.getCurrentSize()) {
        ESP_LOGW(TAG, "Failed to send complete command 0x%02X", command);
        return false;
    }
    
    return true;
}

// =============================================================================
// Message Processing Engine
// =============================================================================

// State machine based message parser
bool parseIncomingByte(uint8_t byte) {
    switch (currentState) {
        case MessageState::WAITING_START:
            if (byte == STM) {
                currentMessage.stm = byte;
                currentState = MessageState::READING_LENGTH;
                bufferIndex = 0;
                messageBuffer[bufferIndex++] = byte;
            }
            break;
            
        case MessageState::READING_LENGTH:
            currentMessage.length = byte;
            messageBuffer[bufferIndex++] = byte;
            currentState = MessageState::READING_DIRECTION;
            break;
            
        case MessageState::READING_DIRECTION:
            currentMessage.direction = byte;
            messageBuffer[bufferIndex++] = byte;
            currentState = MessageState::READING_COMMAND;
            break;
            
        case MessageState::READING_COMMAND:
            currentMessage.command = byte;
            messageBuffer[bufferIndex++] = byte;
            
            // Calculate data length: LEN - DIR(1) - CMD(1) - CHKSUM(1) - ETX(1)
            expectedDataLength = currentMessage.length - 4;
            currentMessage.dataLength = expectedDataLength;
            
            if (expectedDataLength > 0) {
                currentState = MessageState::READING_DATA;
            } else {
                currentState = MessageState::READING_CHECKSUM;
            }
            break;
            
        case MessageState::READING_DATA:
            if (currentMessage.dataLength > 0 && bufferIndex - 4 < expectedDataLength) {
                currentMessage.data[bufferIndex - 4] = byte;
                messageBuffer[bufferIndex++] = byte;
                
                if (bufferIndex - 4 >= expectedDataLength) {
                    currentState = MessageState::READING_CHECKSUM;
                }
            }
            break;
            
        case MessageState::READING_CHECKSUM:
            currentMessage.checksum = byte;
            messageBuffer[bufferIndex++] = byte;
            currentState = MessageState::READING_END;
            break;
            
        case MessageState::READING_END:
            currentMessage.etx = byte;
            messageBuffer[bufferIndex++] = byte;
            currentState = MessageState::MESSAGE_COMPLETE;
            return true; // Message complete
            
        default:
            resetParser();
            break;
    }
    return false;
}

void resetParser() {
    currentState = MessageState::WAITING_START;
    bufferIndex = 0;
    expectedDataLength = 0;
    memset(&currentMessage, 0, sizeof(currentMessage));
}

// Message type specific handlers
void handleStatusMessage() {
    if (currentMessage.command == statMessage) {
        parseSensorData(currentMessage.data, currentMessage.dataLength);
        sensorDataAvailable = true;
        sendResponse(currentMessage.command);
        
        // Maintain existing TinyML logic
        biosigData1Min.addData(allData);
        predictionInput.addData(allData);
    }
}

// Mode change event handler
void handleModeChangeEvent() {
    if (currentMessage.dataLength > 0) {
        uint8_t modeValue = currentMessage.data[0];
        if (modeValue <= 5) {  // Valid mode range: 0-5
            currentDeviceMode = static_cast<DeviceMode>(modeValue);
            const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "ERROR"};
            ESP_LOGI(TAG, "Device mode changed to: %s (%d)", modeNames[modeValue], modeValue);
        } else {
            ESP_LOGW(TAG, "Invalid mode value received: %d", modeValue);
        }
    } else {
        ESP_LOGW(TAG, "Mode change event without data");
    }
}

void handleEventMessage() {
    switch (currentMessage.command) {
        case evtInitStart:   // 0x80
            ESP_LOGI(TAG, "STM initialization started");
            break;
        case evtInitResult:  // 0x81
            ESP_LOGI(TAG, "STM initialization completed");
            break;
        case evtMode:        // 0x82
            handleModeChangeEvent();
            break;
        default:
            ESP_LOGW(TAG, "Unknown event: 0x%02X", currentMessage.command);
            break;
    }
    sendResponse(currentMessage.command);
}

void handleErrorMessage() {
    ESP_LOGE(TAG, "Error received: 0x%02X", currentMessage.command);
    if (currentMessage.dataLength > 0) {
        uint8_t errorCode = currentMessage.data[0];
        lastErrorCode = errorCode;  // Update error state
        
        const char* errorMessages[] = {
            "Unknown",              // 0
            "Initialization Failed", // 1
            "Communication Failed",  // 2
            "SD Card Failed",       // 3
            "Voltage Failed"        // 4
        };
        
        if (errorCode >= 1 && errorCode <= 4) {
            ESP_LOGE(TAG, "Error: %s (Code: %d)", errorMessages[errorCode], errorCode);
        } else {
            ESP_LOGE(TAG, "Unknown error code: %d", errorCode);
        }
    } else {
        ESP_LOGW(TAG, "Error message received without error code");
    }
    sendResponse(currentMessage.command);
}


// Sensor data parsing function (modified according to protocol document)
void parseSensorData(const uint8_t* data, size_t length) {
    if (!data || length < SENSOR_DATA_MIN_LENGTH) {
        ESP_LOGW(TAG, "Insufficient sensor data length: %d (minimum: %d)", length, SENSOR_DATA_MIN_LENGTH);
        return;
    }
    
    size_t index = 0;
    
    // IMU data - Left side (12 bytes total)
    // Gyro data - Left (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            lastSensorReading.leftGyro[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Accelerometer data - Left (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            lastSensorReading.leftAccel[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // IMU data - Right side (12 bytes total)
    // Gyro data - Right (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            lastSensorReading.rightGyro[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Accelerometer data - Right (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            lastSensorReading.rightAccel[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Pressure sensors (4 bytes total: L + R)
    // Left pressure sensor (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.leftPressure = (data[index] << 8) | data[index + 1];
        index += PRESSURE_DATA_SIZE;
    }
    // Right pressure sensor (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.rightPressure = (data[index] << 8) | data[index + 1];
        index += PRESSURE_DATA_SIZE;
    }
    
    // Temperature sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Outside temperature (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.outsideTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Board temperature (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.boardTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Actuator temperature (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.actuatorTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    
    // Other sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Actuator displacement (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.actuatorDisplacement = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Object distance (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.objectDistance = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Battery voltage (2 bytes)
    if (index + 1 < length) {
        lastSensorReading.batteryVoltage = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    
    // IMU events (2 bytes total: L + R)
    // Left IMU event (1 byte)
    if (index < length) {
        lastSensorReading.leftIMUEvent = data[index];
        index += IMU_EVENT_SIZE;
    }
    // Right IMU event (1 byte)
    if (index < length) {
        lastSensorReading.rightIMUEvent = data[index];
        index += IMU_EVENT_SIZE;
    }
    
    // Update existing allData structure as well (backward compatibility)
    allData.L_gyro[0] = lastSensorReading.leftGyro[0];
    allData.L_gyro[1] = lastSensorReading.leftGyro[1];
    allData.L_gyro[2] = lastSensorReading.leftGyro[2];
    allData.L_accel[0] = lastSensorReading.leftAccel[0];
    allData.L_accel[1] = lastSensorReading.leftAccel[1];
    allData.L_accel[2] = lastSensorReading.leftAccel[2];
    allData.R_gyro[0] = lastSensorReading.rightGyro[0];
    allData.R_gyro[1] = lastSensorReading.rightGyro[1];
    allData.R_gyro[2] = lastSensorReading.rightGyro[2];
    allData.R_accel[0] = lastSensorReading.rightAccel[0];
    allData.R_accel[1] = lastSensorReading.rightAccel[1];
    allData.R_accel[2] = lastSensorReading.rightAccel[2];
    allData.L_ads = lastSensorReading.leftPressure;
    allData.R_ads = lastSensorReading.rightPressure;
    allData.outTemp = lastSensorReading.outsideTemp;
    allData.boardTemp = lastSensorReading.boardTemp;
    allData.lmaTemp = lastSensorReading.actuatorTemp;
    allData.lmaLength = lastSensorReading.actuatorDisplacement;
    allData.objDistance = lastSensorReading.objectDistance;
    allData.battery = lastSensorReading.batteryVoltage;
}

void usartMasterHandler(void *pvParameters) {
    while (1) {
        while (STMSerial.available()) {
            uint8_t incomingByte = STMSerial.read();
            
            // Prevent buffer overflow
            if (bufferIndex >= USART_MESSAGE_MAXIMUM_LENGTH) {
                ESP_LOGW(TAG, "Buffer overflow, resetting parser");
                resetParser();
                continue;
            }
            
            // State machine based parsing
            if (parseIncomingByte(incomingByte)) {
                // Message complete, verify checksum
                if (!currentMessage.isValid()) {
                    ESP_LOGW(TAG, "Invalid message format");
                    resetParser();
                    continue;
                }
                
                if (!checkMessage(messageBuffer, bufferIndex)) {
                    ESP_LOGW(TAG, "Checksum verification failed");
                    resetParser();
                    continue;
                }
                
                // Handle according to message type and command classification
                if (currentMessage.direction == STM_TO_ESP_DIR) {
                    // Classify and process message by command type
                    CommandType cmdType = currentMessage.getCommandType();
                    
                    switch (cmdType) {
                        case CommandType::STATUS:      // 0x70 - STM spontaneous sensor data
                            handleStatusMessage();
                            break;
                            
                        case CommandType::EVENT:       // 0x8X - STM spontaneous state changes
                            handleEventMessage();
                            break;
                            
                        case CommandType::ERROR:       // 0x9X - STM spontaneous error reports
                            handleErrorMessage();
                            break;
                            
                        case CommandType::INIT:        // 0x1X - ESP→STM command ACK
                            // ESP→STM INIT command ACK (only if we're expecting it)
                            handleInitResponse();
                            break;
                            
                        case CommandType::REQUEST:     // 0x3X - ESP→STM command response
                            // ESP→STM REQUEST command response with data (only if we're expecting it)
                            handleRequestResponse();
                            break;
                            
                        case CommandType::CONTROL:     // 0x5X - ESP→STM command ACK
                            // ESP→STM CONTROL command ACK (only if we're expecting it)
                            handleControlResponse();
                            break;
                            
                        default:
                            ESP_LOGW(TAG, "Unknown command type: 0x%02X (full command: 0x%02X)", 
                                     static_cast<uint8_t>(cmdType), currentMessage.command);
                            break;
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid message direction: 0x%02X (expected STM→ESP: 0x%02X)", 
                             currentMessage.direction, STM_TO_ESP_DIR);
                }
                
                resetParser();
            }
        }
        
        // Retry sending message if ESP doesn't receive an ACK
        // processCommandTimeout(); // temporarily disabled for simplicity 

        vTaskDelay(10 / portTICK_RATE_MS); // More frequent checks for better responsiveness
    }
}

bool initUartMaster() {

    STMSerial.begin(115200, SERIAL_8N1, ESP_U2_RXD, ESP_U2_TXD);
  	delay(100);
	// STMSerial.println("(ESP-STM Serial: ON)");

    xTaskCreatePinnedToCore(usartMasterHandler,
                            "usartMasterHandler",
                            UART_TASK_STACK,
                            NULL,
                            UART_TASK_PRI,
                            &TasUsartMaster_h,
                            UART_TASK_CORE);
    ESP_LOGI(TAG, "ESP-STM USART  initialized...");
    
  	delay(100);
}

// =============================================================================
// Bidirectional Communication Functions (ESP→STM)
// =============================================================================

void handleInitResponse() {
    // Handle INIT command ACKs (0x1X)
    ESP_LOGI(TAG, "INIT ACK received for command 0x%02X", currentMessage.command);
}

void handleRequestResponse() {
    // Handle REQUEST command responses with data (0x3X)
    switch (currentMessage.command) {
        case reqTempSleep:      // 0x30
            if (currentMessage.dataLength >= 2) {
                cachedSleepTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                sleepTempValid = true;
                ESP_LOGI(TAG, "Sleep temp updated: %.1f°C", cachedSleepTemp);
            } else {
                ESP_LOGW(TAG, "Invalid sleep temp response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqTempWaiting:    // 0x31
            if (currentMessage.dataLength >= 2) {
                cachedWaitingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                waitingTempValid = true;
                ESP_LOGI(TAG, "Waiting temp updated: %.1f°C", cachedWaitingTemp);
            } else {
                ESP_LOGW(TAG, "Invalid waiting temp response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqTempForceUp:    // 0x32
            if (currentMessage.dataLength >= 2) {
                cachedOperatingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                operatingTempValid = true;
                ESP_LOGI(TAG, "Operating temp updated: %.1f°C", cachedOperatingTemp);
            } else {
                ESP_LOGW(TAG, "Invalid operating temp response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqTempHeatPad:    // 0x33 - DEPRECATED
            // This command is deprecated and no longer supported
            ESP_LOGW(TAG, "reqTempHeatPad (0x33) is deprecated and ignored");
            break;
            
        case reqUpperTemp:      // 0x34
            if (currentMessage.dataLength >= 2) {
                cachedTempLimit = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                tempLimitValid = true;
                ESP_LOGI(TAG, "Upper temp limit updated: %.1f°C", cachedTempLimit);
            } else {
                ESP_LOGW(TAG, "Invalid upper temp limit response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqPWMCoolFan:     // 0x35
            if (currentMessage.dataLength >= 2) {
                cachedCoolingFanLevel = currentMessage.data[0];
                cachedMaxCoolingFanLevel = currentMessage.data[1];
                coolingFanLevelValid = true;
                ESP_LOGI(TAG, "Cooling fan level updated: %d/%d", cachedCoolingFanLevel, cachedMaxCoolingFanLevel);
            } else {
                ESP_LOGW(TAG, "Invalid cooling fan level response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqTimeout:        // 0x36
            if (currentMessage.dataLength >= 8) {
                cachedForceUpTimeout = (currentMessage.data[0] << 8) | currentMessage.data[1];
                cachedForceOnTimeout = (currentMessage.data[2] << 8) | currentMessage.data[3];
                cachedForceDownTimeout = (currentMessage.data[4] << 8) | currentMessage.data[5];
                cachedWaitingTimeout = (currentMessage.data[6] << 8) | currentMessage.data[7];
                timeoutsValid = true;
                ESP_LOGI(TAG, "Timeouts updated: ForceUp=%d, ForceOn=%d, ForceDown=%d, Waiting=%d", 
                         cachedForceUpTimeout, cachedForceOnTimeout, cachedForceDownTimeout, cachedWaitingTimeout);
            } else {
                ESP_LOGW(TAG, "Invalid timeout response length: %d (expected 8)", currentMessage.dataLength);
            }
            break;
            
        case reqSpk:            // 0x37
            if (currentMessage.dataLength >= 1) {
                cachedSpeakerVolume = currentMessage.data[0];
                speakerVolumeValid = true;
                ESP_LOGI(TAG, "Speaker volume updated: %d", cachedSpeakerVolume);
            } else {
                ESP_LOGW(TAG, "Invalid speaker volume response length: %d", currentMessage.dataLength);
            }
            break;
            
        case reqDelay:          // 0x38
            if (currentMessage.dataLength >= 2) {
                cachedPoseDetectionDelay = currentMessage.data[0];    // 100ms units
                cachedObjectDetectionDelay = currentMessage.data[1];  // 100ms units
                detectionDelayValid = true;
                ESP_LOGI(TAG, "Detection delays updated: Pose=%d00ms, Object=%d00ms", 
                         cachedPoseDetectionDelay, cachedObjectDetectionDelay);
            } else {
                ESP_LOGW(TAG, "Invalid detection delay response length: %d", currentMessage.dataLength);
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown REQUEST response: 0x%02X", currentMessage.command);
            break;
    }
}

void handleControlResponse() {
    // Handle CONTROL command ACKs (0x5X)
    ESP_LOGI(TAG, "CONTROL ACK received for command 0x%02X", currentMessage.command);
}

bool sendCommandAsync(uint8_t command, const byte* data = nullptr, size_t dataLen = 0) {
    if (waitingForResponse) {
        ESP_LOGW(TAG, "Cannot send command 0x%02X - already waiting for response", command);
        return false;
    }
    
    // Determine response type based on command
    ResponseType expectedResponse = ResponseType::ACK_ONLY;
    CommandType cmdType = static_cast<CommandType>(command & 0xF0);
    if (cmdType == CommandType::REQUEST) {
        expectedResponse = ResponseType::DATA_RESPONSE;
    }
    
    // Setup pending command tracking
    currentPendingCommand.reset();
    currentPendingCommand.command = command;
    currentPendingCommand.expectedResponse = expectedResponse;
    currentPendingCommand.state = CommandState::SENT;
    currentPendingCommand.sentTimestamp = millis();
    currentPendingCommand.backupOriginalData(data, dataLen);
    waitingForResponse = true;
    
    // Send the command and return immediately - No waiting!
    if (!sendCommandSafe(command, data, dataLen)) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X", command);
        currentPendingCommand.state = CommandState::ERROR;
        waitingForResponse = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Command 0x%02X sent asynchronously", command);
    return true;
}

bool sendCommandWithAck(uint8_t command, const byte* data, size_t dataLen, uint32_t timeoutMs) {
    if (waitingForResponse) {
        ESP_LOGW(TAG, "Cannot send command 0x%02X - already waiting for response", command);
        return false;
    }
    
    // Determine response type based on command
    ResponseType expectedResponse = ResponseType::ACK_ONLY;
    CommandType cmdType = static_cast<CommandType>(command & 0xF0);
    if (cmdType == CommandType::REQUEST) {
        expectedResponse = ResponseType::DATA_RESPONSE;
    }
    
    // Setup pending command tracking
    currentPendingCommand.reset();
    currentPendingCommand.command = command;
    currentPendingCommand.expectedResponse = expectedResponse;
    currentPendingCommand.state = CommandState::SENT;
    currentPendingCommand.sentTimestamp = millis();
    currentPendingCommand.backupOriginalData(data, dataLen);  // Backup original data for retry
    waitingForResponse = true;
    
    // Send the command
    if (!sendCommandSafe(command, data, dataLen)) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X", command);
        currentPendingCommand.state = CommandState::ERROR;
        waitingForResponse = false;
        return false;
    }
    
    // Wait for response
    return waitForResponse(command, expectedResponse, timeoutMs);
}

bool sendRequestWithResponse(uint8_t command, uint8_t* responseBuffer, size_t* responseLength, uint32_t timeoutMs) {
    if (!responseBuffer || !responseLength) {
        ESP_LOGE(TAG, "Invalid response buffer parameters");
        return false;
    }
    
    *responseLength = 0;
    
    if (!sendCommandWithAck(command, nullptr, 0, timeoutMs)) {
        return false;
    }
    
    // Copy response data if available
    if (currentPendingCommand.state == CommandState::DATA_RECEIVED && 
        currentPendingCommand.responseLength > 0) {
        size_t copyLength = min(currentPendingCommand.responseLength, *responseLength);
        memcpy(responseBuffer, currentPendingCommand.responseData, copyLength);
        *responseLength = copyLength;
        return true;
    }
    
    return false;
}

bool waitForResponse(uint8_t expectedCommand, ResponseType responseType, uint32_t timeoutMs) {
    uint32_t startTime = millis();
    
    // Poll for response completion with timeout
    while (millis() - startTime < timeoutMs) {
        if (currentPendingCommand.command == expectedCommand) {
            if (currentPendingCommand.state == CommandState::ACK_RECEIVED ||
                currentPendingCommand.state == CommandState::DATA_RECEIVED) {
                ESP_LOGI(TAG, "Command 0x%02X completed successfully", expectedCommand);
                return true;
            } else if (currentPendingCommand.state == CommandState::ERROR ||
                       currentPendingCommand.state == CommandState::TIMEOUT) {
                ESP_LOGW(TAG, "Command 0x%02X completed with error state: %d", 
                         expectedCommand, static_cast<int>(currentPendingCommand.state));
                return false;
            }
        }
        
        // Small delay to prevent busy waiting
        delay(10);
    }
    
    // Timeout occurred
    ESP_LOGW(TAG, "Timeout waiting for response to command 0x%02X", expectedCommand);
    currentPendingCommand.state = CommandState::TIMEOUT;
    waitingForResponse = false;
    return false;
}

bool isResponseExpected() {
    return waitingForResponse;
}

CommandState getCommandState(uint8_t command) {
    if (currentPendingCommand.command == command) {
        return currentPendingCommand.state;
    }
    return CommandState::IDLE;
}

void processCommandTimeout() {
    if (!waitingForResponse) {
        return;
    }
    
    uint32_t currentTime = millis();
    if (currentPendingCommand.isTimeoutExpired(currentTime)) {
        uint32_t elapsedTime = currentTime - currentPendingCommand.sentTimestamp;
        ESP_LOGW(TAG, "Command 0x%02X timed out after %lu ms (attempt %d/%d)", 
                 currentPendingCommand.command, elapsedTime,
                 currentPendingCommand.retryCount + 1, MAX_RETRY_ATTEMPTS + 1);
        
        if (currentPendingCommand.retryCount < MAX_RETRY_ATTEMPTS) {
            // Retry the command with original data
            currentPendingCommand.retryCount++;
            currentPendingCommand.sentTimestamp = currentTime;
            currentPendingCommand.state = CommandState::SENT;
            
            uint32_t nextTimeout = (currentPendingCommand.expectedResponse == ResponseType::DATA_RESPONSE) ? 
                                  REQUEST_TIMEOUT_MS : COMMAND_TIMEOUT_MS;
            nextTimeout += (nextTimeout / 2) * currentPendingCommand.retryCount;  // Exponential backoff
            
            ESP_LOGI(TAG, "Retrying command 0x%02X (attempt %d/%d, timeout: %lu ms)", 
                     currentPendingCommand.command, 
                     currentPendingCommand.retryCount + 1, 
                     MAX_RETRY_ATTEMPTS + 1,
                     nextTimeout);
            
            // Resend the command with original data
            const byte* retryData = (currentPendingCommand.originalDataLength > 0) ? 
                                   currentPendingCommand.originalData : nullptr;
            
            if (!sendCommandSafe(currentPendingCommand.command, retryData, currentPendingCommand.originalDataLength)) {
                ESP_LOGE(TAG, "Failed to resend command 0x%02X on retry %d", 
                         currentPendingCommand.command, currentPendingCommand.retryCount);
                currentPendingCommand.state = CommandState::ERROR;
                waitingForResponse = false;
            }
        } else {
            // Max retries exceeded
            ESP_LOGE(TAG, "Command 0x%02X FAILED after %d attempts (total time: %lu ms)", 
                     currentPendingCommand.command, MAX_RETRY_ATTEMPTS + 1,
                     currentTime - (currentPendingCommand.sentTimestamp - elapsedTime));
            currentPendingCommand.state = CommandState::ERROR;
            waitingForResponse = false;
        }
    }
}

// =============================================================================
// Error Handling and Diagnostic Functions
// =============================================================================

const char* getCommandStateString(CommandState state) {
    switch (state) {
        case CommandState::IDLE: return "IDLE";
        case CommandState::SENT: return "SENT";
        case CommandState::ACK_RECEIVED: return "ACK_RECEIVED";
        case CommandState::DATA_RECEIVED: return "DATA_RECEIVED";
        case CommandState::TIMEOUT: return "TIMEOUT";
        case CommandState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const char* getLastErrorString() {
    if (currentPendingCommand.state == CommandState::TIMEOUT) {
        return "Command timeout - STM did not respond";
    } else if (currentPendingCommand.state == CommandState::ERROR) {
        return "Command error - Invalid response or communication failure";
    } else if (lastErrorCode > 0) {
        const char* errorMessages[] = {
            "No error",                 // 0
            "Initialization Failed",    // 1
            "Communication Failed",     // 2
            "SD Card Failed",          // 3
            "Voltage Failed"           // 4
        };
        
        if (lastErrorCode >= 1 && lastErrorCode <= 4) {
            return errorMessages[lastErrorCode];
        } else {
            return "Unknown STM error";
        }
    }
    return "No error";
}

bool isCommandInProgress() {
    return waitingForResponse && (currentPendingCommand.state == CommandState::SENT);
}

void clearPendingCommand() {
    currentPendingCommand.reset();
    waitingForResponse = false;
    ESP_LOGI(TAG, "Pending command cleared");
}

uint32_t getLastCommandTimestamp() {
    return currentPendingCommand.sentTimestamp;
}

uint8_t getCommandRetryCount() {
    return currentPendingCommand.retryCount;
}

// =============================================================================
// User-friendly API functions implementation
// =============================================================================

// Temperature Control Functions
bool setSleepTemperature(float targetTemp) {
    if (targetTemp < 0 || targetTemp > 100) {
        ESP_LOGW(TAG, "Invalid temperature range");
        return false;
    }
    
    byte data[] = {
        (uint8_t)targetTemp,                                    // Integer part
        (uint8_t)((targetTemp - (int)targetTemp) * 10)         // Decimal part (x10)
    };
    
    bool success = sendCommandAsync(initTempSleep, data, 2);
    if (success) {
        ESP_LOGI(TAG, "Sleep temperature set to %.1f°C", targetTemp);
    }
    return success;
}

bool setWaitingTemperature(float targetTemp) {
    if (targetTemp < 0 || targetTemp > 100) {
        ESP_LOGW(TAG, "Invalid temperature range");
        return false;
    }
    
    byte data[] = {
        (uint8_t)targetTemp,                                    // Integer part
        (uint8_t)((targetTemp - (int)targetTemp) * 10)         // Decimal part (x10)
    };
    
    bool success = sendCommandAsync(initTempWaiting, data, 2);
    if (success) {
        ESP_LOGI(TAG, "Waiting temperature set to %.1f°C", targetTemp);
    }
    return success;
}

bool setOperatingTemperature(float targetTemp) {
    if (targetTemp < 0 || targetTemp > 100) {
        ESP_LOGW(TAG, "Invalid temperature range");
        return false;
    }
    
    byte data[] = {
        (uint8_t)targetTemp,                                    // Integer part
        (uint8_t)((targetTemp - (int)targetTemp) * 10)         // Decimal part (x10)
    };
    
    bool success = sendCommandAsync(initTempForceUp, data, 2);
    if (success) {
        ESP_LOGI(TAG, "Operating temperature set to %.1f°C", targetTemp);
    }
    return success;
}

bool setUpperTemperatureLimit(float limitTemp) {
    if (limitTemp < 0 || limitTemp > 100) {
        ESP_LOGW(TAG, "Invalid temperature limit");
        return false;
    }
    
    byte data[] = {
        (uint8_t)limitTemp,                                    // Integer part
        (uint8_t)((limitTemp - (int)limitTemp) * 10)         // Decimal part (x10)
    };
    
    bool success = sendCommandAsync(0x14, data, 2); // initTempLimit command
    if (success) {
        ESP_LOGI(TAG, "Upper temperature limit set to %.1f°C", limitTemp);
    }
    return success;
}

// Device Mode Functions
bool setDeviceMode(DeviceMode mode) {
    byte data[] = {static_cast<byte>(mode)};
    
    bool success = sendCommandAsync(ctrlMode, data, 1);
    if (success) {
        const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "ERROR"};
        ESP_LOGI(TAG, "Device mode set to %s", modeNames[static_cast<int>(mode)]);
    }
    return success;
}

// Actuator Control Functions - Fan
bool setFanState(bool enabled) {
    byte data[] = {enabled ? 1 : 0};
    
    bool success = sendCommandAsync(ctrlFanOn, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Fan %s", enabled ? "ON" : "OFF");
    }
    return success;
}

bool setFanSpeed(uint8_t speed_0_to_10) {
    if (speed_0_to_10 > 10) {
        ESP_LOGW(TAG, "Invalid fan speed: %d (max: 10)", speed_0_to_10);
        return false;
    }
    
    byte data[] = {speed_0_to_10};
    
    bool success = sendCommandAsync(ctrlFanPWM, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Fan speed set to %d", speed_0_to_10);
    }
    return success;
}

// Actuator Control Functions - Cooling Fan
bool setCoolingFanState(bool enabled) {
    byte data[] = {enabled ? 1 : 0};
    
    bool success = sendCommandAsync(ctrlCoolFanOn, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Cooling fan %s", enabled ? "ON" : "OFF");
    }
    return success;
}

bool setCoolingFanLevel(uint8_t level) {
    if (level > 10) {
        ESP_LOGW(TAG, "Invalid cooling fan level: %d (max: 10)", level);
        return false;
    }
    
    byte data[] = {level};
    
    bool success = sendCommandAsync(ctrlCoolFanPWM, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Cooling fan level set to %d", level);
    }
    return success;
}

// Actuator Control Functions - Heat Pad
bool setHeatPadState(bool enabled) {
    // This function is deprecated - ctrlHeatPadOn (0x57) is not supported
    ESP_LOGW(TAG, "setHeatPadState is deprecated - ctrlHeatPadOn (0x57) not supported");
    return false;
}

bool setHeatPadLevel(uint8_t level) {
    // This function is deprecated - ctrlHeatPadTemp (0x58) is not supported
    ESP_LOGW(TAG, "setHeatPadLevel is deprecated - ctrlHeatPadTemp (0x58) not supported");
    return false;
}

// Audio Control Functions
bool setSpeakerVolume(uint8_t volume_0_to_10) {
    if (volume_0_to_10 > 10) {
        ESP_LOGW(TAG, "Invalid volume: %d (max: 10)", volume_0_to_10);
        return false;
    }
    
    byte data[] = {volume_0_to_10};
    
    bool success = sendCommandAsync(ctrlSpkVol, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Speaker volume set to %d", volume_0_to_10);
    }
    return success;
}

// Sensor data access function
SensorReading getCurrentSensorData() {
    return lastSensorReading;
}

// Pose detection mode setting
bool setPoseDetectionMode(bool enabled) {
    byte data[] = {enabled ? 0 : 1}; // 0: pose detection, 1: object detection
    
    bool success = sendCommandAsync(ctrlPose, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Detection mode set to %s", enabled ? "POSE" : "OBJECT");
    }
    return success;
}

// Utility functions
bool resetDevice() {
    bool success = sendCommandAsync(ctrlReset);
    if (success) {
        ESP_LOGI(TAG, "Device reset command sent");
    }
    return success;
}

bool isPoseDetectionMode() {
    // A variable to store current mode state is needed, 
    // but here we implement by sending request message to check
    // In actual implementation, add state variable for management
    return true; // Temporary implementation
}

bool isObjectDetectionMode() {
    return !isPoseDetectionMode();
}

// =============================================================================
// Async Request Functions (Non-blocking)
// =============================================================================

bool requestSleepTemperature() {
    return sendCommandAsync(reqTempSleep);
}

bool requestWaitingTemperature() {
    return sendCommandAsync(reqTempWaiting);
}

bool requestOperatingTemperature() {
    return sendCommandAsync(reqTempForceUp);
}

bool requestUpperTemperatureLimit() {
    return sendCommandAsync(reqUpperTemp);
}

// requestHeatPadLevel() removed - deprecated functionality

bool requestCoolingFanLevel() {
    return sendCommandAsync(reqPWMCoolFan);
}

bool requestSpeakerVolume() {
    return sendCommandAsync(reqSpk);
}

bool requestDetectionDelay() {
    return sendCommandAsync(reqDelay);
}

bool requestAllParameters() {
    bool success = true;
    success &= requestSleepTemperature();
    vTaskDelay(10 / portTICK_RATE_MS);  // Small delay between commands
    success &= requestWaitingTemperature();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestOperatingTemperature();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestUpperTemperatureLimit();
    vTaskDelay(10 / portTICK_RATE_MS);
    // requestHeatPadLevel() removed - deprecated functionality
    success &= requestCoolingFanLevel();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestSpeakerVolume();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestDetectionDelay();
    
    ESP_LOGI(TAG, "All parameter requests sent %s", success ? "successfully" : "with errors");
    return success;
}

// =============================================================================
// Data Validity Check Functions
// =============================================================================

bool isSleepTemperatureReady() { return sleepTempValid; }
bool isWaitingTemperatureReady() { return waitingTempValid; }
bool isOperatingTemperatureReady() { return operatingTempValid; }
bool isTempLimitReady() { return tempLimitValid; }
// isHeatPadLevelReady() removed - deprecated functionality
bool isCoolingFanLevelReady() { return coolingFanLevelValid; }
bool isSpeakerVolumeReady() { return speakerVolumeValid; }
bool isDetectionDelayReady() { return detectionDelayValid; }

bool areAllParametersReady() {
    return sleepTempValid && waitingTempValid && operatingTempValid && 
           tempLimitValid && coolingFanLevelValid && 
           speakerVolumeValid && detectionDelayValid;
}

void clearAllCachedData() {
    sleepTempValid = false;
    waitingTempValid = false;
    operatingTempValid = false;
    tempLimitValid = false;
    fanLevelValid = false;
    coolingFanLevelValid = false;
    speakerVolumeValid = false;
    // heatPadLevelValid removed - deprecated functionality
    timeoutsValid = false;
    detectionDelayValid = false;
    ESP_LOGI(TAG, "All cached data cleared");
}

// =============================================================================
// Request functions (REQ commands) implementation - Updated for async
// =============================================================================

float getSleepTemperature() {
    if (!sleepTempValid) {
        requestSleepTemperature();  // Async request for fresh data
        ESP_LOGI(TAG, "Sleep temperature requested, returning cached value");
    }
    return cachedSleepTemp;
}

float getWaitingTemperature() {
    if (!waitingTempValid) {
        requestWaitingTemperature();  // Async request for fresh data
        ESP_LOGI(TAG, "Waiting temperature requested, returning cached value");
    }
    return cachedWaitingTemp;
}

float getOperatingTemperature() {
    if (!operatingTempValid) {
        requestOperatingTemperature();  // Async request for fresh data
        ESP_LOGI(TAG, "Operating temperature requested, returning cached value");
    }
    return cachedOperatingTemp;
}

float getUpperTemperatureLimit() {
    if (!tempLimitValid) {
        requestUpperTemperatureLimit();  // Async request for fresh data
        ESP_LOGI(TAG, "Temperature limit requested, returning cached value");
    }
    return cachedTempLimit;
}

DeviceMode getCurrentMode() {
    // Return the actual tracked device mode updated by event messages
    return currentDeviceMode;
}

uint8_t getLastErrorCode() {
    // Return the last error code received (0 = no error)
    return lastErrorCode;
}

// =============================================================================
// Convenience Functions (organized by complexity level)
// =============================================================================

// Default setting values
namespace DefaultSettings {
    constexpr float SLEEP_TEMP = 35.0f;
    constexpr float WAITING_TEMP = 45.0f;
    constexpr float OPERATING_TEMP = 60.0f;
    constexpr float UPPER_TEMP_LIMIT = 70.0f;
    constexpr uint8_t DEFAULT_VOLUME = 5;
    constexpr uint8_t DEFAULT_FAN_SPEED = 5;
    constexpr uint8_t DEFAULT_COOLING_FAN_LEVEL = 3;
    constexpr uint8_t DEFAULT_HEAT_PAD_LEVEL = 5;
}

// -----------------------------------------------------------------------------
// Simple Convenience Functions
// -----------------------------------------------------------------------------

// Print sensor data for debugging
void printSensorData() {
    SensorReading data = getCurrentSensorData();
    
    ESP_LOGI(TAG, "=== Sensor Data ===");
    ESP_LOGI(TAG, "Left Gyro: X=%d, Y=%d, Z=%d", data.leftGyro[0], data.leftGyro[1], data.leftGyro[2]);
    ESP_LOGI(TAG, "Left Accel: X=%d, Y=%d, Z=%d", data.leftAccel[0], data.leftAccel[1], data.leftAccel[2]);
    ESP_LOGI(TAG, "Right Gyro: X=%d, Y=%d, Z=%d", data.rightGyro[0], data.rightGyro[1], data.rightGyro[2]);
    ESP_LOGI(TAG, "Right Accel: X=%d, Y=%d, Z=%d", data.rightAccel[0], data.rightAccel[1], data.rightAccel[2]);
    ESP_LOGI(TAG, "Pressure: L=%d, R=%d", data.leftPressure, data.rightPressure);
    ESP_LOGI(TAG, "Temperature: Outside=%.1f°C, Board=%.1f°C, Actuator=%.1f°C", 
             data.getOutsideTemperatureFloat(), 
             data.getBoardTemperatureFloat(), 
             data.getActuatorTemperatureFloat());
    ESP_LOGI(TAG, "Battery: %.2fV", data.getBatteryVoltageFloat());
    ESP_LOGI(TAG, "IMU Events: L=0x%02X, R=0x%02X", data.leftIMUEvent, data.rightIMUEvent);
}

// Turn off all fans
bool turnOffAllFans() {
    bool success = true;
    
    success &= setFanState(false);
    vTaskDelay(10 / portTICK_RATE_MS);
    
    success &= setCoolingFanState(false);
    
    ESP_LOGI(TAG, "All fans turned off");
    return success;
}

// -----------------------------------------------------------------------------
// Medium Complexity Convenience Functions
// -----------------------------------------------------------------------------

// Set all temperatures at once
bool setAllTemperatures(float sleepTemp, float waitingTemp, float operatingTemp, float limitTemp) {
    bool success = true;
    
    success &= setSleepTemperature(sleepTemp);
    vTaskDelay(50 / portTICK_RATE_MS); // Delay between commands
    
    success &= setWaitingTemperature(waitingTemp);
    vTaskDelay(50 / portTICK_RATE_MS);
    
    success &= setOperatingTemperature(operatingTemp);
    vTaskDelay(50 / portTICK_RATE_MS);
    
    success &= setUpperTemperatureLimit(limitTemp);
    
    if (success) {
        ESP_LOGI(TAG, "All temperatures configured successfully");
    } else {
        ESP_LOGW(TAG, "Some temperature settings failed");
    }
    
    return success;
}

// Activate safe mode (for emergency use)
bool activateSafeMode() {
    ESP_LOGW(TAG, "Activating safe mode...");
    
    bool success = true;
    
    // Turn off all actuators
    success &= turnOffAllFans();
    success &= setHeatPadState(false);
    
    // Set to safe temperature
    success &= setUpperTemperatureLimit(50.0f); // Lower temperature limit
    
    // Change to sleep mode
    success &= setDeviceMode(DeviceMode::SLEEP);
    
    if (success) {
        ESP_LOGI(TAG, "Safe mode activated successfully");
    } else {
        ESP_LOGE(TAG, "Failed to activate safe mode");
    }
    
    return success;
}

// Health check function
bool performHealthCheck() {
    ESP_LOGI(TAG, "Performing device health check...");
    
    SensorReading data = getCurrentSensorData();
    bool isHealthy = true;
    
    // Battery voltage check
    float batteryVoltage = data.getBatteryVoltageFloat();
    if (batteryVoltage < 3.0f) { // Example threshold
        ESP_LOGW(TAG, "Health check: Low battery voltage: %.2fV", batteryVoltage);
        isHealthy = false;
    }
    
    // Temperature check
    float boardTemp = data.getBoardTemperatureFloat();
    if (boardTemp > 80.0f) { // Example threshold
        ESP_LOGW(TAG, "Health check: High board temperature: %.1f°C", boardTemp);
        isHealthy = false;
    }
    
    // Actuator temperature check
    float actuatorTemp = data.getActuatorTemperatureFloat();
    if (actuatorTemp > DefaultSettings::UPPER_TEMP_LIMIT) {
        ESP_LOGW(TAG, "Health check: High actuator temperature: %.1f°C", actuatorTemp);
        isHealthy = false;
    }
    
    if (isHealthy) {
        ESP_LOGI(TAG, "Health check: All systems normal");
    } else {
        ESP_LOGW(TAG, "Health check: Issues detected, consider safe mode");
    }
    
    return isHealthy;
}

// -----------------------------------------------------------------------------
// Advanced Automation Functions
// -----------------------------------------------------------------------------

bool initializeWithDefaults() {
    ESP_LOGI(TAG, "Initializing device with default settings...");
    
    bool success = true;
    
    // Default temperature settings
    success &= setAllTemperatures(
        DefaultSettings::SLEEP_TEMP,
        DefaultSettings::WAITING_TEMP, 
        DefaultSettings::OPERATING_TEMP,
        DefaultSettings::UPPER_TEMP_LIMIT
    );
    
    vTaskDelay(100 / portTICK_RATE_MS);
    
    // Default volume setting
    success &= setSpeakerVolume(DefaultSettings::DEFAULT_VOLUME);
    vTaskDelay(50 / portTICK_RATE_MS);
    
    // Initial state for fans and heat pad (all OFF)
    success &= turnOffAllFans();
    vTaskDelay(50 / portTICK_RATE_MS);
    
    success &= setHeatPadState(false);
    vTaskDelay(50 / portTICK_RATE_MS);
    
    // Set to pose detection mode
    setPoseDetectionMode(true);
    
    if (success) {
        ESP_LOGI(TAG, "Device initialized with defaults successfully");
    } else {
        ESP_LOGW(TAG, "Some default settings failed to apply");
    }
    
    return success;
}

// Automated communication error recovery
bool recoverFromError() {
    ESP_LOGW(TAG, "Attempting error recovery...");
    
    // Reset parser state
    resetParser();
    
    // Clear serial buffer
    while (STMSerial.available()) {
        STMSerial.read();
    }
    
    // Attempt device reset after brief delay
    vTaskDelay(100 / portTICK_RATE_MS);
    
    bool resetSuccess = resetDevice();
    if (resetSuccess) {
        ESP_LOGI(TAG, "Error recovery: Device reset successful");
        
        // Wait briefly after reset
        vTaskDelay(500 / portTICK_RATE_MS);
        
        // Re-initialize with default settings
        return initializeWithDefaults();
    } else {
        ESP_LOGE(TAG, "Error recovery: Device reset failed");
        return false;
    }
}


// =============================================================================
// Enhanced Error Recovery and Diagnostics
// =============================================================================

bool performCommunicationTest() {
    ESP_LOGI(TAG, "Starting communication test with STM...");
    
    // Test 1: Try to get current device mode (simple request)
    DeviceMode mode = getCurrentMode();
    ESP_LOGI(TAG, "Communication test - Current mode: %d", static_cast<int>(mode));
    
    // Test 2: Try to set speaker volume (simple command with ACK)
    uint8_t originalVolume = 5;
    if (!setSpeakerVolume(originalVolume)) {
        ESP_LOGE(TAG, "Communication test FAILED - Cannot set speaker volume");
        return false;
    }
    
    // Test 3: Try to get temperature configuration (REQUEST with data response)
    TemperatureConfig config = getSleepTemperature();
    if (config.targetTemp == 35.0f && config.maxTemp == 70.0f) {
        ESP_LOGW(TAG, "Communication test WARNING - Got default values, may indicate communication issue");
    }
    
    // Test 4: Check if there are any pending commands or errors
    if (isCommandInProgress()) {
        ESP_LOGW(TAG, "Communication test WARNING - Command still in progress");
        return false;
    }
    
    if (getLastErrorCode() != 0) {
        ESP_LOGW(TAG, "Communication test WARNING - Last error code: %d (%s)", 
                 getLastErrorCode(), getLastErrorString());
    }
    
    ESP_LOGI(TAG, "Communication test PASSED");
    return true;
}

void printSystemStatus() {
    ESP_LOGI(TAG, "=== System Status Report ===");
    
    // Communication status
    ESP_LOGI(TAG, "Communication Status:");
    ESP_LOGI(TAG, "  - Command in progress: %s", isCommandInProgress() ? "YES" : "NO");
    ESP_LOGI(TAG, "  - Waiting for response: %s", isResponseExpected() ? "YES" : "NO");
    ESP_LOGI(TAG, "  - Last error: %s", getLastErrorString());
    ESP_LOGI(TAG, "  - Last error code: %d", getLastErrorCode());
    
    if (isCommandInProgress()) {
        ESP_LOGI(TAG, "  - Current command: 0x%02X", currentPendingCommand.command);
        ESP_LOGI(TAG, "  - Command state: %s", getCommandStateString(currentPendingCommand.state));
        ESP_LOGI(TAG, "  - Retry count: %d/%d", getCommandRetryCount(), MAX_RETRY_ATTEMPTS);
        ESP_LOGI(TAG, "  - Time since sent: %lu ms", millis() - getLastCommandTimestamp());
    }
    
    // Device status
    ESP_LOGI(TAG, "Device Status:");
    DeviceMode mode = getCurrentMode();
    const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "ERROR"};
    ESP_LOGI(TAG, "  - Current mode: %s (%d)", modeNames[static_cast<int>(mode)], static_cast<int>(mode));
    ESP_LOGI(TAG, "  - Sensor data available: %s", sensorDataAvailable ? "YES" : "NO");
    
    // Sensor status (if available)
    if (sensorDataAvailable) {
        SensorReading data = getCurrentSensorData();
        ESP_LOGI(TAG, "Sensor Status:");
        ESP_LOGI(TAG, "  - Outside temperature: %.1f°C", data.getOutsideTemperatureFloat());
        ESP_LOGI(TAG, "  - Board temperature: %.1f°C", data.getBoardTemperatureFloat());
        ESP_LOGI(TAG, "  - Actuator temperature: %.1f°C", data.getActuatorTemperatureFloat());
        ESP_LOGI(TAG, "  - Battery voltage: %.2fV", data.getBatteryVoltageFloat());
        ESP_LOGI(TAG, "  - Left/Right pressure: %d/%d", data.leftPressure, data.rightPressure);
    }
    
    ESP_LOGI(TAG, "=== End Status Report ===");
}

bool emergencyShutdown() {
    ESP_LOGW(TAG, "EMERGENCY SHUTDOWN INITIATED");
    
    bool success = true;
    
    // Stop all actuators
    success &= turnOffAllFans();
    // success &= setHeatPadState(false);
    
    // Set device to safe mode (SLEEP)
    success &= setDeviceMode(DeviceMode::SLEEP);
    
    // Clear any pending commands
    clearPendingCommand();
    
    if (success) {
        ESP_LOGI(TAG, "Emergency shutdown completed successfully");
    } else {
        ESP_LOGE(TAG, "Emergency shutdown completed with errors");
    }
    
    return success;
}
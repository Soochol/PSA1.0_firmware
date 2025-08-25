/**
 * @file uartMaster.cpp
 * @brief Implementation of ESP32-STM32 UART Communication Protocol
 * 
 * This file implements the complete UART communication system between ESP32 and STM32
 * microcontrollers for the PSA (Postural Stability Assistant) device. It provides
 * bidirectional communication with asynchronous command/response handling, message
 * parsing, error recovery, and system health monitoring.
 * 
 * Key Implementation Features:
 * - State machine-based message parser for reliable frame synchronization
 * - Asynchronous command queue with timeout and retry mechanism
 * - Real-time sensor data processing and TinyML integration
 * - System health monitoring with automatic error recovery
 * - Temperature and fan control with safety limits
 * - Comprehensive logging for debugging and diagnostics
 * 
 * Communication Flow:
 * 1. ESP32 sends commands to STM32 via sendCommandAsync()
 * 2. STM32 responds with data payload (success) or silence (failure)
 * 3. Message parser processes incoming bytes using state machine
 * 4. Sensor data is continuously parsed and stored for TinyML processing
 * 5. System monitors communication health and handles errors
 * 
 * Protocol Format: [STM][LEN][DIR][CMD][DATA...][CHKSUM][ETX]
 */

#include "uartMaster.h"

static const char TAG[] = __FILE__;

// =============================================================================
// UART Configuration and Global Variables
// =============================================================================

// UART Configuration:
// - UART0 (Serial): PC Python test protocol communication
// - STMSerial: ESP32-STM32 communication (defined in globals.h)
// - ESP_LOG: Debug output (handled automatically)

// -----------------------------------------------
// Message Parsing State Variables
// -----------------------------------------------
static MessageState currentState = MessageState::WAITING_START;    // Parser state machine current state
static ProtocolMessage currentMessage = {};                        // Currently parsed message buffer
static uint8_t messageBuffer[USART_MESSAGE_MAXIMUM_LENGTH];       // Raw message byte buffer
static size_t bufferIndex = 0;                                    // Current position in message buffer
static size_t expectedDataLength = 0;                             // Expected data payload length

// -----------------------------------------------
// Sensor Data Management
// -----------------------------------------------
static bool sensorDataAvailable = false;                          // Flag indicating new sensor data ready

// -----------------------------------------------
// Device State Tracking
// -----------------------------------------------
static DeviceMode currentStmMode = DeviceMode::WAITING;        // Current STM32 operating mode (default: WAITING)
static uint8_t lastErrorCode = 0;                                 // Last error code received from STM (0 = no error)

// -----------------------------------------------
// TinyML Processing Variables
// -----------------------------------------------
static uint8_t tinyMLInferenceResult = 0;                         // Result from TinyML pose detection inference
static uint8_t finalResult = 0;                                   // Final processed result for decision making

// -----------------------------------------------
// Asynchronous Command Management (ESP→STM)
// -----------------------------------------------
static PendingCommand pendingCommands[MAX_PENDING_COMMANDS];       // Queue of pending ESP→STM commands
static uint8_t nextCommandId = 1;                                 // Next command ID for tracking (unused currently)

// -----------------------------------------------
// System Health and Error Tracking
// -----------------------------------------------
static SystemState currentEspState = SystemState::NORMAL;      // Overall ESP system health state
static uint32_t lastCommunicationError = 0;                       // Timestamp of last communication error
static uint8_t consecutiveTimeouts = 0;                           // Count of consecutive command timeouts
static uint8_t consecutiveChecksumErrors = 0;                      // Count of consecutive checksum errors  
static uint8_t consecutiveProtocolErrors = 0;                      // Count of consecutive protocol errors
static uint32_t lastProtocolErrorTime = 0;                         // Timestamp of last protocol error

// -----------------------------------------------
// Message Construction Working Variables
// -----------------------------------------------
byte LEN;                                                          // Message length field (working variable)
uint8_t CHKSUM;                                                   // Checksum field (working variable)

// =============================================================================
// Helper Functions for Command Management
// =============================================================================

/**
 * @brief Find empty slot in pending commands array
 * @return Index of empty slot, or -1 if no slots available
 */
static int findEmptySlot() {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command == 0) {
            return i;
        }
    }
    return -1; // No empty slot found
}

/**
 * @brief Find command slot by command code
 * @param command Command code to search for
 * @return Index of command slot, or -1 if not found
 */
static int findCommandSlot(uint8_t command) {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command == command && 
            pendingCommands[i].state == CommandState::SENT) {
            return i;
        }
    }
    return -1; // Command not found
}

// =============================================================================
// Core Protocol Functions
// =============================================================================

/**
 * @brief Calculate XOR checksum for protocol message verification
 * @param data Message data including STM, LEN, DIR, CMD, and DATA fields
 * @param length Total message length
 * @return Calculated XOR checksum
 */
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
    size_t dataStart = 4;
    size_t dataEnd = length - 2;  // Exclude CHKSUM and ETX
    
    for (size_t i = dataStart; i < dataEnd; i++) {
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
    // Send protocol ACK response: STX + LEN + DIR(0x02) + CMD + CHK + ETX
    sendAckResponse(receivedCommand);
}

void sendAckResponse(uint8_t receivedCommand) {
    // Stack-based buffer for ACK response
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t index = 0;
    
    // 1. STM (start byte)
    buffer[index++] = STM;  // 0x02
    
    // 2. LEN (length: DIR + CMD + CHKSUM + ETX = 4)
    buffer[index++] = 4;
    
    // 3. DIR (ACK direction)
    buffer[index++] = MSG_RESPONSE;  // 0x02
    
    // 4. CMD (echo received command)
    buffer[index++] = receivedCommand;
    
    // 5. CHECKSUM
    uint8_t checksum = calculateChecksum(buffer, index);
    buffer[index++] = checksum;
    
    // 6. ETX (end byte)
    buffer[index++] = ETX;  // 0x03
    
    // Send the ACK
    size_t bytesWritten = STMSerial.write(buffer, index);
    STMSerial.flush();
    
    if (bytesWritten != index) {
        ESP_LOGW(TAG, "[ACK-ERR] Failed to send complete ACK for 0x%02X (%zu/%zu bytes)", 
                 receivedCommand, bytesWritten, index);
    } else {
        // ESP_LOGI(TAG, "[ESP→STM] ACK 0x%02X %s (%zu bytes)", 
        //          receivedCommand, getCommandName(receivedCommand), index);
    }
}


/**
 * @brief Construct complete protocol message in provided buffer
 * 
 * Builds a complete UART protocol message with proper framing:
 * [STM][LEN][DIR][CMD][DATA...][CHKSUM][ETX]
 * 
 * @param buffer Output buffer for constructed message
 * @param command Command code to send
 * @param data Optional payload data
 * @param dataLen Length of payload data
 * @return Total message length, or 0 on error
 */
size_t buildMessage(uint8_t* buffer, byte command, const byte* data, size_t dataLen) {
    if (!buffer) return 0;  // Safety check
    
    size_t index = 0;
    
    // 1. STM (start byte)
    buffer[index++] = STM;  // 0x02
    
    // 2. LEN (length: DIR + CMD + DATA + CHKSUM + ETX)
    uint8_t length = 4 + dataLen;  // DIR(1) + CMD(1) + CHKSUM(1) + ETX(1) + DATA(dataLen)
    buffer[index++] = length;
    
    // 3. DIR (direction)
    buffer[index++] = MSG_REQUEST;  // 0x20
    
    // 4. CMD (command)
    buffer[index++] = command;
    
    // 5. DATA (payload data)
    if (data && dataLen > 0) {
        for (size_t i = 0; i < dataLen; i++) {
            buffer[index++] = data[i];
        }
    }
    
    // 6. CHKSUM (checksum)
    uint8_t checksum = calculateChecksum(buffer, index + 2);  // +2 for CHKSUM and ETX
    buffer[index++] = checksum;
    
    // 7. ETX (end byte)
    buffer[index++] = ETX;  // 0x03
    
    return index;  // Total message length
}

// Efficient command transmission function - stack-based buffer
bool sendCommandSafe(byte command, const byte* data, size_t dataLen) {
    // Stack-based buffer - no dynamic allocation
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    
    // Build message using new stack-based function
    size_t messageSize = buildMessage(buffer, command, data, dataLen);
    if (messageSize == 0) {
        ESP_LOGE(TAG, "Failed to build message for command 0x%02X", command);
        return false;
    }
    
    // Log outgoing message (DEBUG level to reduce verbosity)
    ESP_LOGD(TAG, "[ESP→STM] 0x%02X %s (%d bytes)", command, getCommandName(command), messageSize);
    
    // Hex dump of outgoing packet
    char hexDump[256] = {0};
    for (size_t i = 0; i < messageSize && i < 32; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), 
                 "%02X ", buffer[i]);
    }
    ESP_LOGD(TAG, "[ESP→STM] Raw: %s", hexDump);
    
    // Send message via UART
    size_t bytesWritten = STMSerial.write(buffer, messageSize);
    
    if (bytesWritten != messageSize) {
        ESP_LOGW(TAG, "[STM-ERR] Failed to send complete command 0x%02X (%zu/%zu bytes)", 
                 command, bytesWritten, messageSize);
        return false;
    }
    
    return true;
}

// =============================================================================
// Message Processing Engine
// =============================================================================

/**
 * @brief State machine-based message parser for incoming UART bytes
 * @param byte Incoming byte from UART
 * @return true if complete message parsed, false if still parsing
 */
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
            
            // Only calculate data length if not already set (e.g., by resetParserWithFlush)
            if (expectedDataLength == 0 && currentMessage.dataLength == 0) {
                // Calculate data length: LEN - DIR(1) - CMD(1) - CHKSUM(1) - ETX(1)
                expectedDataLength = currentMessage.length - 4;
                currentMessage.dataLength = expectedDataLength;
                
            }
            
            // CRITICAL BUG FIX: Validate expectedDataLength to prevent infinite loops
            if (expectedDataLength < 0) {
                ESP_LOGW(TAG, "[PARSER] Invalid message: negative dataLength %d (len=%d), resetting parser", 
                         expectedDataLength, currentMessage.length);
                resetParser();
                return false;
            }
            
            if (expectedDataLength > 64) {  // Max 64 bytes of data (conservative limit)
                ESP_LOGW(TAG, "[PARSER] Invalid message: dataLength %d exceeds maximum 64, resetting parser", 
                         expectedDataLength);
                resetParser();
                return false;
            }
            
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
            {
                currentMessage.etx = byte;
                messageBuffer[bufferIndex++] = byte;
                
                
                // Only declare completion if EVERYTHING is perfect
                currentState = MessageState::MESSAGE_COMPLETE;
                return true; // Message complete
            }
            
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

/**
 * @brief Simplified parser reset with complete buffer flush
 * @param forceResync If true, performs more aggressive clearing
 */
void resetParserWithFlush(bool forceResync = false) {
    // Reset software parser state completely
    resetParser();
    
    // Flush hardware UART buffers to clear any remaining data
    STMSerial.flush();
    
    // SIMPLIFIED: Always clear buffer completely - no preservation attempts
    int bytesCleared = 0;
    const int maxBytesToClear = forceResync ? 200 : 50;  // More aggressive clearing
    
    // Clear everything from receive buffer
    while (STMSerial.available() && bytesCleared < maxBytesToClear) {
        uint8_t discardedByte = STMSerial.read();
        bytesCleared++;
        
        // Log first few discarded bytes for debugging
        if (bytesCleared <= 10) {
            ESP_LOGV(TAG, "[PARSER] Discarding byte[%d]: 0x%02X", bytesCleared, discardedByte);
        }
        
        // Small delay to prevent overwhelming the system
        if (bytesCleared % 10 == 0) {
            vTaskDelay(1 / portTICK_RATE_MS);
        }
    }
    
    // Log completion
    if (bytesCleared > 0) {
        ESP_LOGW(TAG, "[PARSER] Complete buffer flush - cleared %d bytes (forceResync=%s)", 
                 bytesCleared, forceResync ? "true" : "false");
    } else {
        ESP_LOGV(TAG, "[PARSER] Buffer was already empty");
    }
    
    // Additional wait for force resync to let STM32 finish transmission
    if (forceResync) {
        ESP_LOGW(TAG, "[PARSER] Force resync - waiting 20ms for STM32 to stabilize");
        vTaskDelay(20 / portTICK_RATE_MS);
    }
    
    ESP_LOGV(TAG, "[PARSER] Simplified parser reset completed - ready for new message");
}

/**
 * @brief Handle protocol errors with immediate recovery and escalating measures
 * @param errorType Type of error that occurred
 */
void handleConsecutiveErrors(const char* errorType) {
    uint32_t currentTime = xTaskGetTickCount();
    
    // Update error counters first
    if (currentTime - lastProtocolErrorTime < pdMS_TO_TICKS(1000)) {
        // Consecutive error within 1 second
        if (strcmp(errorType, "CHECKSUM") == 0) {
            consecutiveChecksumErrors++;
        } else {
            consecutiveProtocolErrors++;
        }
    } else {
        // Reset counters for non-consecutive errors  
        consecutiveChecksumErrors = 0;
        consecutiveProtocolErrors = 0;
        
        // Start new error sequence
        if (strcmp(errorType, "CHECKSUM") == 0) {
            consecutiveChecksumErrors = 1;
        } else {
            consecutiveProtocolErrors = 1;
        }
    }
    
    lastProtocolErrorTime = currentTime;
    
    // Apply single recovery level based on error count (no duplicate execution)
    if (consecutiveChecksumErrors >= 3) {
        ESP_LOGW(TAG, "[RECOVERY-L3] %d consecutive %s errors - FORCE RESYNC with delay", 
                 consecutiveChecksumErrors, errorType);
        resetParserWithFlush(true);  // Force aggressive resync
        vTaskDelay(pdMS_TO_TICKS(50));  // Longer recovery delay
        consecutiveChecksumErrors = 0;  // Reset after Level 3 recovery
        
    } else if (consecutiveProtocolErrors >= 2 || consecutiveChecksumErrors >= 2) {
        ESP_LOGW(TAG, "[RECOVERY-L2] %d %s errors - enhanced recovery with short delay", 
                 (strcmp(errorType, "CHECKSUM") == 0) ? consecutiveChecksumErrors : consecutiveProtocolErrors, 
                 errorType);
        resetParserWithFlush(false);  // Basic flush only (reduced intensity)  
        vTaskDelay(pdMS_TO_TICKS(10));  // Reduced delay (20ms → 10ms)
        
    } else {
        ESP_LOGD(TAG, "[RECOVERY-L1] First %s error - basic recovery", errorType);
        resetParserWithFlush(false);  // Basic recovery only
    }
    
    ESP_LOGD(TAG, "[ERROR-STATS] Checksum: %d, Protocol: %d", 
             consecutiveChecksumErrors, consecutiveProtocolErrors);
}

// Message type specific handlers
void handleStatusMessage() {
    if (currentMessage.command == statMessage) {
        parseSensorData(currentMessage.data, currentMessage.dataLength);
        sensorDataAvailable = true;
        
        // Create consolidated log for STATUS message (like INIT commands)
        SensorReading currentReading = getCurrentSensorData();
        
        // Convert sensor values for display
        float leftPressure = currentReading.leftPressure / 100.0f;
        float rightPressure = currentReading.rightPressure / 100.0f;
        float outsideTemp = currentReading.outsideTemp / 100.0f;
        float boardTemp = currentReading.boardTemp / 100.0f;
        float actuatorTemp = currentReading.actuatorTemp / 100.0f;
        float batteryVoltage = currentReading.batteryVoltage / 100.0f;
        float objectDistance = currentReading.objectDistance / 100.0f;
        float actuatorDisp = currentReading.actuatorDisplacement / 100.0f;
        
        // Check if IMU is active
        bool imuActive = (currentReading.leftGyro[0] != 0 || currentReading.leftGyro[1] != 0 || currentReading.leftGyro[2] != 0 ||
                         currentReading.rightGyro[0] != 0 || currentReading.rightGyro[1] != 0 || currentReading.rightGyro[2] != 0 ||
                         currentReading.leftAccel[0] != 0 || currentReading.leftAccel[1] != 0 || currentReading.leftAccel[2] != 0 ||
                         currentReading.rightAccel[0] != 0 || currentReading.rightAccel[1] != 0 || currentReading.rightAccel[2] != 0);
        
        // Log consolidated STATUS message with detailed IMU data
        if (imuActive) {
            ESP_LOGI(TAG, "[STATUS] STM → ESP (0x%02X) | Press:%.2fg/%.2fg | Temp:%.2f°C,%.2f°C,%.2f°C | Batt:%.2fV | Obj:%.2fmm | Disp:%.2fmm | L_Gyro:[%d,%d,%d] L_Acc:[%d,%d,%d] | R_Gyro:[%d,%d,%d] R_Acc:[%d,%d,%d] | IMU_EVT:0x%02X/0x%02X",
                     statMessage,
                     leftPressure, rightPressure,
                     outsideTemp, boardTemp, actuatorTemp,
                     batteryVoltage, objectDistance, actuatorDisp,
                     currentReading.leftGyro[0], currentReading.leftGyro[1], currentReading.leftGyro[2],
                     currentReading.leftAccel[0], currentReading.leftAccel[1], currentReading.leftAccel[2],
                     currentReading.rightGyro[0], currentReading.rightGyro[1], currentReading.rightGyro[2],
                     currentReading.rightAccel[0], currentReading.rightAccel[1], currentReading.rightAccel[2],
                     currentReading.leftIMUEvent, currentReading.rightIMUEvent);
        } else {
            ESP_LOGI(TAG, "[STATUS] STM → ESP (0x%02X), IMU:OFF | Press:%.2fg/%.2fg | Temp:%.2f°C,%.2f°C,%.2f°C | Batt:%.2fV | Obj:%.2fmm | Disp:%.2fmm | IMU_EVT:0x%02X/0x%02X",
                     statMessage,
                     leftPressure, rightPressure,
                     outsideTemp, boardTemp, actuatorTemp,
                     batteryVoltage, objectDistance, actuatorDisp,
                     currentReading.leftIMUEvent, currentReading.rightIMUEvent);
        }
        
        sendResponse(currentMessage.command);
        
        // Maintain existing TinyML logic
        biosigData1Min.addData(allData);
        predictionInput.addData(allData);
        
        // TinyML inference and STM control logic
        if (predictionInput.index >= TINYML_BUFFER_LEN && 
            predictionInput.tof1SecMin < TOF_THRESHOLD && 
            predictionInput.tofBuffer[TINYML_BUFFER_LEN-1] > predictionInput.tofBuffer[TINYML_BUFFER_LEN - 2] + 10) {
            
            tinyMLInferenceResult = tinyMLInference(predictionInput);
            
            if (tinyMLInferenceResult == 1) { 
                finalResult += 1; 
            } else { 
                finalResult = 0; 
            }

#ifdef ADJUST_TINYML_PREDICTION
            if (predictionInput.calculateTofSlope(15, 3)) {
                ESP_LOGI(TAG, "Original: %d, adjusted prediction result to 1", tinyMLInferenceResult);
                tinyMLInferenceResult = 1;                       
                finalResult = 2;
            }
#endif
        }
        
        // STM FORCE_UP control
        if (finalResult == 2) {
            ESP_LOGI(TAG, "[STM-SIM] TinyML trigger detected: ToF=%d, sending CONTROL: Set Device Mode to FORCE_UP", 
                     predictionInput.tofBuffer[TINYML_BUFFER_LEN-1]);
            setDeviceMode(DeviceMode::FORCE_UP);
            finalResult = 0;
        }
        
        // TODO: Add FORCE_DOWN control logic
        // - Detect condition for FORCE_DOWN (e.g., specific sensor pattern, timeout, or user input)
        // - Send setDeviceMode(DeviceMode::FORCE_DOWN) command
        // - Log appropriate message for debugging
    }
}

// Mode change event handler
void handleModeChangeEvent() {
    if (currentMessage.dataLength > 0) {
        uint8_t modeValue = currentMessage.data[0];
        if (modeValue <= 7) {  // Valid mode range: 0-7
            currentStmMode = static_cast<DeviceMode>(modeValue);
            const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "IMU", "TEST", "ERROR"};
            ESP_LOGI(TAG, "Mode: %s (%d)", modeNames[modeValue], modeValue);
            
            switch (currentStmMode) {
                case DeviceMode::SLEEP:
                    // TODO: Implement SLEEP mode handling
                    ESP_LOGI(TAG, "Sleep mode activated");

                    break;
                    
                case DeviceMode::WAITING:
                    // TODO: Implement WAITING mode handling
                    ESP_LOGI(TAG, "Waiting mode activated");
                    break;
                    
                case DeviceMode::FORCE_UP:
                    // TODO: Implement FORCE_UP mode handling
                    ESP_LOGI(TAG, "Force Up mode activated");
                    break;
                    
                case DeviceMode::FORCE_ON:
                    // TODO: Implement FORCE_ON mode handling
                    ESP_LOGI(TAG, "Force On mode activated");
                    break;
                    
                case DeviceMode::FORCE_DOWN:
                    // TODO: Implement FORCE_DOWN mode handling
                    ESP_LOGI(TAG, "Force Down mode activated");
                    break;
                    
                case DeviceMode::IMU:
                    // TODO: Implement IMU mode handling
                    ESP_LOGI(TAG, "IMU mode activated");
                    break;
                    
                case DeviceMode::TEST:
                    // TODO: Implement TEST mode handling
                    ESP_LOGI(TAG, "TEST mode activated");
                    break;
                    
                case DeviceMode::ERROR:
                    ESP_LOGW(TAG, "STM entered ERROR mode - will be handled by main loop");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown mode: %d", modeValue);
                    break;
            }
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
            ESP_LOGI(TAG, "STM init started");          

            break;

        case evtInitResult:  // 0x81
            ESP_LOGI(TAG, "STM init complete"); // STM ready to start

            digitalWrite(PWM_ESP_HEATER, HIGH); // Turn on heater after STM init

            initUpperTemperatureLimit(80, 0);
            initOperatingTemperature(OperatingTempLevel::FORCE_LOW);
            initWaitingTemperature(38, 0);
            initSleepTemperature(32, 0);

            initTimeoutConfiguration(10, 60, 60, 250);  //forceup, forceon, forcedown, waiting

            
            initCoolingFanPWM(1, 10);  // pwm range 0-10, max pwm 10
            initSpeakerVolume(7);
            initForeDownDelay(3, 3);

            initGyroActiveAngle(20);
            initGyroRelativeAngle(5);

            initDeviceMode(1);    //imu mode

            break;

        case evtMode:        // 0x82
            ESP_LOGI(TAG, "Mode change event");
그거 머
            handleModeChangeEvent();
            
            break;

        default:
            ESP_LOGW(TAG, "Unknown event: 0x%02X (%s)", 
                     currentMessage.command, getCommandName(currentMessage.command));
            break;
    }
    sendResponse(currentMessage.command);
}

void handleErrorMessage() {
    if (currentMessage.dataLength >= 2) {
        // Extract 16-bit error code from DATA1 (2 bytes, little-endian)
        uint16_t errorCode = (currentMessage.data[1] << 8) | currentMessage.data[0];
        lastErrorCode = errorCode;  // Update error state
        
        ESP_LOGW(TAG, "ERROR notification from STM: 0x%04X", errorCode);
        
        // Decode error bits and log each active error
        if (errorCode & ERR_IR_TEMP)     ESP_LOGW(TAG, "  → IR temperature error");
        if (errorCode & ERR_AMB_TEMP)    ESP_LOGW(TAG, "  → Ambient temperature error");
        if (errorCode & ERR_IMU)         ESP_LOGW(TAG, "  → IMU error");
        if (errorCode & ERR_FAN)         ESP_LOGW(TAG, "  → Fan error");
        if (errorCode & ERR_COOL_FAN)    ESP_LOGW(TAG, "  → Cooling fan error");
        if (errorCode & ERR_TOF)         ESP_LOGW(TAG, "  → TOF sensor error");
        if (errorCode & ERR_AUDIO)       ESP_LOGW(TAG, "  → Audio system error");
        if (errorCode & ERR_FSR)         ESP_LOGW(TAG, "  → FSR sensor error");
        if (errorCode & ERR_SD_CARD)     ESP_LOGW(TAG, "  → SD card error");
        if (errorCode & ERR_MP3_FILE)    ESP_LOGW(TAG, "  → MP3 file error");
        
        // Check for unknown error bits
        uint16_t knownErrors = ERR_IR_TEMP | ERR_AMB_TEMP | ERR_IMU | ERR_FAN | 
                              ERR_COOL_FAN | ERR_TOF | ERR_AUDIO | ERR_FSR |
                              ERR_SD_CARD | ERR_MP3_FILE;
        if (errorCode & ~knownErrors) {
            ESP_LOGW(TAG, "  → Unknown error bits: 0x%04X", errorCode & ~knownErrors);
        }
        
    } else if (currentMessage.dataLength == 1) {
        // Handle legacy single-byte error codes for backward compatibility
        uint8_t errorCode = currentMessage.data[0];
        lastErrorCode = errorCode;
        ESP_LOGW(TAG, "ERROR (legacy format): %d", errorCode);
        
    } else {
        ESP_LOGW(TAG, "ERROR message with invalid data length: %d", currentMessage.dataLength);
    }
    
    // STM sent error message, so update STM state to ERROR
    currentStmMode = DeviceMode::ERROR;
    ESP_LOGW(TAG, "STM mode set to ERROR due to error message");
    
    // STM error will be handled by main loop state check
    
    sendResponse(currentMessage.command);
}


/**
 * @brief Parse incoming sensor data from STM32 status messages
 * 
 * This algorithm extracts all sensor readings from the binary data payload
 * received in STM status messages (0x70). The data is packed in a specific
 * format with 16-bit values for most sensors.
 * 
 * Data Format (total ~34 bytes):
 * - IMU data: Left gyro (6), Left accel (6), Right gyro (6), Right accel (6)
 * - Pressure: Left (2), Right (2)
 * - Temperature: Outside (2), Board (2), Actuator (2)
 * - Other: Actuator displacement (2), Object distance (2), Battery (2)
 * - Events: Left IMU event (1), Right IMU event (1)
 * 
 * @param data Binary sensor data payload
 * @param length Length of data payload
 */
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
            allData.L_gyro[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Accelerometer data - Left (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            allData.L_accel[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // IMU data - Right side (12 bytes total)
    // Gyro data - Right (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            allData.R_gyro[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Accelerometer data - Right (6 bytes: X, Y, Z axes * 2 bytes each)
    for (int i = 0; i < 3; i++) {
        if (index + 1 < length) {
            allData.R_accel[i] = (data[index] << 8) | data[index + 1];
            index += PRESSURE_DATA_SIZE;
        }
    }
    
    // Pressure sensors (4 bytes total: L + R)
    // Format: First byte = integer part, Second byte = decimal part (e.g., 0x05 0x25 = 5.25g)
    // Left pressure sensor (2 bytes)
    if (index + 1 < length) {
        uint8_t pressureInt = data[index];      // Integer part
        uint8_t pressureDec = data[index + 1];  // Decimal part
        allData.L_ads = (pressureInt * 100) + pressureDec;  // Store as integer.decimal format
        index += PRESSURE_DATA_SIZE;
    }
    // Right pressure sensor (2 bytes)
    if (index + 1 < length) {
        uint8_t pressureInt = data[index];      // Integer part
        uint8_t pressureDec = data[index + 1];  // Decimal part
        allData.R_ads = (pressureInt * 100) + pressureDec;  // Store as integer.decimal format
        index += PRESSURE_DATA_SIZE;
    }
    
    // Temperature sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Format: First byte = integer part, Second byte = decimal part (e.g., 0x18 0x40 = 24.64°C)
    // Outside temperature (2 bytes)
    if (index + 1 < length) {
        uint8_t tempInt = data[index];      // Integer part
        uint8_t tempDec = data[index + 1];  // Decimal part
        allData.outTemp = (tempInt * 100) + tempDec;  // Store as 2464 for 24.64°C
        index += TEMPERATURE_DATA_SIZE;
    }
    // Board temperature (2 bytes)
    if (index + 1 < length) {
        uint8_t tempInt = data[index];      // Integer part
        uint8_t tempDec = data[index + 1];  // Decimal part
        allData.boardTemp = (tempInt * 100) + tempDec;  // Store as integer.decimal format
        index += TEMPERATURE_DATA_SIZE;
    }
    // Actuator temperature (2 bytes)
    if (index + 1 < length) {
        uint8_t tempInt = data[index];      // Integer part
        uint8_t tempDec = data[index + 1];  // Decimal part
        allData.lmaTemp = (tempInt * 100) + tempDec;  // Store as integer.decimal format
        index += TEMPERATURE_DATA_SIZE;
    }
    
    // Other sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Format: First byte = integer part, Second byte = decimal part
    // Actuator displacement (2 bytes)
    if (index + 1 < length) {
        uint8_t dispInt = data[index];      // Integer part (mm)
        uint8_t dispDec = data[index + 1];  // Decimal part
        allData.lmaLength = (dispInt * 100) + dispDec;  // Store as integer.decimal format
        index += TEMPERATURE_DATA_SIZE;
    }
    // Object distance (2 bytes)
    if (index + 1 < length) {
        uint8_t distInt = data[index];      // Integer part (mm)
        uint8_t distDec = data[index + 1];  // Decimal part
        allData.objDistance = (distInt * 100) + distDec;  // Store as integer.decimal format
        index += TEMPERATURE_DATA_SIZE;
    }
    // Battery voltage (2 bytes)
    if (index + 1 < length) {
        uint8_t voltInt = data[index];      // Integer part (V)
        uint8_t voltDec = data[index + 1];  // Decimal part
        allData.battery = (voltInt * 100) + voltDec;  // Store as integer.decimal format
        index += TEMPERATURE_DATA_SIZE;
    }
    
    // IMU events (2 bytes total: L + R)
    // Left IMU event (1 byte)
    if (index < length) {
        allData.leftIMUEvent = data[index];
        index += IMU_EVENT_SIZE;
    }
    // Right IMU event (1 byte)
    if (index < length) {
        allData.rightIMUEvent = data[index];
        index += IMU_EVENT_SIZE;
    }
}

/**
 * @brief Format and log sensor data in human-readable format
 * @param reading SensorReading structure with parsed sensor data
 */
void logSensorDataFormatted(const SensorReading& reading) {
    // Check if IMU data is active (non-zero values)
    bool imuActive = (reading.leftGyro[0] != 0 || reading.leftGyro[1] != 0 || reading.leftGyro[2] != 0 ||
                     reading.rightGyro[0] != 0 || reading.rightGyro[1] != 0 || reading.rightGyro[2] != 0 ||
                     reading.leftAccel[0] != 0 || reading.leftAccel[1] != 0 || reading.leftAccel[2] != 0 ||
                     reading.rightAccel[0] != 0 || reading.rightAccel[1] != 0 || reading.rightAccel[2] != 0);
    
    // Convert temperature values from integer.decimal format (e.g., 2464 = 24.64°C)
    float outsideTemp = reading.outsideTemp / 100.0f;
    float boardTemp = reading.boardTemp / 100.0f;
    float actuatorTemp = reading.actuatorTemp / 100.0f;
    
    // Convert other sensor values from integer.decimal format
    float leftPressure = reading.leftPressure / 100.0f;     // e.g., 525 = 5.25g
    float rightPressure = reading.rightPressure / 100.0f;   // e.g., 325 = 3.25g
    float batteryVoltage = reading.batteryVoltage / 100.0f;  // e.g., 1496 = 14.96V
    float objectDistance = reading.objectDistance / 100.0f;  // e.g., 1250 = 12.50mm
    float actuatorDisp = reading.actuatorDisplacement / 100.0f; // e.g., 825 = 8.25mm
    
    // Log formatted sensor data
    ESP_LOGI(TAG, "[SENSOR] IMU:%s | Press:%.2fg/%.2fg | Temp:%.2f°C,%.2f°C,%.2f°C | Batt:%.2fV | Obj:%.2fmm | Disp:%.2fmm | IMU_EVT:0x%02X/0x%02X",
             imuActive ? "ON" : "OFF",
             leftPressure, rightPressure,
             outsideTemp, boardTemp, actuatorTemp,
             batteryVoltage, objectDistance, actuatorDisp,
             reading.leftIMUEvent, reading.rightIMUEvent);
    
    // Log IMU details only if active
    if (imuActive) {
        ESP_LOGI(TAG, "[IMU] L_Gyro:[%d,%d,%d] L_Accel:[%d,%d,%d] R_Gyro:[%d,%d,%d] R_Accel:[%d,%d,%d]",
                 reading.leftGyro[0], reading.leftGyro[1], reading.leftGyro[2],
                 reading.leftAccel[0], reading.leftAccel[1], reading.leftAccel[2],
                 reading.rightGyro[0], reading.rightGyro[1], reading.rightGyro[2],
                 reading.rightAccel[0], reading.rightAccel[1], reading.rightAccel[2]);
    }
}

/**
 * @brief Main UART communication handler task
 * 
 * This is the core message processing loop that handles all incoming UART data
 * from the STM32. It implements the complete message reception, parsing, validation,
 * and response handling algorithm.
 * 
 * Algorithm Flow:
 * 1. Read incoming bytes from UART buffer
 * 2. Feed bytes to state machine parser (parseIncomingByte)
 * 3. When complete message received, validate frame and checksum
 * 4. Classify message type and route to appropriate handler
 * 5. Process sensor data and update system state
 * 6. Send acknowledgments as required by protocol
 * 7. Handle timeouts and error recovery
 * 
 * @param pvParameters FreeRTOS task parameters (unused)
 */
void usartMasterHandler(void *pvParameters) {
    static bool testExecuted = false;
    static uint32_t startTime = xTaskGetTickCount();
    static uint32_t lastParserStateTime = xTaskGetTickCount();
    static MessageState lastParserState = MessageState::WAITING_START;

    while (1) {
        int bytesAvailable = STMSerial.available();
        
        while (STMSerial.available()) {
            uint8_t incomingByte = STMSerial.read();
            
            // Prevent buffer overflow
            if (bufferIndex >= USART_MESSAGE_MAXIMUM_LENGTH) {
                // Show what we have and reset
                printf("[%6d] [RAW] ", (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
                for (size_t i = 0; i < bufferIndex; i++) {
                    printf("%02X ", messageBuffer[i]);
                }
                printf("(OVERFLOW %zu bytes)\n", bufferIndex);
                resetParser();
                continue;
            }
            
            // Try parsing
            bool messageComplete = parseIncomingByte(incomingByte);
            
            // If buffer seems stuck (no STM found for a while), show what we have
            if (!messageComplete && bufferIndex > 50 && currentState == MessageState::WAITING_START) {
                printf("[%6d] [RAW] ", (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
                for (size_t i = 0; i < bufferIndex; i++) {
                    printf("%02X ", messageBuffer[i]);
                }
                printf("(NO_STM %zu bytes)\n", bufferIndex);
                resetParser();
                continue;
            }
            
            if (messageComplete) {
                // Message complete - create hex dump with STRICT message boundary
                size_t expectedTotalLength = currentMessage.length + 2; // +2 for STM and LEN bytes
                
                // Log warning if buffer contains more than expected (consecutive message detection)
                if (bufferIndex > expectedTotalLength) {
                    size_t extraBytes = bufferIndex - expectedTotalLength;
                    ESP_LOGD(TAG, "[MESSAGE-BOUNDARY] Detected %zu extra bytes in buffer - likely consecutive message", extraBytes);
                }
                
                // Raw packet logging and message info
                // Show raw packet data for debugging
                printf("[%6d] [RAW] ", (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
                for (size_t i = 0; i < expectedTotalLength && i < bufferIndex; i++) {
                    printf("%02X ", messageBuffer[i]);
                }
                printf("(%zu bytes)\n", bufferIndex);
                
                if (currentMessage.command == statMessage) {
                    // For sensor data, minimal logging (detailed in handleStatusMessage)
                } else {
                    // For other commands, show first 8 bytes only
                    ESP_LOGI(TAG, "[STM→ESP] %s, LEN=%d, First8: %02X %02X %02X %02X %02X %02X %02X %02X", 
                             getCommandName(currentMessage.command), currentMessage.length,
                             messageBuffer[0], messageBuffer[1], messageBuffer[2], messageBuffer[3],
                             messageBuffer[4], messageBuffer[5], messageBuffer[6], messageBuffer[7]);
                }
                
                // Enhanced protocol validation with command-type-specific direction checking
                const char* validationError = nullptr;
                if (!currentMessage.isValidStrictWithCommandCheck(&validationError)) {
                    ESP_LOGW(TAG, "[STM-ERR] Protocol validation failed: %s", validationError);
                    ESP_LOGW(TAG, "[STM-ERR] Frame details: STM=0x%02X (expect 0x%02X), DIR=0x%02X, CMD=0x%02X, ETX=0x%02X (expect 0x%02X)", 
                             currentMessage.stm, STM, currentMessage.direction, currentMessage.command,
                             currentMessage.etx, ETX);
                    
                    // Specific STM32 protocol error detection
                    if (currentMessage.etx != ETX) {
                        ESP_LOGE(TAG, "[STM32-PROTOCOL-ERROR] ETX mismatch - STM32 firmware should send 0x%02X instead of 0x%02X", 
                                 ETX, currentMessage.etx);
                    }
                    
                    // Apply consecutive error recovery mechanism
                    handleConsecutiveErrors("PROTOCOL");
                    continue;
                }
                
                // Message length validation
                if (bufferIndex != expectedTotalLength) {
                    ESP_LOGW(TAG, "[STM32-PROTOCOL-ERROR] Length mismatch: received %zu bytes, expected %zu bytes (LEN=0x%02X)", 
                             bufferIndex, expectedTotalLength, currentMessage.length);
                    ESP_LOGW(TAG, "[STM32-FIX-NEEDED] STM32 should send exactly %zu bytes per message", expectedTotalLength);
                    
                    if (bufferIndex > expectedTotalLength) {
                        size_t extraBytes = bufferIndex - expectedTotalLength;
                        ESP_LOGE(TAG, "[STM32-PROTOCOL-ERROR] STM32 sent %zu extra bytes - check for buffer overflow or padding", extraBytes);
                        ESP_LOGE(TAG, "[STM32-DEV-GUIDE] Fix: Ensure STM32 UART transmission stops after ETX byte");
                    }
                }
                
                // Additional STM32 protocol compliance check
                if (currentMessage.etx != ETX) {
                    ESP_LOGE(TAG, "[STM32-DEV-GUIDE] STM32 Protocol Fix Required:");
                    ESP_LOGE(TAG, "[STM32-DEV-GUIDE]   Current ETX: 0x%02X", currentMessage.etx);
                    ESP_LOGE(TAG, "[STM32-DEV-GUIDE]   Expected ETX: 0x%02X", ETX);
                    ESP_LOGE(TAG, "[STM32-DEV-GUIDE]   Action: Set ETX = 0x03 in STM32 message construction");
                }
                
                if (!checkMessage(messageBuffer, bufferIndex)) {
                    uint8_t calculatedChecksum = calculateChecksum(messageBuffer, bufferIndex);
                    ESP_LOGW(TAG, "[STM-ERR] Checksum FAIL: received=0x%02X, calculated=0x%02X", 
                             currentMessage.checksum, calculatedChecksum);
                    
                    
                    // Check if this is a well-formed message - relaxed conditions for debugging
                    bool wellFormedMessage = (currentMessage.stm == STM) && 
                                           (bufferIndex >= 6); // At least STM+LEN+DIR+CMD+CHKSUM+ETX
                    
                    
                    if (wellFormedMessage) {
                        // Continue processing instead of recovery - message structure is correct
                    } else {
                        ESP_LOGW(TAG, "[CHECKSUM] Message malformed - applying recovery mechanism");
                        handleConsecutiveErrors("CHECKSUM");
                        continue;
                    }
                }
                
                // Handle according to message type and command classification
                if (currentMessage.direction == MSG_REQUEST) {
                    // STM spontaneous messages (STATUS, EVENT, ERROR)
                    CommandType cmdType = currentMessage.getCommandType();
                    
                    switch (cmdType) {
                        case CommandType::STATUS:      // 0x70 - STM spontaneous sensor data
                            // STM->ESP STATUS command with sensor data
                            handleStatusMessage();
                            break;
                            
                        case CommandType::EVENT:       // 0x8X - STM spontaneous state changes
                            // STM->ESP EVENT command with state changes
                            handleEventMessage();
                            break;
                            
                        case CommandType::ERROR:       // 0x9X - STM spontaneous error reports
                            // STM->ESP ERROR command with error details
                            handleErrorMessage();
                            break;
                            
                        default:
                            ESP_LOGW(TAG, "Unknown spontaneous command: 0x%02X", currentMessage.command);
                            break;
                    }
                } else if (currentMessage.direction == MSG_RESPONSE) {
                    // STM ACK/response messages for ESP→STM commands
                    CommandType cmdType = currentMessage.getCommandType();
                    
                    switch (cmdType) {
                        case CommandType::INIT:        // 0x1X - ESP→STM INIT command ACK
                            handleInitResponse();
                            break;
                            
                        case CommandType::REQUEST:     // 0x3X - ESP→STM REQUEST command response
                            handleRequestResponse();
                            break;
                            
                        case CommandType::CONTROL:     // 0x5X - ESP→STM CONTROL command ACK
                            handleControlResponse();
                            break;
                            
                        default:
                            ESP_LOGW(TAG, "Unknown response command: 0x%02X", currentMessage.command);
                            break;
                    }
                } else {
                    ESP_LOGW(TAG, "[STM-ERR] Invalid direction: 0x%02X (expected: 0x%02X or 0x%02X)", 
                             currentMessage.direction, MSG_REQUEST, MSG_RESPONSE);
                }
                
                // Reset error counters on successful message processing
                consecutiveChecksumErrors = 0;
                consecutiveProtocolErrors = 0;
                
                resetParser();
            }
        }
        
        // Parser state timeout detection and forced reset
        uint32_t currentTime = xTaskGetTickCount();
        if (lastParserState != currentState) {
            lastParserState = currentState;
            lastParserStateTime = currentTime;
        } else if (currentState != MessageState::WAITING_START && 
                   (currentTime - lastParserStateTime) > (5000 / portTICK_RATE_MS)) {
            ESP_LOGW(TAG, "[PARSER] State %d stuck for 5 seconds, forcing reset", (int)currentState);
            ESP_LOGW(TAG, "[PARSER] bufferIndex: %d, expectedDataLength: %d", bufferIndex, expectedDataLength);
            resetParser();
            lastParserStateTime = currentTime;
        }
        
        // Check for command timeouts and handle critical errors
        processCommandTimeout(); 
        
        // === Centralized Error State Management ===
        // Check ESP state - if already in critical error, minimize processing
        if (currentEspState == SystemState::CRITICAL_ERROR) {
            // Already in critical error - only maintain minimal operations
            vTaskDelay(pdMS_TO_TICKS(1000));  // Longer delay to reduce CPU usage
            continue;
        }
        
        // Check STM state - if STM is in ERROR mode, enter critical state
        if (currentStmMode == DeviceMode::ERROR) {
            ESP_LOGW(TAG, "STM in ERROR mode - entering ESP critical state");
            enterCriticalErrorState();
        }

        vTaskDelay(5 / portTICK_RATE_MS); // OPTIMIZED: Faster response to consecutive messages (10ms → 5ms)
    }
}

bool initUartMaster() {
    // === UART 초기화: STMSerial = UART2 (GPIO 15/16) → STM32 통신 ===
    STMSerial.begin(115200, SERIAL_8N1, ESP_U2_RXD, ESP_U2_TXD);
    ESP_LOGI(TAG, "STMSerial initialized on UART2 (GPIO %d/%d)", ESP_U2_RXD, ESP_U2_TXD);
    
    // Initialize command state variables
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        pendingCommands[i].reset();
    }
    
    // Initialize message parser
    resetParser();
    
    // Create UART task
    xTaskCreatePinnedToCore(
        usartMasterHandler,
        "UART_MASTER_TASK",
        UART_TASK_STACK,
        NULL,
        UART_TASK_PRI,
        &TasUsartMaster_h,
        UART_TASK_CORE
    );
    
    ESP_LOGI(TAG, "[UART-MASTER] UART Master initialization completed");
    return true;
}

// =============================================================================
// Bidirectional Communication Functions (ESP→STM)
// =============================================================================

void handleInitResponse() {
    // Handle INIT command ACKs (0x1X)
    int slot = findCommandSlot(currentMessage.command);
    if (slot >= 0) {
        pendingCommands[slot].state = CommandState::ACK_RECEIVED;
        ESP_LOGI(TAG, "INIT ACK received for command 0x%02X (%s) (slot %d)", 
                 currentMessage.command, getCommandName(currentMessage.command), slot);
        
        // Communication recovery check
        if (currentEspState != SystemState::NORMAL) {
            checkCommunicationRecovery();
        }
        
        // Clear completed command
        pendingCommands[slot].reset();
    } else {
        ESP_LOGI(TAG, "INIT ACK received for command 0x%02X (%s) (no pending command)", 
                 currentMessage.command, getCommandName(currentMessage.command));
    }
}

void handleRequestResponse() {
    // Handle REQUEST command responses with data (0x3X)
    int slot = findCommandSlot(currentMessage.command);
    if (slot >= 0) {
        pendingCommands[slot].state = CommandState::DATA_RECEIVED;
        
        // Store response data
        if (currentMessage.dataLength > 0) {
            size_t copyLen = (currentMessage.dataLength < RESPONSE_BUFFER_SIZE) ? currentMessage.dataLength : RESPONSE_BUFFER_SIZE;
            memcpy(pendingCommands[slot].responseData, currentMessage.data, copyLen);
            pendingCommands[slot].responseLength = copyLen;
        }
        
        // Create consolidated log for REQUEST response (like INIT/STATUS commands)
        char hexDump[32] = {0};
        for (size_t i = 0; i < currentMessage.dataLength && i < 10; i++) {
            snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", currentMessage.data[i]);
        }
        
        // Communication recovery check
        if (currentEspState != SystemState::NORMAL) {
            checkCommunicationRecovery();
        }
        
        // Process response data and create unified log based on command type
        switch (currentMessage.command) {
            case reqTempSleep:      // 0x30
                if (currentMessage.dataLength >= 2) {
                    systemConfig.sleepTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Sleep Temp: %.1f°C [%s]", 
                             currentMessage.command, systemConfig.sleepTemp, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Sleep Temp: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqTempWaiting:    // 0x31
                if (currentMessage.dataLength >= 2) {
                    systemConfig.waitingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Waiting Temp: %.1f°C [%s]", 
                             currentMessage.command, systemConfig.waitingTemp, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Waiting Temp: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqTempForceUp:    // 0x32
                if (currentMessage.dataLength >= 2) {
                    systemConfig.operatingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Operating Temp: %.1f°C [%s]", 
                             currentMessage.command, systemConfig.operatingTemp, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Operating Temp: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqTempHeatPad:    // 0x33 - DEPRECATED
                ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Heat Pad Temp: DEPRECATED [%s]", 
                         currentMessage.command, hexDump);
                break;
                
            case reqUpperTemp:      // 0x34
                if (currentMessage.dataLength >= 2) {
                    systemConfig.upperTempLimit = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Upper Temp Limit: %.1f°C [%s]", 
                             currentMessage.command, systemConfig.upperTempLimit, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Upper Temp Limit: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqPWMCoolFan:     // 0x35
                if (currentMessage.dataLength >= 2) {
                    systemConfig.coolingFanLevel = currentMessage.data[0];
                    systemConfig.maxCoolingFanLevel = currentMessage.data[1];
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Fan Speed: Level %d/%d [%s]", 
                             currentMessage.command, systemConfig.coolingFanLevel, systemConfig.maxCoolingFanLevel, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Fan Speed: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqTimeout:        // 0x36
                if (currentMessage.dataLength >= 8) {
                    systemConfig.forceUpTimeout = (currentMessage.data[0] << 8) | currentMessage.data[1];
                    systemConfig.forceOnTimeout = (currentMessage.data[2] << 8) | currentMessage.data[3];
                    systemConfig.forceDownTimeout = (currentMessage.data[4] << 8) | currentMessage.data[5];
                    systemConfig.waitingTimeout = (currentMessage.data[6] << 8) | currentMessage.data[7];
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Timeout Config: Up=%d, On=%d, Down=%d, Wait=%d [%s]", 
                             currentMessage.command, systemConfig.forceUpTimeout, systemConfig.forceOnTimeout, 
                             systemConfig.forceDownTimeout, systemConfig.waitingTimeout, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Timeout Config: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqSpk:            // 0x37
                if (currentMessage.dataLength >= 1) {
                    systemConfig.speakerVolume = currentMessage.data[0];
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Speaker Volume: %d [%s]", 
                             currentMessage.command, systemConfig.speakerVolume, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Speaker Volume: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqDelay:          // 0x38
                if (currentMessage.dataLength >= 2) {
                    systemConfig.poseDetectionDelay = currentMessage.data[0];    // 100ms units
                    systemConfig.objectDetectionDelay = currentMessage.data[1];  // 100ms units
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Detection Delay: Pose=%ds, Object=%ds [%s]", 
                             currentMessage.command, systemConfig.poseDetectionDelay, systemConfig.objectDetectionDelay, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Detection Delay: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqGyroAct:        // 0x39
                if (currentMessage.dataLength >= 1) {
                    systemConfig.gyroActiveAngle = currentMessage.data[0];
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Gyro Active Angle: %d° [%s]", 
                             currentMessage.command, systemConfig.gyroActiveAngle, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Gyro Active Angle: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqGyroRel:        // 0x40
                if (currentMessage.dataLength >= 1) {
                    systemConfig.gyroRelativeAngle = currentMessage.data[0];
                    ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Gyro Relative Angle: %d° (Total: %d°) [%s]", 
                             currentMessage.command, systemConfig.gyroRelativeAngle,
                             systemConfig.gyroActiveAngle + systemConfig.gyroRelativeAngle, hexDump);
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Gyro Relative Angle: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            case reqMode:           // 0x41
                if (currentMessage.dataLength >= 1) {
                    systemConfig.currentModeValue = currentMessage.data[0];
                    const char* modeNames[] = {"AI mode", "IMU mode"};
                    if (systemConfig.currentModeValue <= 1) {
                        ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Current Mode: %s (%d) [%s]", 
                                 currentMessage.command, modeNames[systemConfig.currentModeValue], 
                                 systemConfig.currentModeValue, hexDump);
                    } else {
                        ESP_LOGI(TAG, "[REQUEST] STM → ESP (0x%02X), Current Mode: Unknown (%d) [%s]", 
                                 currentMessage.command, systemConfig.currentModeValue, hexDump);
                    }
                } else {
                    ESP_LOGW(TAG, "[REQUEST] STM → ESP (0x%02X), Current Mode: INVALID DATA (len=%d) [%s]", 
                             currentMessage.command, currentMessage.dataLength, hexDump);
                }
                break;
                
            default:
                ESP_LOGW(TAG, "Unknown REQUEST response: 0x%02X", currentMessage.command);
                break;
        }
        
        // Clear completed command
        pendingCommands[slot].reset();
    } else {
        ESP_LOGI(TAG, "REQUEST response received for command 0x%02X (%s) (no pending command)", 
                 currentMessage.command, getCommandName(currentMessage.command));
    }
}

void handleControlResponse() {
    // Handle CONTROL command ACKs (0x5X)
    int slot = findCommandSlot(currentMessage.command);
    if (slot >= 0) {
        pendingCommands[slot].state = CommandState::ACK_RECEIVED;
        ESP_LOGI(TAG, "CONTROL ACK received for command 0x%02X (%s) (slot %d)", 
                 currentMessage.command, getCommandName(currentMessage.command), slot);
        
        // Communication recovery check
        if (currentEspState != SystemState::NORMAL) {
            checkCommunicationRecovery();
        }
        
        // Clear completed command
        pendingCommands[slot].reset();
    } else {
        ESP_LOGI(TAG, "CONTROL ACK received for command 0x%02X (%s) (no pending command)", 
                 currentMessage.command, getCommandName(currentMessage.command));
    }
}

bool sendCommandAsync(uint8_t command, const byte* data, size_t dataLen) {
    int slot = findEmptySlot();
    if (slot == -1) {
        ESP_LOGW(TAG, "Cannot send command 0x%02X (%s) - no free slots (max %d)", 
                 command, getCommandName(command), MAX_PENDING_COMMANDS);
        return false;
    }
    
    // Determine response type based on command
    ResponseType expectedResponse = ResponseType::ACK_ONLY;
    CommandType cmdType = static_cast<CommandType>(command & 0xF0);
    if (cmdType == CommandType::REQUEST) {
        expectedResponse = ResponseType::DATA_RESPONSE;
    }
    
    // Setup pending command tracking in available slot
    pendingCommands[slot].reset();
    pendingCommands[slot].command = command;
    pendingCommands[slot].expectedResponse = expectedResponse;
    pendingCommands[slot].state = CommandState::SENT;
    pendingCommands[slot].sentTimestamp = millis();
    pendingCommands[slot].backupOriginalData(data, dataLen);
    
    // Send the command and return immediately - No waiting!
    if (!sendCommandSafe(command, data, dataLen)) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X (%s)", command, getCommandName(command));
        pendingCommands[slot].reset();
        return false;
    }
    
    ESP_LOGD(TAG, "Command 0x%02X (%s) sent asynchronously (slot %d)", 
             command, getCommandName(command), slot);
    return true;
}

// Fire-and-forget command sender - no ACK waiting, no slot management
bool sendCommandFireAndForget(uint8_t command, const byte* data, size_t dataLen) {
    // Send command directly without slot management
    if (!sendCommandSafe(command, data, dataLen)) {
        ESP_LOGE(TAG, "Failed to send fire-and-forget command 0x%02X (%s)", command, getCommandName(command));
        return false;
    }
    
    ESP_LOGD(TAG, "Fire-and-forget command 0x%02X (%s) sent successfully", 
             command, getCommandName(command));
    return true;
}

bool isResponseExpected() {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command != 0) {
            return true;
        }
    }
    return false;
}

CommandState getCommandState(uint8_t command) {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command == command) {
            return pendingCommands[i].state;
        }
    }
    return CommandState::IDLE;
}

void processCommandTimeout() {
    uint32_t currentTime = millis();
    bool hasTimeoutError = false;
    
    // Check all pending commands for timeouts
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command != 0 && 
            pendingCommands[i].state == CommandState::SENT &&
            pendingCommands[i].isTimeoutExpired(currentTime)) {
            
            uint32_t elapsedTime = currentTime - pendingCommands[i].sentTimestamp;
            ESP_LOGW(TAG, "Command 0x%02X (%s) timed out after %lu ms (slot %d, attempt %d/%d)", 
                     pendingCommands[i].command, getCommandName(pendingCommands[i].command), 
                     elapsedTime, i, pendingCommands[i].retryCount + 1, MAX_RETRY_ATTEMPTS + 1);
            
            if (pendingCommands[i].retryCount < MAX_RETRY_ATTEMPTS) {
                // Retry the command with original data
                pendingCommands[i].retryCount++;
                pendingCommands[i].sentTimestamp = currentTime;
                pendingCommands[i].state = CommandState::SENT;
                
                ESP_LOGI(TAG, "Retrying command 0x%02X (%s) (slot %d, attempt %d/%d)", 
                         pendingCommands[i].command, getCommandName(pendingCommands[i].command), i,
                         pendingCommands[i].retryCount + 1, MAX_RETRY_ATTEMPTS + 1);
                
                // Resend the command with original data
                const byte* retryData = (pendingCommands[i].originalDataLength > 0) ? 
                                       pendingCommands[i].originalData : nullptr;
                
                if (!sendCommandSafe(pendingCommands[i].command, retryData, pendingCommands[i].originalDataLength)) {
                    ESP_LOGE(TAG, "Failed to resend command 0x%02X (%s) on retry %d (slot %d)", 
                             pendingCommands[i].command, getCommandName(pendingCommands[i].command), 
                             pendingCommands[i].retryCount, i);
                    pendingCommands[i].reset();
                    hasTimeoutError = true;
                }
            } else {
                // Max retries exceeded - Critical communication failure
                ESP_LOGE(TAG, "CRITICAL: Command 0x%02X (%s) FAILED after %d attempts (slot %d)", 
                         pendingCommands[i].command, getCommandName(pendingCommands[i].command), 
                         MAX_RETRY_ATTEMPTS + 1, i);
                
                pendingCommands[i].reset();
                hasTimeoutError = true;
                
                // Increment consecutive timeout counter
                consecutiveTimeouts++;
                lastCommunicationError = currentTime;
            }
        }
    }
    
    // Handle communication error if any timeouts occurred
    if (hasTimeoutError) {
        handleCommunicationError(currentTime);
    }
}

// =============================================================================
// System State Management and Critical Error Handling Functions
// =============================================================================

void handleCommunicationError(uint32_t currentTime) {
    ESP_LOGE(TAG, "Communication error detected (consecutive timeouts: %d)", consecutiveTimeouts);
    
    // Check for critical communication failure conditions
    if (consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
        ESP_LOGE(TAG, "CRITICAL COMMUNICATION FAILURE - Too many consecutive timeouts (%d)", consecutiveTimeouts);
        enterCriticalErrorState();
        return;
    }
    
    // Check for prolonged communication failure
    if (lastCommunicationError > 0 && 
        (currentTime - lastCommunicationError) < CRITICAL_ERROR_THRESHOLD) {
        ESP_LOGE(TAG, "CRITICAL COMMUNICATION FAILURE - No response for %lu ms", 
                 currentTime - lastCommunicationError);
        enterCriticalErrorState();
        return;
    }
    
    // Set communication error state (but not critical yet)
    if (currentEspState == SystemState::NORMAL) {
        currentEspState = SystemState::COMMUNICATION_ERROR;
        ESP_LOGW(TAG, "System state changed to COMMUNICATION_ERROR");
    }
}

void enterCriticalErrorState() {
    currentEspState = SystemState::CRITICAL_ERROR;
    
    ESP_LOGE(TAG, "=== ENTERING CRITICAL ERROR STATE ===");
    ESP_LOGE(TAG, "STM communication completely failed");
    ESP_LOGE(TAG, "Shutting down actuators for safety");
    
    // Execute emergency safety measures
    emergencyShutdown();
    notifySystemError();
    
    ESP_LOGE(TAG, "=== SYSTEM IN SAFE MODE ===");
}

void emergencyShutdown() {
    ESP_LOGE(TAG, "Emergency shutdown: Turning OFF all actuators");
    
    // ESP 직접 제어 장치
    digitalWrite(PWM_ESP_HEATER, LOW);  // 히터 끄기
    
    // STM 제어 장치 (통신 가능한 경우)
    setBlowerFanState(false);           // 송풍 팬 끄기
    setCoolingFanState(true);           // 냉각팬 켜기 (히터 잔열 제거)
    // setDeviceMode(DeviceMode::ERROR) 제거 - STM과 통신 불가능한 상태일 수 있음
    
    ESP_LOGE(TAG, "Emergency shutdown completed");
}


void notifySystemError() {
    // Notify external systems of critical error
    // TODO: Implement based on system requirements
    
    // 1. Cloud notification (if LTE is available)
    // publishErrorToCloud("STM_COMMUNICATION_FAILURE");
    
    // 2. Local indicators (LED, buzzer, etc.)
    // setErrorLED(true);
    
    // 3. Log critical error
    ESP_LOGE(TAG, "SYSTEM ERROR NOTIFICATION: STM communication failure");
}

void checkCommunicationRecovery() {
    // Communication recovered - reset timeout counters
    consecutiveTimeouts = 0;
    
    if (currentEspState == SystemState::COMMUNICATION_ERROR) {
        ESP_LOGI(TAG, "Communication recovered - returning to normal state");
        currentEspState = SystemState::NORMAL;
        
        // TODO: Add any recovery procedures here
        // restoreNormalOperation();
    } else if (currentEspState == SystemState::CRITICAL_ERROR) {
        ESP_LOGW(TAG, "Communication recovered but system still in critical error");
        ESP_LOGW(TAG, "Manual intervention may be required for full recovery");
        // Critical error requires manual recovery
    }
}

// System state query functions
SystemState getCurrentSystemState() {
    return currentEspState;
}

bool isSystemInErrorState() {
    return (currentEspState == SystemState::COMMUNICATION_ERROR || 
            currentEspState == SystemState::CRITICAL_ERROR);
}

bool isCommunicationHealthy() {
    return (currentEspState == SystemState::NORMAL && consecutiveTimeouts == 0);
}

bool manualRecoveryFromCriticalError() {
    if (currentEspState == SystemState::CRITICAL_ERROR) {
        ESP_LOGW(TAG, "Manual recovery initiated from critical error state");
        currentEspState = SystemState::NORMAL;
        consecutiveTimeouts = 0;
        lastCommunicationError = 0;
        
        // TODO: Add system reinitialization if needed
        // reinitializeSystem();
        
        ESP_LOGI(TAG, "System manually recovered from critical error");
        return true;
    }
    return false;
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

// const char* getLastErrorString() {
//     if (currentPendingCommand.state == CommandState::TIMEOUT) {
//         return "Command timeout - STM did not respond";
//     } else if (currentPendingCommand.state == CommandState::ERROR) {
//         return "Command error - Invalid response or communication failure";
//     } else if (lastErrorCode > 0) {
//         const char* errorMessages[] = {
//             "No error",                 // 0
//             "Initialization Failed",    // 1
//             "Communication Failed",     // 2
//             "SD Card Failed",          // 3
//             "Voltage Failed"           // 4
//         };
        
//         if (lastErrorCode >= 1 && lastErrorCode <= 4) {
//             return errorMessages[lastErrorCode];
//         } else {
//             return "Unknown STM error";
//         }
//     }
//     return "No error";
// }

bool isCommandInProgress() {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command != 0 && pendingCommands[i].state == CommandState::SENT) {
            return true;
        }
    }
    return false;
}

void clearPendingCommand() {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        pendingCommands[i].reset();
    }
    ESP_LOGI(TAG, "All pending commands cleared");
}

uint32_t getLastCommandTimestamp() {
    uint32_t latestTimestamp = 0;
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command != 0 && pendingCommands[i].sentTimestamp > latestTimestamp) {
            latestTimestamp = pendingCommands[i].sentTimestamp;
        }
    }
    return latestTimestamp;
}

uint8_t getCommandRetryCount() {
    uint8_t maxRetries = 0;
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command != 0 && pendingCommands[i].retryCount > maxRetries) {
            maxRetries = pendingCommands[i].retryCount;
        }
    }
    return maxRetries;
}

// =============================================================================
// Command Name Lookup Function
// =============================================================================

const char* getCommandName(uint8_t command) {
    switch (command) {
        // INIT Commands (0x1X)
        case initTempSleep:     return "INIT: Sleep Temperature";
        case initTempWaiting:   return "INIT: Waiting Temperature";
        case initTempForceUp:   return "INIT: Operating Temperature";
        case initTempHeatPad:   return "INIT: Heat Pad Temperature (DEPRECATED)";
        case initPWMCoolFan:    return "INIT: Cooling Fan PWM";
        case initTout:          return "INIT: Timeout Settings";
        case initTempLimit:     return "INIT: Upper Temperature Limit";
        case initSpk:           return "INIT: Speaker Volume";
        case initDelay:         return "INIT: Detection Delay";
        case initGyroAct:       return "INIT: Heating Angle (IMU Active)";
        case initGyroRel:       return "INIT: Cooling Angle (IMU Relative)";
        case initMode:          return "INIT: Mode Change";
        
        // REQUEST Commands (0x3X)
        case reqTempSleep:      return "REQUEST: Sleep Temperature";
        case reqTempWaiting:    return "REQUEST: Waiting Temperature";
        case reqTempForceUp:    return "REQUEST: Operating Temperature";
        case reqTempHeatPad:    return "REQUEST: Heat Pad Temperature (DEPRECATED)";
        case reqUpperTemp:      return "REQUEST: Upper Temperature Limit";
        case reqPWMCoolFan:     return "REQUEST: Cooling Fan Level";
        case reqTimeout:        return "REQUEST: Timeout Settings";
        case reqSpk:            return "REQUEST: Speaker Volume";
        case reqDelay:          return "REQUEST: Detection Delay";
        case reqGyroAct:        return "REQUEST: Heating Angle (IMU Active)";
        case reqGyroRel:        return "REQUEST: Cooling Angle (IMU Relative)";
        case reqMode:           return "REQUEST: Current Mode";
        
        // CONTROL Commands (0x5X)
        case ctrlReset:         return "CONTROL: Reset Device";
        case ctrlMode:          return "CONTROL: Set Device Mode";
        case ctrlSpkVol:        return "CONTROL: Set Speaker Volume";
        case ctrlFanOn:         return "CONTROL: Set Fan State";
        case ctrlFanPWM:        return "CONTROL: Set Fan Speed";
        case ctrlCoolFanOn:     return "CONTROL: Set Cooling Fan State";
        case ctrlCoolFanPWM:    return "CONTROL: Set Cooling Fan Level";
        case ctrlHeatPadOn:     return "CONTROL: Set Heat Pad State (DEPRECATED)";
        case ctrlHeatPadTemp:   return "CONTROL: Set Heat Pad Temperature (DEPRECATED)";
        case ctrlForceUp:       return "CONTROL: Force Up Mode";
        case ctrlForceDown:     return "CONTROL: Force Down Mode";
        case ctrlSleeping:      return "CONTROL: Sleep Mode";
        
        // STATUS Commands (0x7X)
        case statMessage:       return "STATUS: Sensor Data";
        
        // EVENT Commands (0x8X)
        case evtInitStart:      return "EVENT: Initialization Started";
        case evtInitResult:     return "EVENT: Initialization Complete";
        case evtMode:           return "EVENT: Device Mode Changed";
        
        // ERROR Commands (0x9X)
        case errInit:           return "ERROR: Initialization Failed";
        
        default:
            // Check command type patterns
            uint8_t cmdType = command & 0xF0;
            switch (cmdType) {
                case 0x10: return "INIT: Unknown Command";
                case 0x30: return "REQUEST: Unknown Command";
                case 0x50: return "CONTROL: Unknown Command";
                case 0x70: return "STATUS: Unknown Command";
                case 0x80: return "EVENT: Unknown Command";
                case 0x90: return "ERROR: Unknown Command";
                default:   return "UNKNOWN: Invalid Command";
            }
    }
}

// =============================================================================
// INIT Command Functions (0x10-0x21) - Device Parameter Initialization
// =============================================================================

// Temperature Initialization Functions
bool initSleepTemperature(uint8_t tempInt, uint8_t tempDec) {
    if (tempInt > 100 || tempDec > 99) {
        ESP_LOGW(TAG, "Invalid temperature - %d.%02d", tempInt, tempDec);
        return false;
    }
    
    byte data[] = {
        tempInt,    // Temperature integer part
        tempDec     // Temperature decimal part
    };
    
    // Build message to get HEX packet for logging
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t messageSize = buildMessage(buffer, initTempSleep, data, 2);
    
    // Create HEX dump string
    char hexDump[64] = {0};
    for (size_t i = 0; i < messageSize && i < 16; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", buffer[i]);
    }
    
    bool success = sendCommandFireAndForget(initTempSleep, data, 2);
    if (success) {
        ESP_LOGI(TAG, "[INIT] ESP → STM (0x%02X), Sleep Temp: %d.%02d°C [%s]", 
                 initTempSleep, tempInt, tempDec, hexDump);
    }
    return success;
}

bool initWaitingTemperature(uint8_t tempInt, uint8_t tempDec) {
    if (tempInt > 100 || tempDec > 99) {
        ESP_LOGW(TAG, "Invalid temperature - %d.%02d", tempInt, tempDec);
        return false;
    }
    
    byte data[] = {
        tempInt,    // Temperature integer part
        tempDec     // Temperature decimal part
    };
    
    // Build message to get HEX packet for logging
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t messageSize = buildMessage(buffer, initTempWaiting, data, 2);
    
    // Create HEX dump string
    char hexDump[64] = {0};
    for (size_t i = 0; i < messageSize && i < 16; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", buffer[i]);
    }
    
    bool success = sendCommandAsync(initTempWaiting, data, 2);
    if (success) {
        ESP_LOGI(TAG, "[INIT] ESP → STM (0x%02X), Waiting Temp: %d.%02d°C [%s]", 
                 initTempWaiting, tempInt, tempDec, hexDump);
    }
    return success;
}

bool initOperatingTemperature(uint8_t tempInt, uint8_t tempDec) {
    if (tempInt > 100 || tempDec > 99) {
        ESP_LOGW(TAG, "Invalid temperature - %d.%02d", tempInt, tempDec);
        return false;
    }
    
    byte data[] = {
        tempInt,    // Temperature integer part
        tempDec     // Temperature decimal part
    };
    
    // Build message to get HEX packet for logging
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t messageSize = buildMessage(buffer, initTempForceUp, data, 2);
    
    // Create HEX dump string
    char hexDump[64] = {0};
    for (size_t i = 0; i < messageSize && i < 16; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", buffer[i]);
    }
    
    bool success = sendCommandFireAndForget(initTempForceUp, data, 2);
    if (success) {
        ESP_LOGI(TAG, "[INIT] ESP → STM (0x%02X), Operating Temp: %d.%02d°C [%s]", 
                 initTempForceUp, tempInt, tempDec, hexDump);
    }
    return success;
}

bool initUpperTemperatureLimit(uint8_t tempInt, uint8_t tempDec) {
    // Uses command 0x14 (initTempLimit) - valid command
    
    if (tempInt > 100 || tempDec > 99) {
        ESP_LOGW(TAG, "Invalid temperature - %d.%02d", tempInt, tempDec);
        return false;
    }
    
    byte data[] = {
        tempInt,    // Temperature integer part
        tempDec     // Temperature decimal part
    };
    
    // Build message to get HEX packet for logging
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t messageSize = buildMessage(buffer, initTempLimit, data, 2);
    
    // Create HEX dump string
    char hexDump[64] = {0};
    for (size_t i = 0; i < messageSize && i < 16; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", buffer[i]);
    }
    
    bool success = sendCommandFireAndForget(initTempLimit, data, 2); // initTempLimit command
    if (success) {
        ESP_LOGI(TAG, "[INIT] ESP → STM (0x%02X), Upper Temp Limit: %d.%02d°C [%s]", 
                 initTempLimit, tempInt, tempDec, hexDump);
    }
    return success;
}

bool initOperatingTemperature(OperatingTempLevel level) {
    return initOperatingTemperature(static_cast<uint8_t>(level), 0);
}

// System Configuration Initialization Functions
bool initTimeoutConfiguration(uint16_t forceUpTimeout, uint16_t forceOnTimeout, uint16_t forceDownTimeout, uint16_t waitingTimeout) {
    if (forceUpTimeout == 0 || forceOnTimeout == 0 || forceDownTimeout == 0 || waitingTimeout == 0) {
        ESP_LOGW(TAG, "Invalid timeout configuration - values cannot be zero");
        return false;
    }
    
    byte data[] = {
        (uint8_t)(forceUpTimeout >> 8),         // DATA1: ForceUp timeout HIGH byte
        (uint8_t)(forceUpTimeout & 0xFF),       // DATA2: ForceUp timeout LOW byte
        (uint8_t)(forceOnTimeout >> 8),         // DATA3: ForceOn timeout HIGH byte
        (uint8_t)(forceOnTimeout & 0xFF),       // DATA4: ForceOn timeout LOW byte
        (uint8_t)(forceDownTimeout >> 8),       // DATA5: ForceDown timeout HIGH byte
        (uint8_t)(forceDownTimeout & 0xFF),     // DATA6: ForceDown timeout LOW byte
        (uint8_t)(waitingTimeout >> 8),         // DATA7: Waiting timeout HIGH byte
        (uint8_t)(waitingTimeout & 0xFF)        // DATA8: Waiting timeout LOW byte
    };
    
    // Build message to get HEX packet for logging
    uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
    size_t messageSize = buildMessage(buffer, initTout, data, 8);
    
    // Create HEX dump string
    char hexDump[64] = {0};
    for (size_t i = 0; i < messageSize && i < 16; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), "%02X ", buffer[i]);
    }
    
    bool success = sendCommandFireAndForget(initTout, data, 8);
    if (success) {
        ESP_LOGI(TAG, "[INIT] ESP → STM (0x%02X), Timeout Config: Up=%d, On=%d, Down=%d, Wait=%d [%s]", 
                 initTout, forceUpTimeout, forceOnTimeout, forceDownTimeout, waitingTimeout, hexDump);
    }
    return success;
}

bool initSpeakerVolume(uint8_t volume_0_to_10) {
    if (volume_0_to_10 > 10) {
        ESP_LOGW(TAG, "Invalid speaker volume: %d (max: 10)", volume_0_to_10);
        return false;
    }
    
    byte data[] = {volume_0_to_10};
    
    bool success = sendCommandAsync(initSpk, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Speaker volume initialization set to %d - Command: 0x%02X (%s)", 
                 volume_0_to_10, initSpk, getCommandName(initSpk));
    }
    return success;
}

bool initCoolingFanPWM(uint8_t currentLevel, uint8_t maxLevel) {
    if (currentLevel > maxLevel) {
        ESP_LOGW(TAG, "Invalid cooling fan PWM - current level (%d) > max level (%d)", 
                 currentLevel, maxLevel);
        return false;
    }
    if (maxLevel > 10) {
        ESP_LOGW(TAG, "Invalid cooling fan max level: %d (should be 0-10)", maxLevel);
        return false;
    }
    
    byte data[] = {
        currentLevel,  // DATA1: Current level
        maxLevel      // DATA2: Max level
    };
    
    bool success = sendCommandAsync(initPWMCoolFan, data, 2);
    if (success) {
        ESP_LOGI(TAG, "Cooling fan PWM init - current: %d, max: %d - Command: 0x%02X (%s)", 
                 currentLevel, maxLevel, initPWMCoolFan, getCommandName(initPWMCoolFan));
    }
    return success;
}

bool initForeDownDelay(uint8_t poseDetectionDelay, uint8_t objectDetectionDelay) {
    if (poseDetectionDelay == 0 || objectDetectionDelay == 0) {
        ESP_LOGW(TAG, "Invalid detection delay configuration - values cannot be zero");
        return false;
    }
    
    byte data[] = {
        poseDetectionDelay,     // Pose detection delay (100ms units)
        objectDetectionDelay    // Object detection delay (100ms units)
    };
    
    bool success = sendCommandAsync(initDelay, data, 2);
    if (success) {
        ESP_LOGI(TAG, "Detection delay configuration set - pose: %d×100ms, object: %d×100ms", 
                 poseDetectionDelay, objectDetectionDelay);
    }
    return success;
}

bool initGyroActiveAngle(uint8_t activeAngle) {
    if (activeAngle > 180) {
        ESP_LOGW(TAG, "Invalid gyro active angle: %d (max: 180)", activeAngle);
        return false;
    }
    
    byte data[] = {activeAngle};
    
    bool success = sendCommandAsync(initGyroAct, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Gyro active angle set to %d degrees - Command: 0x%02X (%s)", 
                 activeAngle, initGyroAct, getCommandName(initGyroAct));
    }
    return success;
}

bool initGyroRelativeAngle(uint8_t relativeAngle) {
    if (relativeAngle > 180) {
        ESP_LOGW(TAG, "Invalid gyro relative angle: %d (max: 180)", relativeAngle);
        return false;
    }
    
    byte data[] = {relativeAngle};
    
    bool success = sendCommandAsync(initGyroRel, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Gyro relative angle set to %d degrees - Command: 0x%02X (%s)", 
                 relativeAngle, initGyroRel, getCommandName(initGyroRel));
        ESP_LOGI(TAG, "Actual cooling angle = active_angle + relative_angle");
    }
    return success;
}

bool initDeviceMode(uint8_t mode) {
    if (mode > 1) {
        ESP_LOGW(TAG, "Invalid initial mode: %d (0: AI mode, 1: IMU mode)", mode);
        return false;
    }
    
    byte data[] = {mode};
    
    bool success = sendCommandAsync(initMode, data, 1);
    if (success) {
        const char* modeNames[] = {"AI mode", "IMU mode"};
        ESP_LOGI(TAG, "Initial mode set to %s (%d) - Command: 0x%02X (%s)", 
                 modeNames[mode], mode, initMode, getCommandName(initMode));
    }
    return success;
}

// =============================================================================
// REQUEST Command Functions (0x30-0x41) - Parameter Query Functions
// =============================================================================

// Temperature Request Functions
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

bool requestTimeoutConfiguration() {
    return sendCommandAsync(reqTimeout);
}

// System Configuration Request Functions
bool requestCoolingFanLevel() {
    return sendCommandAsync(reqPWMCoolFan);
}

bool requestSpeakerVolume() {
    return sendCommandAsync(reqSpk);
}

bool requestDetectionDelay() {
    return sendCommandAsync(reqDelay);
}

bool requestGyroActiveAngle() {
    return sendCommandAsync(reqGyroAct);
}

bool requestGyroRelativeAngle() {
    return sendCommandAsync(reqGyroRel);
}

bool requestCurrentMode() {
    return sendCommandAsync(reqMode);
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
    success &= requestTimeoutConfiguration();
    vTaskDelay(10 / portTICK_RATE_MS);
    // requestHeatPadLevel() removed - deprecated functionality
    success &= requestCoolingFanLevel();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestSpeakerVolume();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestDetectionDelay();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestGyroActiveAngle();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestGyroRelativeAngle();
    vTaskDelay(10 / portTICK_RATE_MS);
    success &= requestCurrentMode();
    
    ESP_LOGI(TAG, "All parameter requests sent %s", success ? "successfully" : "with errors");
    return success;
}

// =============================================================================
// CONTROL Command Functions (0x50-0x61) - Real-time Device Control
// =============================================================================

// Utility Control Functions
bool resetDevice() {
    bool success = sendCommandAsync(ctrlReset);
    if (success) {
        ESP_LOGI(TAG, "Device reset command sent");
    }
    return success;
}

// Device Mode Control Functions
bool setDeviceMode(DeviceMode mode) {
    byte data[] = {static_cast<byte>(mode)};
    
    bool success = sendCommandAsync(ctrlMode, data, 1);
    if (success) {
        const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "IMU", "TEST", "ERROR"};
        ESP_LOGI(TAG, "Device mode set to %s - Command: 0x%02X (%s)", 
                 modeNames[static_cast<int>(mode)], ctrlMode, getCommandName(ctrlMode));
    }
    return success;
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

// Actuator Control Functions - Fan
bool setBlowerFanState(bool enabled) {
    byte data[] = {enabled ? 1 : 0};
    
    bool success = sendCommandAsync(ctrlFanOn, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Fan %s", enabled ? "ON" : "OFF");
    }
    return success;
}

bool setBlowerFanSpeed(uint8_t speed_0_to_3) {
    if (speed_0_to_3 > 3) {
        ESP_LOGW(TAG, "Invalid fan speed: %d (max: 3)", speed_0_to_3);
        return false;
    }
    
    byte data[] = {speed_0_to_3};
    
    bool success = sendCommandAsync(ctrlFanPWM, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Fan speed set to %d", speed_0_to_3);
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

// Actuator Control Functions - Heat Pad (DEPRECATED)
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

// Force Mode Control Functions (Korean Protocol)
bool setForceUpMode() {
    byte data[] = {1}; // Always send 1 as specified
    
    bool success = sendCommandAsync(ctrlForceUp, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Force Up mode activated - Command: 0x%02X (%s)", 
                 ctrlForceUp, getCommandName(ctrlForceUp));
    }
    return success;
}

bool setForceDownMode() {
    byte data[] = {1}; // Always send 1 as specified
    
    bool success = sendCommandAsync(ctrlForceDown, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Force Down mode activated - Command: 0x%02X (%s)", 
                 ctrlForceDown, getCommandName(ctrlForceDown));
    }
    return success;
}

bool setSleepingMode() {
    byte data[] = {1}; // Always send 1 as specified
    
    bool success = sendCommandAsync(ctrlSleeping, data, 1);
    if (success) {
        ESP_LOGI(TAG, "Sleep mode activated - Command: 0x%02X (%s)", 
                 ctrlSleeping, getCommandName(ctrlSleeping));
    }
    return success;
}

// setPoseDetectionMode function removed - ctrlPose (0x5A) doesn't exist in specification

// =============================================================================
// STATUS Command Functions (0x70-0x7F) - Status and Sensor Data
// =============================================================================

// Sensor Data Access Function
SensorReading getCurrentSensorData() {
    SensorReading reading = {};
    
    // Copy data from allData to SensorReading structure
    reading.leftGyro[0] = allData.L_gyro[0];
    reading.leftGyro[1] = allData.L_gyro[1];
    reading.leftGyro[2] = allData.L_gyro[2];
    reading.leftAccel[0] = allData.L_accel[0];
    reading.leftAccel[1] = allData.L_accel[1];
    reading.leftAccel[2] = allData.L_accel[2];
    reading.rightGyro[0] = allData.R_gyro[0];
    reading.rightGyro[1] = allData.R_gyro[1];
    reading.rightGyro[2] = allData.R_gyro[2];
    reading.rightAccel[0] = allData.R_accel[0];
    reading.rightAccel[1] = allData.R_accel[1];
    reading.rightAccel[2] = allData.R_accel[2];
    reading.leftPressure = allData.L_ads;
    reading.rightPressure = allData.R_ads;
    reading.outsideTemp = allData.outTemp;
    reading.boardTemp = allData.boardTemp;
    reading.actuatorTemp = allData.lmaTemp;
    reading.actuatorDisplacement = allData.lmaLength;
    reading.objectDistance = allData.objDistance;
    reading.batteryVoltage = allData.battery;
    reading.leftIMUEvent = allData.leftIMUEvent;
    reading.rightIMUEvent = allData.rightIMUEvent;
    
    return reading;
}

// =============================================================================
// EVENT Command Functions - Event Processing and Status
// =============================================================================

DeviceMode getCurrentMode() {
    // Return the actual tracked device mode updated by event messages
    return currentStmMode;
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
// ERROR Command Functions - Error Handling
// =============================================================================

uint8_t getLastErrorCode() {
    // Return the last error code received (0 = no error)
    return lastErrorCode;
}

// =============================================================================
// System Configuration Getter Functions
// =============================================================================

// Temperature Configuration Getters
float getSleepTemperature() {
    return systemConfig.sleepTemp;
}

float getWaitingTemperature() {
    return systemConfig.waitingTemp;
}

float getOperatingTemperature() {
    return systemConfig.operatingTemp;
}

float getUpperTemperatureLimit() {
    return systemConfig.upperTempLimit;
}

// Fan Configuration Getters
uint8_t getCoolingFanLevel() {
    return systemConfig.coolingFanLevel;
}

uint8_t getMaxCoolingFanLevel() {
    return systemConfig.maxCoolingFanLevel;
}

// Audio Configuration Getters
uint8_t getSpeakerVolume() {
    return systemConfig.speakerVolume;
}

// Timeout Configuration Getters
uint16_t getForceUpTimeout() {
    return systemConfig.forceUpTimeout;
}

uint16_t getForceOnTimeout() {
    return systemConfig.forceOnTimeout;
}

uint16_t getForceDownTimeout() {
    return systemConfig.forceDownTimeout;
}

uint16_t getWaitingTimeout() {
    return systemConfig.waitingTimeout;
}

// Detection Configuration Getters
uint8_t getPoseDetectionDelay() {
    return systemConfig.poseDetectionDelay;
}

uint8_t getObjectDetectionDelay() {
    return systemConfig.objectDetectionDelay;
}

// Gyro Configuration Getters (Korean Protocol)
uint8_t getGyroActiveAngle() {
    return systemConfig.gyroActiveAngle;
}

uint8_t getGyroRelativeAngle() {
    return systemConfig.gyroRelativeAngle;
}

// Mode Configuration Getters (Korean Protocol)
uint8_t getCurrentModeValue() {
    return systemConfig.currentModeValue;
}


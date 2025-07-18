#include "uartMaster.h"

static const char TAG[] = __FILE__;

// ===== UART Configuration =====
// UART0 (Serial): PC Python 테스트 프로토콜 통신용
// ESP_LOG: 디버깅 로그 출력용

// ESP_LOG handles debug output automatically
// STMSerial now directly uses the standardized serial definition from globals.h

// Global variables for message parsing
static MessageState currentState = MessageState::WAITING_START;
static ProtocolMessage currentMessage = {};
static uint8_t messageBuffer[USART_MESSAGE_MAXIMUM_LENGTH];
static size_t bufferIndex = 0;
static size_t expectedDataLength = 0;

// Global variables for sensor data storage
static bool sensorDataAvailable = false;

// Global variables for device state tracking
static DeviceMode currentDeviceMode = DeviceMode::WAITING;  // Default mode
static uint8_t lastErrorCode = 0;  // Last error code received (0 = no error)

// Global variables for TinyML processing
static uint8_t tinyMLInferenceResult = 0;
static uint8_t finalResult = 0;

// Global variables for multi-command state management (ESP→STM)
static PendingCommand pendingCommands[MAX_PENDING_COMMANDS];
static uint8_t nextCommandId = 1;

// Global variables for system state tracking
static SystemState currentSystemState = SystemState::NORMAL;
static uint32_t lastCommunicationError = 0;
static uint8_t consecutiveTimeouts = 0;

// Variables for message construction
byte LEN;
uint8_t CHKSUM;

// =============================================================================
// Multi-Command Management Helper Functions
// =============================================================================

// Find empty slot in pending commands array
static int findEmptySlot() {
    for (int i = 0; i < MAX_PENDING_COMMANDS; i++) {
        if (pendingCommands[i].command == 0) {
            return i;
        }
    }
    return -1; // No empty slot found
}

// Find command slot by command code
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
        ESP_LOGI(TAG, "[ESP→STM] ACK 0x%02X %s (%zu bytes)", 
                 receivedCommand, getCommandName(receivedCommand), index);
    }
}

void nackResponse() {
    // NACK is not defined in protocol - no response sent (silence = NACK)
    ESP_LOGW(TAG, "NACK: No response sent as per protocol");
}

// Stack-based message construction function - improved efficiency
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
    
    // Log outgoing message
    ESP_LOGI(TAG, "[ESP→STM] 0x%02X %s (%d bytes)", command, getCommandName(command), messageSize);
    
    // Hex dump of outgoing packet
    char hexDump[256] = {0};
    for (size_t i = 0; i < messageSize && i < 32; i++) {
        snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), 
                 "%02X ", buffer[i]);
    }
    ESP_LOGV(TAG, "[ESP→STM] Raw: %s", hexDump);
    
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
        ESP_LOGI(TAG, "Sensor data (%d bytes)", 
                 currentMessage.dataLength);
        parseSensorData(currentMessage.data, currentMessage.dataLength);
        sensorDataAvailable = true;
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
        if (modeValue <= 5) {  // Valid mode range: 0-5
            currentDeviceMode = static_cast<DeviceMode>(modeValue);
            const char* modeNames[] = {"SLEEP", "WAITING", "FORCE_UP", "FORCE_ON", "FORCE_DOWN", "ERROR"};
            ESP_LOGI(TAG, "Mode: %s (%d)", modeNames[modeValue], modeValue);
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
            ESP_LOGI(TAG, "STM init complete");
            break;
        case evtMode:        // 0x82
            ESP_LOGI(TAG, "Mode change event");
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
    if (currentMessage.dataLength > 0) {
        uint8_t errorCode = currentMessage.data[0];
        lastErrorCode = errorCode;  // Update error state
        
        // 간결하고 직관적인 에러 메시지
        const char* errorMessages[] = {
            "ERR_UNKNOWN",      // 0
            "ERR_INIT_FAIL",    // 1 - 초기화 실패
            "ERR_COMM_FAIL",    // 2 - 통신 실패  
            "ERR_SD_FAIL",      // 3 - SD 카드 실패
            "ERR_VOLTAGE_FAIL"  // 4 - 전압 실패
        };
        
        const char* errorDesc[] = {
            "Unknown error",
            "System initialization failed",
            "Communication timeout/error", 
            "SD card read/write failed",
            "Power supply voltage error"
        };
        
        if (errorCode >= 1 && errorCode <= 4) {
            ESP_LOGI(TAG, "ERROR message ACK [Code:%d]", errorCode);
        } else {
            ESP_LOGI(TAG, "ERROR message ACK [Invalid Code:%d]", errorCode);
        }
    } else {
        ESP_LOGI(TAG, "ERROR message ACK [No Data]");
    }
    sendResponse(currentMessage.command);
    
    // Python 테스트 모드에서는 시스템 상태 업데이트 생략
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
    // Left pressure sensor (2 bytes)
    if (index + 1 < length) {
        allData.L_ads = (data[index] << 8) | data[index + 1];
        index += PRESSURE_DATA_SIZE;
    }
    // Right pressure sensor (2 bytes)
    if (index + 1 < length) {
        allData.R_ads = (data[index] << 8) | data[index + 1];
        index += PRESSURE_DATA_SIZE;
    }
    
    // Temperature sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Outside temperature (2 bytes)
    if (index + 1 < length) {
        allData.outTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Board temperature (2 bytes)
    if (index + 1 < length) {
        allData.boardTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Actuator temperature (2 bytes)
    if (index + 1 < length) {
        allData.lmaTemp = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    
    // Other sensors (6 bytes total: 3 sensors * 2 bytes each)
    // Actuator displacement (2 bytes)
    if (index + 1 < length) {
        allData.lmaLength = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Object distance (2 bytes)
    if (index + 1 < length) {
        allData.objDistance = (data[index] << 8) | data[index + 1];
        index += TEMPERATURE_DATA_SIZE;
    }
    // Battery voltage (2 bytes)
    if (index + 1 < length) {
        allData.battery = (data[index] << 8) | data[index + 1];
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

void usartMasterHandler(void *pvParameters) {
    static bool testExecuted = false;
    static uint32_t startTime = xTaskGetTickCount();

    while (1) {
        // // Check if 5 seconds have passed and test hasn't been executed yet
        // if (!testExecuted && (xTaskGetTickCount() - startTime) > (5000 / portTICK_PERIOD_MS)) {
        //     ESP_LOGI(TAG, "=== ESP→STM 통신 테스트 시작 (within usartMasterHandler) ===");
        //     testESPtoSTMCommunication();
        //     ESP_LOGI(TAG, "=== ESP→STM 통신 테스트 완료 ===");
        //     testExecuted = true;
        // }
        
        while (STMSerial.available()) {
            uint8_t incomingByte = STMSerial.read();
            
            // Prevent buffer overflow
            if (bufferIndex >= USART_MESSAGE_MAXIMUM_LENGTH) {
                ESP_LOGW(TAG, "[STM-ERR] Buffer overflow, resetting parser");
                resetParser();
                continue;
            }
            
            // State machine based parsing
            if (parseIncomingByte(incomingByte)) {
                // Message complete - create hex dump and log with command symbol
                char hexDump[256] = {0};
                for (size_t i = 0; i < bufferIndex && i < 32; i++) {
                    snprintf(hexDump + strlen(hexDump), sizeof(hexDump) - strlen(hexDump), 
                             "%02X ", messageBuffer[i]);
                }
                
                // Single line log with packet and command symbol
                ESP_LOGI(TAG, "[STM→ESP] %s→ %s", hexDump, getCommandName(currentMessage.command));
                
                // Message complete, verify checksum
                if (!currentMessage.isValid()) {
                    ESP_LOGW(TAG, "[STM-ERR] Invalid format: STM=0x%02X (expect 0x%02X), DIR=0x%02X (expect 0x%02X), ETX=0x%02X (expect 0x%02X)", 
                             currentMessage.stm, STM, currentMessage.direction, MSG_REQUEST, 
                             currentMessage.etx, ETX);
                    resetParser();
                    continue;
                }
                
                if (!checkMessage(messageBuffer, bufferIndex)) {
                    uint8_t calculatedChecksum = calculateChecksum(messageBuffer, bufferIndex);
                    ESP_LOGW(TAG, "[STM-ERR] Checksum FAIL: received=0x%02X, calculated=0x%02X", 
                             currentMessage.checksum, calculatedChecksum);
                    resetParser();
                    continue;
                }
                
                // Handle according to message type and command classification
                if (currentMessage.direction == MSG_REQUEST) {
                    // Classify and process message by command type
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
                            ESP_LOGW(TAG, "Unknown CMD: 0x%02X", currentMessage.command);
                            break;
                    }
                } else {
                    ESP_LOGW(TAG, "[STM-ERR] Invalid direction: 0x%02X (expected: 0x%02X)", 
                             currentMessage.direction, MSG_REQUEST);
                }
                
                resetParser();
            }
        }
        
        // Check for command timeouts and handle critical errors
        processCommandTimeout(); 

        vTaskDelay(10 / portTICK_RATE_MS); // More frequent checks for better responsiveness
    }
}

bool initUartMaster() {
    // === UART 초기화: 하드웨어 연결 모드에 따라 STMSerial 설정 ===
#ifdef STM_HARDWARE_CONNECTED
    // STM 하드웨어 연결 모드: STMSerial = UART2 (GPIO 15/16) → STM 통신
    STMSerial.begin(115200, SERIAL_8N1, ESP_U2_RXD, ESP_U2_TXD);
    ESP_LOGI(TAG, "STM Hardware Mode: STMSerial initialized on UART2 (GPIO %d/%d)", ESP_U2_RXD, ESP_U2_TXD);
#else
    // Python 테스트 모드: STMSerial = UART0 (USB) → Python 통신
    STMSerial.begin(115200);
    ESP_LOGI(TAG, "Python Test Mode: STMSerial initialized on UART0 (USB)");
#endif
    
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
        if (currentSystemState != SystemState::NORMAL) {
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
        
        ESP_LOGI(TAG, "REQUEST response received for command 0x%02X (%s) (slot %d)", 
                 currentMessage.command, getCommandName(currentMessage.command), slot);
        
        // Communication recovery check
        if (currentSystemState != SystemState::NORMAL) {
            checkCommunicationRecovery();
        }
        
        // Process response data based on command type
        switch (currentMessage.command) {
            case reqTempSleep:      // 0x30
                if (currentMessage.dataLength >= 2) {
                    systemConfig.sleepTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "Sleep temp response: %.1f°C", systemConfig.sleepTemp);
                } else {
                    ESP_LOGW(TAG, "Invalid sleep temp response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqTempWaiting:    // 0x31
                if (currentMessage.dataLength >= 2) {
                    systemConfig.waitingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "Waiting temp response: %.1f°C", systemConfig.waitingTemp);
                } else {
                    ESP_LOGW(TAG, "Invalid waiting temp response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqTempForceUp:    // 0x32
                if (currentMessage.dataLength >= 2) {
                    systemConfig.operatingTemp = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "Operating temp response: %.1f°C", systemConfig.operatingTemp);
                } else {
                    ESP_LOGW(TAG, "Invalid operating temp response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqTempHeatPad:    // 0x33 - DEPRECATED
                ESP_LOGW(TAG, "reqTempHeatPad (0x33) is deprecated and ignored");
                break;
                
            case reqUpperTemp:      // 0x34
                if (currentMessage.dataLength >= 2) {
                    systemConfig.upperTempLimit = currentMessage.data[0] + (currentMessage.data[1] / 10.0f);
                    ESP_LOGI(TAG, "Upper temp limit response: %.1f°C", systemConfig.upperTempLimit);
                } else {
                    ESP_LOGW(TAG, "Invalid upper temp limit response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqPWMCoolFan:     // 0x35
                if (currentMessage.dataLength >= 2) {
                    systemConfig.coolingFanLevel = currentMessage.data[0];
                    systemConfig.maxCoolingFanLevel = currentMessage.data[1];
                    ESP_LOGI(TAG, "Cooling fan level response: %d/%d", systemConfig.coolingFanLevel, systemConfig.maxCoolingFanLevel);
                } else {
                    ESP_LOGW(TAG, "Invalid cooling fan level response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqTimeout:        // 0x36
                if (currentMessage.dataLength >= 8) {
                    systemConfig.forceUpTimeout = (currentMessage.data[0] << 8) | currentMessage.data[1];
                    systemConfig.forceOnTimeout = (currentMessage.data[2] << 8) | currentMessage.data[3];
                    systemConfig.forceDownTimeout = (currentMessage.data[4] << 8) | currentMessage.data[5];
                    systemConfig.waitingTimeout = (currentMessage.data[6] << 8) | currentMessage.data[7];
                    ESP_LOGI(TAG, "Timeout response: ForceUp=%d, ForceOn=%d, ForceDown=%d, Waiting=%d", 
                             systemConfig.forceUpTimeout, systemConfig.forceOnTimeout, systemConfig.forceDownTimeout, systemConfig.waitingTimeout);
                } else {
                    ESP_LOGW(TAG, "Invalid timeout response length: %d (expected 8)", currentMessage.dataLength);
                }
                break;
                
            case reqSpk:            // 0x37
                if (currentMessage.dataLength >= 1) {
                    systemConfig.speakerVolume = currentMessage.data[0];
                    ESP_LOGI(TAG, "Speaker volume response: %d", systemConfig.speakerVolume);
                } else {
                    ESP_LOGW(TAG, "Invalid speaker volume response length: %d", currentMessage.dataLength);
                }
                break;
                
            case reqDelay:          // 0x38
                if (currentMessage.dataLength >= 2) {
                    systemConfig.poseDetectionDelay = currentMessage.data[0];    // 100ms units
                    systemConfig.objectDetectionDelay = currentMessage.data[1];  // 100ms units
                    ESP_LOGI(TAG, "Detection delay response: Pose=%d00ms, Object=%d00ms", 
                             systemConfig.poseDetectionDelay, systemConfig.objectDetectionDelay);
                } else {
                    ESP_LOGW(TAG, "Invalid detection delay response length: %d", currentMessage.dataLength);
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
        if (currentSystemState != SystemState::NORMAL) {
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
    
    ESP_LOGI(TAG, "Command 0x%02X (%s) sent asynchronously (slot %d)", 
             command, getCommandName(command), slot);
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
    if (currentSystemState == SystemState::NORMAL) {
        currentSystemState = SystemState::COMMUNICATION_ERROR;
        ESP_LOGW(TAG, "System state changed to COMMUNICATION_ERROR");
    }
}

void enterCriticalErrorState() {
    currentSystemState = SystemState::CRITICAL_ERROR;
    
    ESP_LOGE(TAG, "=== ENTERING CRITICAL ERROR STATE ===");
    ESP_LOGE(TAG, "STM communication completely failed");
    ESP_LOGE(TAG, "Shutting down actuators for safety");
    
    // Execute emergency safety measures
    emergencyShutdownActuators();
    setSystemErrorMode();
    notifySystemError();
    
    ESP_LOGE(TAG, "=== SYSTEM IN SAFE MODE ===");
}

void emergencyShutdownActuators() {
    ESP_LOGE(TAG, "Emergency shutdown: Turning OFF all actuators");
    
    // Cannot send commands to STM due to communication failure
    // STM should implement its own safety timeout mechanism
    ESP_LOGW(TAG, "Cannot send shutdown commands to STM - communication failed");
    ESP_LOGW(TAG, "STM should implement its own safety timeout mechanism");
    
    // Local safety measures that don't require STM communication
    // TODO: Add any local GPIO-controlled safety measures here
    // Example: gpio_set_level(EMERGENCY_SHUTDOWN_PIN, 0);
    
    ESP_LOGE(TAG, "Local emergency shutdown completed");
}

void setSystemErrorMode() {
    // Set internal system state to error mode
    currentSystemState = SystemState::CRITICAL_ERROR;
    
    // Stop non-critical processing
    // TODO: Add any system-specific shutdown procedures
    // Example: stopTinyMLProcessing();
    // Example: stopSensorProcessing();
    
    ESP_LOGE(TAG, "System error mode activated");
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
    
    if (currentSystemState == SystemState::COMMUNICATION_ERROR) {
        ESP_LOGI(TAG, "Communication recovered - returning to normal state");
        currentSystemState = SystemState::NORMAL;
        
        // TODO: Add any recovery procedures here
        // restoreNormalOperation();
    } else if (currentSystemState == SystemState::CRITICAL_ERROR) {
        ESP_LOGW(TAG, "Communication recovered but system still in critical error");
        ESP_LOGW(TAG, "Manual intervention may be required for full recovery");
        // Critical error requires manual recovery
    }
}

// System state query functions
SystemState getCurrentSystemState() {
    return currentSystemState;
}

bool isSystemInErrorState() {
    return (currentSystemState == SystemState::COMMUNICATION_ERROR || 
            currentSystemState == SystemState::CRITICAL_ERROR);
}

bool isCommunicationHealthy() {
    return (currentSystemState == SystemState::NORMAL && consecutiveTimeouts == 0);
}

bool manualRecoveryFromCriticalError() {
    if (currentSystemState == SystemState::CRITICAL_ERROR) {
        ESP_LOGW(TAG, "Manual recovery initiated from critical error state");
        currentSystemState = SystemState::NORMAL;
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
        case initTempHeatPad:   return "INIT: Heat Pad Temperature";
        case initPWMCoolFan:    return "INIT: Cooling Fan PWM";
        case initTout:          return "INIT: Timeout Settings";
        case initDelay:         return "INIT: Detection Delay";
        case 0x14:              return "INIT: Upper Temperature Limit";
        
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
        case ctrlPose:          return "CONTROL: Set Detection Mode";
        
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
        ESP_LOGI(TAG, "Sleep temperature set to %.1f°C - Command: 0x%02X (%s)", 
                 targetTemp, initTempSleep, getCommandName(initTempSleep));
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
        ESP_LOGI(TAG, "Device mode set to %s - Command: 0x%02X (%s)", 
                 modeNames[static_cast<int>(mode)], ctrlMode, getCommandName(ctrlMode));
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

DeviceMode getCurrentMode() {
    // Return the actual tracked device mode updated by event messages
    return currentDeviceMode;
}

uint8_t getLastErrorCode() {
    // Return the last error code received (0 = no error)
    return lastErrorCode;
}

// =============================================================================
// System Configuration Getter Functions
// =============================================================================

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

uint8_t getCoolingFanLevel() {
    return systemConfig.coolingFanLevel;
}

uint8_t getMaxCoolingFanLevel() {
    return systemConfig.maxCoolingFanLevel;
}

uint8_t getSpeakerVolume() {
    return systemConfig.speakerVolume;
}

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

uint8_t getPoseDetectionDelay() {
    return systemConfig.poseDetectionDelay;
}

uint8_t getObjectDetectionDelay() {
    return systemConfig.objectDetectionDelay;
}

// =============================================================================
// ESP→STM Communication Test Functions
// =============================================================================

void testESPtoSTMCommunication() {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "=== ESP→STM 통신 테스트 시작 ===");
    ESP_LOGI(TAG, "======================================================");
    
    uint32_t testStartTime = millis();
    uint8_t totalTests = 0;
    uint8_t passedTests = 0;
    
    // 테스트 시작 전 대기
    ESP_LOGI(TAG, "테스트 시작 전 2초 대기...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // =================
    // 1. INIT 명령 테스트
    // =================
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== INIT 명령 테스트 (0x1X) ===");
    
    // 1-1. Sleep Temperature
    ESP_LOGI(TAG, "1-1. Sleep Temperature 설정 테스트");
    totalTests++;
    if (setSleepTemperature(25.5)) {
        ESP_LOGI(TAG, "✓ Sleep Temperature 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Sleep Temperature 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 1-2. Waiting Temperature
    ESP_LOGI(TAG, "1-2. Waiting Temperature 설정 테스트");
    totalTests++;
    if (setWaitingTemperature(30.0)) {
        ESP_LOGI(TAG, "✓ Waiting Temperature 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Waiting Temperature 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 1-3. Operating Temperature
    ESP_LOGI(TAG, "1-3. Operating Temperature 설정 테스트");
    totalTests++;
    if (setOperatingTemperature(35.0)) {
        ESP_LOGI(TAG, "✓ Operating Temperature 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Operating Temperature 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 1-4. Upper Temperature Limit
    ESP_LOGI(TAG, "1-4. Upper Temperature Limit 설정 테스트");
    totalTests++;
    if (setUpperTemperatureLimit(40.0)) {
        ESP_LOGI(TAG, "✓ Upper Temperature Limit 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Upper Temperature Limit 명령 전송 실패");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // ===================
    // 2. REQUEST 명령 테스트
    // ===================
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== REQUEST 명령 테스트 (0x3X) ===");
    
    // 2-1. Sleep Temperature Request
    ESP_LOGI(TAG, "2-1. Sleep Temperature 요청 테스트");
    totalTests++;
    if (requestSleepTemperature()) {
        ESP_LOGI(TAG, "✓ Sleep Temperature 요청 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Sleep Temperature 요청 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 2-2. Speaker Volume Request
    ESP_LOGI(TAG, "2-2. Speaker Volume 요청 테스트");
    totalTests++;
    if (requestSpeakerVolume()) {
        ESP_LOGI(TAG, "✓ Speaker Volume 요청 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Speaker Volume 요청 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 2-3. Cooling Fan Level Request
    ESP_LOGI(TAG, "2-3. Cooling Fan Level 요청 테스트");
    totalTests++;
    if (requestCoolingFanLevel()) {
        ESP_LOGI(TAG, "✓ Cooling Fan Level 요청 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Cooling Fan Level 요청 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 2-4. Detection Delay Request
    ESP_LOGI(TAG, "2-4. Detection Delay 요청 테스트");
    totalTests++;
    if (requestDetectionDelay()) {
        ESP_LOGI(TAG, "✓ Detection Delay 요청 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Detection Delay 요청 명령 전송 실패");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // ====================
    // 3. CONTROL 명령 테스트
    // ====================
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CONTROL 명령 테스트 (0x5X) ===");
    
    // 3-1. Device Mode - WAITING
    ESP_LOGI(TAG, "3-1. Device Mode → WAITING 테스트");
    totalTests++;
    if (setDeviceMode(DeviceMode::WAITING)) {
        ESP_LOGI(TAG, "✓ Device Mode WAITING 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Device Mode WAITING 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-2. Device Mode - FORCE_UP
    ESP_LOGI(TAG, "3-2. Device Mode → FORCE_UP 테스트");
    totalTests++;
    if (setDeviceMode(DeviceMode::FORCE_UP)) {
        ESP_LOGI(TAG, "✓ Device Mode FORCE_UP 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Device Mode FORCE_UP 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-3. Fan State Control
    ESP_LOGI(TAG, "3-3. Fan State ON 테스트");
    totalTests++;
    if (setFanState(true)) {
        ESP_LOGI(TAG, "✓ Fan State ON 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Fan State ON 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-4. Fan Speed Control
    ESP_LOGI(TAG, "3-4. Fan Speed 설정 테스트");
    totalTests++;
    if (setFanSpeed(7)) {
        ESP_LOGI(TAG, "✓ Fan Speed 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Fan Speed 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-5. Speaker Volume Control
    ESP_LOGI(TAG, "3-5. Speaker Volume 설정 테스트");
    totalTests++;
    if (setSpeakerVolume(5)) {
        ESP_LOGI(TAG, "✓ Speaker Volume 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Speaker Volume 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-6. Cooling Fan State
    ESP_LOGI(TAG, "3-6. Cooling Fan State ON 테스트");
    totalTests++;
    if (setCoolingFanState(true)) {
        ESP_LOGI(TAG, "✓ Cooling Fan State ON 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Cooling Fan State ON 명령 전송 실패");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 3-7. Pose Detection Mode
    ESP_LOGI(TAG, "3-7. Pose Detection Mode 설정 테스트");
    totalTests++;
    if (setPoseDetectionMode(true)) {
        ESP_LOGI(TAG, "✓ Pose Detection Mode 명령 전송 성공");
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ Pose Detection Mode 명령 전송 실패");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // ========================
    // 4. 연속 명령 전송 테스트
    // ========================
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== 연속 명령 전송 테스트 ===");
    
    ESP_LOGI(TAG, "4-1. 연속 5개 명령 전송 테스트 시작");
    uint32_t consecutiveStartTime = millis();
    
    totalTests++;
    bool consecutiveSuccess = true;
    consecutiveSuccess &= setDeviceMode(DeviceMode::SLEEP);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    consecutiveSuccess &= requestSpeakerVolume();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    consecutiveSuccess &= setFanSpeed(3);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    consecutiveSuccess &= setCoolingFanLevel(2);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    consecutiveSuccess &= setDeviceMode(DeviceMode::WAITING);
    
    uint32_t consecutiveEndTime = millis();
    uint32_t consecutiveElapsed = consecutiveEndTime - consecutiveStartTime;
    
    if (consecutiveSuccess) {
        ESP_LOGI(TAG, "✓ 연속 5개 명령 전송 성공 (소요시간: %lums)", consecutiveElapsed);
        passedTests++;
    } else {
        ESP_LOGE(TAG, "✗ 연속 5개 명령 전송 실패");
    }
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // =================
    // 5. 테스트 결과 요약
    // =================
    uint32_t testEndTime = millis();
    uint32_t totalElapsed = testEndTime - testStartTime;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "=== ESP→STM 통신 테스트 결과 요약 ===");
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "총 테스트 수: %d", totalTests);
    ESP_LOGI(TAG, "성공한 테스트: %d", passedTests);
    ESP_LOGI(TAG, "실패한 테스트: %d", totalTests - passedTests);
    ESP_LOGI(TAG, "성공률: %.1f%%", (float)passedTests / totalTests * 100.0);
    ESP_LOGI(TAG, "총 소요시간: %lu초", totalElapsed / 1000);
    ESP_LOGI(TAG, "");
    
    if (passedTests == totalTests) {
        ESP_LOGI(TAG, "🎉 모든 테스트가 성공했습니다!");
    } else {
        ESP_LOGW(TAG, "⚠️  일부 테스트가 실패했습니다. 로그를 확인하세요.");
    }
    
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "=== ESP→STM 통신 테스트 완료 ===");
    ESP_LOGI(TAG, "======================================================");
}

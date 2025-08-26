/**
 * @file test_command_main.cpp
 * @brief ESP32→STM32 Command Test Program
 * 
 * This program provides an interactive test interface for ESP32→STM32 communication.
 * Users can input command numbers to test various protocol commands and receive
 * ACK responses from the STM32.
 * 
 * Usage:
 * - Connect to Serial monitor (115200 baud)
 * - Enter command numbers from the menu
 * - Provide parameters when requested
 * - View results and ACK status
 */

#include <Arduino.h>
#include <esp_log.h>
#include "globals.h"
#include "uartMaster.h"

// Minimal global variables needed for uartMaster
TaskHandle_t TasUsartMaster_h = NULL;
SystemConfig_t systemConfig = {};
biosigDataClass biosigData1Min;
tinyMLDataClass predictionInput;
Kalman_t kalmanL;
Kalman_t kalmanR;
allData_t allData;

static const char TAG[] = __FILE__;

// Command test structure for menu display
struct CommandTest {
    uint8_t commandCode;
    const char* name;
    const char* description;
    uint8_t paramCount;
    const char* paramNames[4];
};

// Define all available commands for testing
static const CommandTest commands[] = {
    // INIT Commands (0x10-0x21)
    {initTempSleep, "initTempSleep", "Set sleep mode temperature", 2, {"temp_int", "temp_dec"}},
    {initTempWaiting, "initTempWaiting", "Set waiting mode temperature", 2, {"temp_int", "temp_dec"}},
    {initTempForceUp, "initTempForceUp", "Set force-up mode temperature", 2, {"temp_int", "temp_dec"}},
    {initTempLimit, "initTempLimit", "Set upper temperature limit", 2, {"temp_int", "temp_dec"}},
    {initPWMCoolFan, "initPWMCoolFan", "Set cooling fan PWM level", 2, {"current_level", "max_level"}},
    {initTout, "initTout", "Set timeout values", 4, {"force_up_timeout", "force_on_timeout", "force_down_timeout", "waiting_timeout"}},
    {initSpk, "initSpk", "Set speaker volume", 1, {"volume_0_to_10"}},
    {initDelay, "initDelay", "Set ForceDown delay value (0-20)", 1, {"force_down_delay"}},
    {initGyroAct, "initGyroAct", "Set heating angle (IMU active angle)", 1, {"active_angle"}},
    {initGyroRel, "initGyroRel", "Set cooling angle (IMU relative angle)", 1, {"relative_angle"}},
    {initMode, "initMode", "Set mode change (0: AI mode, 1: IMU mode)", 1, {"mode"}},
    {initPWMFan, "initPWMFan", "Set PWM fan speed (0-3)", 1, {"fan_speed"}},
    
    // REQUEST Commands (0x30-0x41)
    {reqTempSleep, "reqTempSleep", "Get sleep mode temperature", 0, {}},
    {reqTempWaiting, "reqTempWaiting", "Get waiting mode temperature", 0, {}},
    {reqTempForceUp, "reqTempForceUp", "Get force-up mode temperature", 0, {}},
    {reqUpperTemp, "reqUpperTemp", "Get upper temperature limit", 0, {}},
    {reqPWMCoolFan, "reqPWMCoolFan", "Get cooling fan PWM level", 0, {}},
    {reqTimeout, "reqTimeout", "Get timeout values", 0, {}},
    {reqSpk, "reqSpk", "Get speaker volume", 0, {}},
    {reqDelay, "reqDelay", "Get detection delay values", 0, {}},
    {reqGyroAct, "reqGyroAct", "Get heating angle (IMU active angle)", 0, {}},
    {reqGyroRel, "reqGyroRel", "Get cooling angle (IMU relative angle)", 0, {}},
    {reqMode, "reqMode", "Get current mode", 0, {}},
    {reqPWMFan, "reqPWMFan", "Get PWM fan speed", 0, {}},
    
    // CONTROL Commands (0x50-0x61)
    {ctrlReset, "ctrlReset", "Reset device", 0, {}},
    {ctrlMode, "ctrlMode", "Set device operating mode", 1, {"mode"}},
    {ctrlSpkOn, "ctrlSpkOn", "Turn speaker on/off", 1, {"on_off_1_0"}},
    {ctrlFanOn, "ctrlFanOn", "Turn main fan on/off", 1, {"on_off_1_0"}},
    {ctrlFanPWM, "ctrlFanPWM", "Set main fan speed (0-3) - DEPRECATED: Use initPWMFan (0x22)", 1, {"speed_0_to_3"}},
    {ctrlCoolFanOn, "ctrlCoolFanOn", "Turn cooling fan on/off", 1, {"on_off_1_0"}},
    {ctrlCoolFanPWM, "ctrlCoolFanPWM", "Set cooling fan PWM level - DEPRECATED: Use initPWMCoolFan (0x15)", 1, {"pwm_level"}},
    {ctrlForceUp, "ctrlForceUp", "Force Up mode (ON=1 only) - DEPRECATED: Use ctrlMode (0x51) with value 2", 1, {"1"}},
    {ctrlForceDown, "ctrlForceDown", "Force Down mode (ON=1 only) - DEPRECATED: Use ctrlMode (0x51) with value 4", 1, {"1"}},
    {ctrlSleeping, "ctrlSleeping", "Sleep mode (ON=1 only) - DEPRECATED: Use ctrlMode (0x51) with value 0", 1, {"1"}}
};

static const size_t commandCount = sizeof(commands) / sizeof(commands[0]);

/**
 * @brief Display the command menu
 */
void displayMenu() {
    Serial.println();
    Serial.println("=====================================");
    Serial.println("  ESP32→STM32 Command Test Program");
    Serial.println("=====================================");
    Serial.println();
    
    Serial.println("INIT Commands (0x10-0x29) - Initialize device parameters:");
    for (size_t i = 0; i < commandCount; i++) {
        if (commands[i].commandCode >= 0x10 && commands[i].commandCode <= 0x29) {
            Serial.printf("[0x%02X] %s - %s\n", 
                         commands[i].commandCode, 
                         commands[i].name, 
                         commands[i].description);
        }
    }
    
    Serial.println();
    Serial.println("REQUEST Commands (0x30-0x49) - Query current parameter values:");
    for (size_t i = 0; i < commandCount; i++) {
        if (commands[i].commandCode >= 0x30 && commands[i].commandCode <= 0x49) {
            Serial.printf("[0x%02X] %s - %s\n", 
                         commands[i].commandCode, 
                         commands[i].name, 
                         commands[i].description);
        }
    }
    
    Serial.println();
    Serial.println("CONTROL Commands (0x50-0x61) - Real-time device control:");
    for (size_t i = 0; i < commandCount; i++) {
        if (commands[i].commandCode >= 0x50 && commands[i].commandCode <= 0x61) {
            Serial.printf("[0x%02X] %s - %s\n", 
                         commands[i].commandCode, 
                         commands[i].name, 
                         commands[i].description);
        }
    }
    
    Serial.println();
    Serial.println("Enter command code in HEX (e.g., 0x10 or 10) or 'menu' to show this menu again:");
    Serial.print("> ");
}

/**
 * @brief Find command by code
 */
const CommandTest* findCommand(uint8_t commandCode) {
    for (size_t i = 0; i < commandCount; i++) {
        if (commands[i].commandCode == commandCode) {
            return &commands[i];
        }
    }
    return nullptr;
}

/**
 * @brief Read integer input from Serial with default value support
 */
int readIntFromSerial(const char* prompt, int defaultValue, int minVal = 0, int maxVal = 255) {
    Serial.printf("%s [default: %d]: ", prompt, defaultValue);
    
    // Wait for input or timeout
    unsigned long startTime = millis();
    String input = "";
    
    while (millis() - startTime < 10000) { // 10 second timeout
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                break;
            } else if (c >= '0' && c <= '9') {
                input += c;
                Serial.print(c); // Echo the character
            } else if (c == 8 || c == 127) { // Backspace or DEL
                if (input.length() > 0) {
                    input.remove(input.length() - 1);
                    Serial.print("\b \b"); // Erase character on screen
                }
            }
        }
        delay(10);
    }
    
    Serial.println(); // New line after input
    
    // If no input or just whitespace, use default
    input.trim();
    if (input.length() == 0) {
        Serial.printf("Using default value: %d\n", defaultValue);
        return defaultValue;
    }
    
    // Parse the input
    int value = input.toInt();
    
    // Validate range
    if (value < minVal || value > maxVal) {
        Serial.printf("Invalid value %d. Must be between %d and %d.\n", value, minVal, maxVal);
        return readIntFromSerial(prompt, defaultValue, minVal, maxVal);
    }
    
    return value;
}

/**
 * @brief Execute command with user input
 */
void executeCommand(uint8_t commandCode) {
    const CommandTest* cmd = findCommand(commandCode);
    if (!cmd) {
        Serial.printf("Command 0x%02X not found in test menu\n", commandCode);
        return;
    }
    
    Serial.printf("\n--- Executing Command: %s (0x%02X) ---\n", cmd->name, commandCode);
    Serial.printf("Description: %s\n", cmd->description);
    
    uint8_t data[8] = {0}; // Maximum parameter data
    size_t dataLen = 0;
    
    // Get parameters based on command type
    if (cmd->paramCount > 0) {
        Serial.printf("This command requires %d parameter(s):\n", cmd->paramCount);
        
        for (uint8_t i = 0; i < cmd->paramCount; i++) {
            int defaultValue = 0;
            int minVal = 0;
            int maxVal = 255;
            
            // Set appropriate default values based on parameter name
            if (strstr(cmd->paramNames[i], "temp_int") != nullptr) {
                defaultValue = 25; // 25°C
            } else if (strstr(cmd->paramNames[i], "temp_dec") != nullptr) {
                defaultValue = 0;  // .0°C
            } else if (strstr(cmd->paramNames[i], "timeout") != nullptr) {
                defaultValue = 10; // 1000ms (10 * 100ms)
                maxVal = 100;      // Max 10 seconds
            } else if (strstr(cmd->paramNames[i], "volume") != nullptr) {
                defaultValue = 5;  // Volume level 5
                maxVal = 10;       // Max volume 10
            } else if (strstr(cmd->paramNames[i], "level") != nullptr || strstr(cmd->paramNames[i], "pwm") != nullptr) {
                defaultValue = 5;  // Mid-level PWM
                maxVal = 10;       // Max level 10
            } else if (strstr(cmd->paramNames[i], "speed") != nullptr || strstr(cmd->paramNames[i], "fan_speed") != nullptr) {
                defaultValue = 2;  // Speed level 2
                maxVal = 3;        // Max speed 3
            } else if (strstr(cmd->paramNames[i], "angle") != nullptr) {
                defaultValue = 45; // 45 degrees
                maxVal = 180;      // Max 180 degrees
            } else if (strstr(cmd->paramNames[i], "delay") != nullptr) {
                defaultValue = 10; // 10 * 100ms = 1 second
                maxVal = 100;      // Max 10 seconds
            } else if (strstr(cmd->paramNames[i], "mode") != nullptr) {
                // For initMode (0x21): default to IMU mode as agreed between ESP and STM
                // For ctrlMode (0x51): default to SLEEP mode  
                defaultValue = (commandCode == initMode) ? 1 : 0;  // IMU mode for initMode, SLEEP for ctrlMode
                maxVal = (commandCode == initMode) ? 1 : 4;        // Max mode 1 for initMode, 4 for ctrlMode
            } else if (strstr(cmd->paramNames[i], "on_off") != nullptr || strcmp(cmd->paramNames[i], "1") == 0) {
                defaultValue = 1;  // ON
                maxVal = 1;        // Only 0 or 1
            } else {
                defaultValue = 1;  // Generic default
            }
            
            int value = readIntFromSerial(cmd->paramNames[i], defaultValue, minVal, maxVal);
            data[i] = (uint8_t)value;
            dataLen++;
        }
    }
    
    // Display what we're sending
    Serial.printf("Sending command 0x%02X", commandCode);
    if (dataLen > 0) {
        Serial.print(" with data: ");
        for (size_t i = 0; i < dataLen; i++) {
            Serial.printf("0x%02X ", data[i]);
        }
    }
    Serial.println();
    
    // Send the command
    bool success = sendCommandAsync(commandCode, dataLen > 0 ? data : nullptr, dataLen);
    
    if (success) {
        Serial.println("✓ Command sent successfully");
        Serial.println("Waiting for ACK response...");
        
        // Wait for response with timeout
        uint32_t startTime = millis();
        const uint32_t timeout = 5000; // 5 second timeout
        bool responseReceived = false;
        
        while (millis() - startTime < timeout) {
            CommandState state = getCommandState(commandCode);
            
            switch (state) {
                case CommandState::ACK_RECEIVED:
                    Serial.println("✓ ACK received successfully!");
                    responseReceived = true;
                    break;
                    
                case CommandState::DATA_RECEIVED:
                    Serial.println("✓ Data response received successfully!");
                    responseReceived = true;
                    break;
                    
                case CommandState::TIMEOUT:
                    Serial.println("✗ Command timed out - no response from STM32");
                    responseReceived = true;
                    break;
                    
                case CommandState::ERROR:
                    Serial.println("✗ Communication error occurred");
                    responseReceived = true;
                    break;
                    
                case CommandState::SENT:
                    // Still waiting for response
                    delay(100);
                    break;
                    
                default:
                    // Command completed or unknown state
                    responseReceived = true;
                    break;
            }
            
            if (responseReceived) {
                break;
            }
        }
        
        if (!responseReceived) {
            Serial.println("✗ Test timeout - no response status available");
        }
        
    } else {
        Serial.println("✗ Failed to send command");
    }
    
    Serial.println("--- Command test completed ---\n");
}

/**
 * @brief Parse hex input from user
 */
uint8_t parseHexInput(String input) {
    input.trim();
    input.toLowerCase();
    
    // Remove 0x prefix if present
    if (input.startsWith("0x")) {
        input = input.substring(2);
    }
    
    // Convert hex string to integer
    return (uint8_t)strtoul(input.c_str(), NULL, 16);
}

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    delay(100);
    
    // Set logging level
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // esp_log_level_set("uartMaster.cpp", ESP_LOG_ERROR);
    
    ESP_LOGI(TAG, "=== ESP32→STM32 Command Test Program Starting ===");
    
    // Initialize UART Master for STM32 communication
    ESP_LOGI(TAG, "[SETUP] Initializing UART Master...");
    if (initUartMaster()) {
        ESP_LOGI(TAG, "[SETUP] UART Master initialization completed successfully");
    } else {
        ESP_LOGE(TAG, "[ERROR] UART Master initialization failed");
    }
    
    // Display initial menu
    displayMenu();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readString();
        input.trim();
        
        if (input.equalsIgnoreCase("menu")) {
            displayMenu();
        } else if (input.length() > 0) {
            uint8_t commandCode = parseHexInput(input);
            
            if (commandCode == 0 && !input.equals("0") && !input.equals("0x0")) {
                Serial.println("Invalid input. Please enter a valid hex command code (e.g., 0x10 or 10)");
                Serial.print("> ");
            } else {
                executeCommand(commandCode);
                Serial.print("> ");
            }
        }
    }
    
    delay(10); // Small delay to prevent excessive CPU usage
}
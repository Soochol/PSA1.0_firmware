// GENERAL
#include "globals.h"
#include <time.h>
#include <stdlib.h>
#include <Arduino.h>
#include <esp_coexist.h>
#include <esp_log.h>
#include <Wire.h>
#include <SPI.h>
#include "timer.h"
#include "uartMaster.h"
#include "irqhandler.h"
#include "mallocator.h"
#include "resetCheck.h"
#include <Preferences.h>
#include "configManager.h"
#include "data.h"

// AI INFERENCE
#include "tinyml.h"

// BLE
// #include "bleData.h"
// #include "bleServerHub4.h"

// AWS + LTE
#include "type1scMqttLte.h"

// GNSS
#include "samM10Q.h"

static const char TAG[] = __FILE__;

// DebugSerial is now defined as macro in globals.h


TaskHandle_t irqHandlerTask = NULL;
TaskHandle_t TaskBle_h = NULL;
TaskHandle_t TasUsartMaster_h = NULL;
TaskHandle_t TaskGnss_h = NULL;
TaskHandle_t TaskLte_h = NULL;
TaskHandle_t TaskSubscribe_h = NULL;
TaskHandle_t TaskPublish_h = NULL;
TaskHandle_t TaskTimer_h = NULL;
TaskHandle_t TaskBiosigProcess_h = NULL;

configData_t *cfg;
Preferences preferences;
SemaphoreHandle_t reportReadySemaphore;
SemaphoreHandle_t processBiosignalsSemaphore;

allData_t allData;
SystemConfig_t systemConfig = {};
tinyMLDataClass predictionInput;
biosigDataClass biosigData1Min;

Kalman_t kalmanL;
Kalman_t kalmanR;

// Function declarations

void setup() {

    // Set heating available
    pinMode(PWM_ESP_HEATER, OUTPUT);
    digitalWrite(PWM_ESP_HEATER, HIGH); // Turn on heater by default
    
#ifdef STM_HARDWARE_CONNECTED
    Serial.begin(115200);
    delay(100); // Wait for Serial to initialize
    esp_log_level_set("*", ESP_LOG_ERROR); // Hide almost everything  
    esp_log_level_set(TAG, ESP_LOG_INFO); // Allow main setup messages
    esp_log_level_set("/home/blessp/my_code/PSA1.0_firmware-master/src/uartMaster.cpp", ESP_LOG_INFO); // Allow STATUS messages only
    ESP_LOGI(TAG, "=== PSA1.0 FIRMWARE SETUP START (STM HARDWARE MODE) ===");
    ESP_LOGI(TAG, "[DEBUG] ESP_LOG initialized on UART0 (USB), STM communication on UART2");
#else
    // ESP_LOG 완전 비활성화 (STM 통신 채널 보호)
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

// CONFIGS --------------------------------------------------
    reportReadySemaphore = xSemaphoreCreateBinary();
    processBiosignalsSemaphore = xSemaphoreCreateBinary();

    cfg = ALLOCATION(configData_t, 1);
    makeThingName(); 
    setEndpoints(cfg);

// BLE FACTORY ----------------------------------------------

// STM32 CONNECT (priority initialization) -----------------
    ESP_LOGI(TAG, "[SETUP] Initializing UART Master...");
    initUartMaster(); // Basic version for step-by-step debugging
    ESP_LOGI(TAG, "[SETUP] UART Master initialization completed");

// DATA PROCESSING + TinyML ----------------------------------
    ESP_LOGI(TAG, "[SETUP] Initializing TinyML...");
    initTinyML();
    ESP_LOGI(TAG, "[SETUP] TinyML initialization completed");

	// initKalman(&kalmanL);
	// initKalman(&kalmanR);

    // initBiosignalProcessing();

// TIMER (early start) ----------------------------------------
    ESP_LOGI(TAG, "[SETUP] Initializing Timer...");
    initTimer();
    ESP_LOGI(TAG, "[SETUP] Timer initialization completed");

// LTE MQTT (moved to last) -----------------------------------
    ESP_LOGI(TAG, "[SETUP] Initializing LTE...");
    setSerials();
    // type1scSetAPN();
    initType1sc();
    // type1scSetCertificates();
    type1scConnectAws();
    type1scRegisterCallback();
    ESP_LOGI(TAG, "[SETUP] LTE initialization completed");

// GNSS -----------------------------------------------------
    // bool dummy1;
	// dummy1 = Wire1.begin(static_cast<int>(ESP_I2C2_SDA), static_cast<int>(ESP_I2C2_SCL), static_cast<uint32_t>(400000));
	// Serial.println(dummy1);
    // initGNSS();

    ESP_LOGI(TAG, "=== PSA1.0 FIRMWARE SETUP COMPLETE ===");

	vTaskDelete(NULL);
}


void loop() {
	// Main loop - should run continuously
}
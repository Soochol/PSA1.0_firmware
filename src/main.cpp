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

// Custom log output function for ESP_LOG to use UART2
int debug_log_vprintf(const char* format, va_list args) {
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    DebugSerial.print(buffer);
    return len;
}

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

void setup() {

#ifdef STM_HARDWARE_CONNECTED
    // 원래 설계: UART0=디버깅, UART2=STM 프로토콜
    DebugSerial.begin(115200);      // Debug output
    STMSerial.begin(115200, SERIAL_8N1, ESP_U2_RXD, ESP_U2_TXD);  // STM protocol
    delay(1000);
    
    // ESP_LOG를 UART0으로 출력 (기본값)
    DebugSerial.println("=== PSA1.0 FIRMWARE SETUP START (STM MODE) ===");
#else
    // 현재 테스트 상황: UART0=Python 프로토콜, UART2=디버깅
    DebugSerial.begin(115200);      // Protocol with Python
    STMSerial.begin(115200);     // Debug output
    delay(1000);
    
    // ESP_LOG를 UART2로 리다이렉트
    esp_log_set_vprintf(debug_log_vprintf);
    
    DebugSerial.println("=== PSA1.0 FIRMWARE SETUP START (PYTHON MODE) ===");
    DebugSerial.println("=== PSA1.0 FIRMWARE SETUP START (PYTHON MODE) ===");
#endif

// CONFIGS --------------------------------------------------
    reportReadySemaphore = xSemaphoreCreateBinary();
    processBiosignalsSemaphore = xSemaphoreCreateBinary();

    cfg = ALLOCATION(configData_t, 1);
    makeThingName(); 
    setEndpoints(cfg);

// BLE FACTORY ----------------------------------------------


// LTE MQTT -------------------------------------------------
    setSerials();
    // type1scSetAPN();
    initType1sc();
    // type1scSetCertificates();
    type1scConnectAws();
    type1scRegisterCallback();

// GNSS -----------------------------------------------------
    // bool dummy1;
	// dummy1 = Wire1.begin(static_cast<int>(ESP_I2C2_SDA), static_cast<int>(ESP_I2C2_SCL), static_cast<uint32_t>(400000));
	// Serial.println(dummy1);
    // initGNSS();

// STM CONNECT ----------------------------------------------
    DebugSerial.println("[SETUP] Initializing UART Master...");
    initUartMaster(); // Basic version for step-by-step debugging
    DebugSerial.println("[SETUP] UART Master initialization completed");

// DATA PROCESSING + eloquentML -----------------------------
    DebugSerial.println("[SETUP] Initializing TinyML...");
    initTinyML();
    DebugSerial.println("[SETUP] TinyML initialization completed");

	// initKalman(&kalmanL);
	// initKalman(&kalmanR);

    // initBiosignalProcessing();

// TIMER ----------------------------------------------------
    // initializing the timer starts sending data to the backend
    DebugSerial.println("[SETUP] Initializing Timer...");
    initTimer();
    DebugSerial.println("[SETUP] Timer initialization completed");

    DebugSerial.println("=== PSA1.0 FIRMWARE SETUP COMPLETE ===");
	vTaskDelete(NULL);
}

void loop() {
	vTaskDelete(NULL);
}
// GENERAL
#include "globals.h"
#include <time.h>
#include <stdlib.h>
#include <Arduino.h>
#include <esp_coexist.h>
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
tinyMLDataClass predictionInput;
biosigDataClass biosigData1Min;

Kalman_t kalmanL;
Kalman_t kalmanR;

void setup() {

    Serial.begin(115200);

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
    initUartMaster();

// DATA PROCESSING + eloquentML -----------------------------
    initTinyML();

	// initKalman(&kalmanL);
	// initKalman(&kalmanR);

    // initBiosignalProcessing();

// TIMER ----------------------------------------------------
    // initializing the timer starts sending data to the backend
    initTimer();

	vTaskDelete(NULL);
}

void loop() {
	vTaskDelete(NULL);
}
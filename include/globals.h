/* Developer Note
(cshin)
- Test Code: Code to test out new code (to be committed or deleted)
- Test Temp: Temporary code just for testing (comment or delete prior to commit)
- Test Silenced: Existing code that was silenced for testing (uncomment or delete prior to commit)

(cshin)
- LEDs for V1.0.2 (pre-batch) hardware has Gr/Bl flipped, FIXED PINMAP (20230919)
*/

#ifndef _GLOBALS_H
#define _GLOBALS_H

#include <Arduino.h>  // For HardwareSerial

// release version---------------------
#define RELEASE_MODE
#define DEV_MODE
#define TEST_MODE

// #define USE_WIFI
#define LILYGO

// #define PRINT_SENSOR_DATA
#define TINY_ML_PREDICTION
// #define BLE_TEST
//-------------------------------------

// debug log level---------------------
#define LOG_LEVEL 4 //ERROR, WARN, INFO, DEBUG, (5) VERBOSE
#define CORE_DEBUG_LEVEL LOG_LEVEL
//-------------------------------

// version config---------------------
#define REV_CODE 0
#define FV_CODE 0
#define STAGE_CODE "DEV"
//-------------------------------

// model config---------------------
#define MODEL_NUMBER "PSP2.0-001"
#define BLE_ADVERTISE_NAME "PSP2.0"
//------------------------------- 

// #define SENSORS_STM_CONNECTED

// === UART 할당 모드 설정 ===
// STM_HARDWARE_CONNECTED 매크로가 활성화/비활성화에 따라 UART 할당이 달라짐
// 
// [STM_HARDWARE_CONNECTED 활성화 시]
// - ESP_LOG = UART0 (USB) → 디버깅 출력
// - STMSerial = UART2 (GPIO 15/16) → STM 하드웨어 통신
//
// [STM_HARDWARE_CONNECTED 비활성화 시] 
// - ESP_LOG = 비활성화 (출력 없음)
// - STMSerial = UART0 (USB) → Python 테스트 통신
//
// STM_HARDWARE_CONNECTED 매크로는 platformio.ini에서 빌드 플래그로 관리됨

/*
   MEMORY KEEPING
*/
#define MEM_LOW 2048 // [Bytes] low memory threshold triggering a send cycle

/*
   GPIO pin map
*/
/// NOTE: PIN DEFNITIONS NEEDED --------------------------------------------------------------------------------------

// EXTERNAL 8PIN for 2 I2C channels
#define ESP_I2C1_SDA    9
#define ESP_I2C1_SCL    10
#define ESP_I2C1_INT    11
#define ESP_I2C2_SDA    13
#define ESP_I2C2_SCL    14
#define ESP_I2C2_INT    12

// LTE MODEM
#define ESP_U1_RXD      18
#define ESP_U1_TXD      17
#define ESP_U1_RTS      19
#define ESP_U1_CTS      20

// STM SERIAL
#define ESP_U2_RXD      16
#define ESP_U2_TXD      15

// === 하드웨어별 시리얼 포트 정의 ===
#define LTESerial Serial1       // UART1 → LTE 모뎀 (고정)

// === 조건부 시리얼 매핑: STM 하드웨어 연결 여부에 따라 결정 ===
#ifdef STM_HARDWARE_CONNECTED
    // STM 하드웨어 연결 모드: ESP_LOG가 UART0(USB)로 디버깅 출력
    #define STMSerial Serial2       // UART2 (GPIO 15/16) → STM 하드웨어 통신
#else
    // Python 테스트 모드: ESP_LOG 비활성화, STMSerial만 사용  
    #define STMSerial Serial        // UART0 (USB) → Python 테스트 통신
#endif

// ESP HEATER
#define PWM_ESP_HEATER  8

// EXTERNAL IO
#define EXT_IO1         4
#define EXT_IO2         5
#define EXT_IO3         6
#define EXT_IO4         7


/*
    Timer Configurations 
*/
#define TIMER_DIVIDER_MICRO 80 // timer prescaler (MHz) (ESP32 default 80MHz). set timer ticker to 1MHz (1us)
#define TIMER_0     uint8_t(0) // Central timer to collect and send data
#define TIMER_1     uint8_t(1)
#define TIMER_2     uint8_t(2)
#define TIMER_3     uint8_t(3)

// BCG Timer
#define GLOBAL_TIMER_NUM    TIMER_0
#define DATA_SET_TIMER_NUM  TIMER_1

/*
   Ticker Time (SECs)
*/
#define SEND_DATA_TIME      60
#define UPDATE_REPORT_TIME  10 * 60 // 10 minutes
#define INIT_REPORT_TIME    60 * 60 * 24 * 7 // 1 week
#define HOME_KEEPING_TIME   25

/*
   Application retry or thres time
*/
#define BLE_SERVER_TIME 120                                // sec
#define SEND_REPORT_DATA_INTERVAL_TIME 24 * 60 * 60 * 1000 // 24 hours(ms)

/*
   Irq Bit Flag
*/
#define RESET_MANU              BIT0
#define SEND_REPORT_IRQ         BIT1
#define SEND_EMERGENCY_IRQ      BIT2

/*
   Wait Bit Flag
*/
#define NOTIFY_BIT BIT0
#define DISCONNECTED_BIT BIT1
#define SEND_BIT BIT2

/*
   RTOS TASK CONF
*/
// Core 0: 
#define CORE_0 0
// Core 1: 
#define CORE_1 1

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE CORE_0
#else
#define ARDUINO_RUNNING_CORE CORE_1
#endif

/* CORE 0 Tasks */

#define TIMER_TASK_PRI 1
#define TIMER_TASK_STACK 2048
#define TIMER_TASK_CORE CORE_0

#define IRQ_TASK_PRI 5
#define IRQ_TASK_STACK 2048
#define IRQ_TASK_CORE CORE_0

#define BLE_TASK_PRI 4
#define BLE_TASK_STACK 4096
#define BLE_TASK_CORE CORE_0

#define GNSS_TASK_PRI 3
#define GNSS_TASK_STACK 4096
#define GNSS_TASK_CORE CORE_0

#define PROCESS_BIOSIG_TASK_PRI 2
#define PROCESS_BIOSIG_TASK_STACK 4096
#define PROCESS_BIOSIG_TASK_CORE CORE_0

/* CORE 1 Tasks */

#define UART_TASK_PRI 1
#define UART_TASK_STACK 4096
#define UART_TASK_CORE CORE_1

#define MQTT_TASK_PRI 2
#define MQTT_TASK_STACK 4096
#define MQTT_TASK_CORE CORE_1

#define PUBLISH_TASK_PRI 4
#define PUBLISH_TASK_STACK 8192
#define PUBLISH_TASK_CORE CORE_1


#define PAYLOAD_BUFFER_RCV_MAXIMUM_SIZE 1024
#define PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE 1024 // maximum size of payload block per transmit


/* Central timer and data buffer */
#define TIMER_HZ                10
#define TIMER2_HZ               20
#define DATA_BUFFER_CYCLE_TIME  1 // 0.1
// #define DATA_TINYML_BUFFER_LEN         (TIMER_HZ * DATA_BUFFER_CYCLE_TIME * NUM_SENSORS * 2)

#define USART_MESSAGE_MAXIMUM_LENGTH 256

// Sensor data parsing constants
#define SENSOR_DATA_MIN_LENGTH 42  // Total bytes for complete sensor data
#define IMU_DATA_SIZE 6           // Gyro or Accel data size (3 axes * 2 bytes)
#define PRESSURE_DATA_SIZE 2      // Single pressure sensor size
#define TEMPERATURE_DATA_SIZE 2   // Single temperature sensor size
#define IMU_EVENT_SIZE 1          // Single IMU event size

#define BLE_DATA_MAXIMUM_SIZE 200

#define MAC_MAXIMUM_SIZE 30
#define BLE_MANU_MAXIMUM_SIZE 20
#define IDV_MAXIMUM_SIZE 8
#define D_ID_MAXIMUM_SIZE MAC_MAXIMUM_SIZE + IDV_MAXIMUM_SIZE

#define COMMAND_MAXIMUM_SIZE 100
#define UUID_MAXIMUM_SIZE 40
#define ADD_DATA_MAXIMUM_SIZE 512

#define THING_NAME_MAXIMUM_SIZE 128
#define CERT_MAXIMUM_SIZE 4096
#define TOPIC_MAXIMUM_SIZE 128
#define VERSION_MAXIMUM_SIZE 10
#define MAXIMUM_BASE64_DEVICE_DATA_SIZE 25

#define MAXIMUM_CONFIG_DATA_SIZE 512
#define MAXIMUM_DEVICE_FILE_DATA_SIZE 4096
#define MAXIMUM_REPORT_FILE_DATA_SIZE 8192
#define MAXIMUM_CERT_FILE_DATA_SIZE 6000

#define UTC 9
#define NTP_INTERVAL 60000 * 5
#define CHECK_LOOP_STUCK_THRES 90000
#define CHECK_MQTT_STUCK_THRES 30000

#define ALLOCATION(T, SIZE) psramFound() ? (T *)ps_malloc((SIZE) * sizeof(T)) : (T *)malloc((SIZE) * sizeof(T))
#define ALLOCATION_OBJ(T) psramFound() ? ps_malloc(sizeof(T)) : malloc(sizeof(T))
#define ROUND_DATA(DATA) roundf(DATA * 100) / 100
#define INT_8_TO_INT_16(d1, d2) ((uint16_t)d1 << 8) | d2

#define D_NUM 8

#include <Arduino.h>

enum sendprio_t {
    prio_low,
    prio_normal,
    prio_high
};

enum msgtype_t {
    ble_client,
    ble_client_server,
    ble_scan,
    mqtt_publish
};

enum reset_reason_t {
    rst_undefined,                      // 0
    rst_device_reset_manu,              // 1
    rst_memory_low,                     // 2
    rst_main_ble_client_stuck,          // 3
    rst_mqtt_stuck,                     // 4
    rst_failed_to_connect_mqtt,         // 5
    rst_cannot_get_cert,                // 6
    rst_manufacture_test,               // 7
};

enum report_category_t {
    mqtt_check_pub_try,
    mqtt_check_pub_success,
    mqtt_check_pub_retry,
    mqtt_check_pub_fail,
    mqtt_check_disconnect,

    ble_check_attempt,
    ble_check_attempt_success,
    ble_check_attempt_fail,
    ble_check_try,

    ble_fail_connect,
    ble_fail_disconnected,
    ble_fail_get_service,
    ble_fail_notif_find,
    ble_fail_notif,
    ble_fail_write_find,
    ble_fail_write_cannot,
    ble_fail_write_value,
    ble_fail_write_invalid,

    ble_scan_attempt,
    ble_scan_find,
    ble_scan_cannot_find_all,
};

#define MAX_REPORT_ARRAY 100

typedef struct {

    char thing_name[THING_NAME_MAXIMUM_SIZE];

    char version[VERSION_MAXIMUM_SIZE];

    char subTopic[TOPIC_MAXIMUM_SIZE];
    char pubReportTopic[TOPIC_MAXIMUM_SIZE];
    char pubPowerOnTopic[TOPIC_MAXIMUM_SIZE];
    char pubEmergencyTopic[TOPIC_MAXIMUM_SIZE];

    int8_t time_offset = 9;
    bool isConnected = false;
    int bootCnt = 0;
    reset_reason_t current_reset_reason;
    reset_reason_t prev_reset_reason;

} configData_t; 

typedef struct {
    sendprio_t MessagePrio;
    msgtype_t MessageType;
    uint8_t index;
} Queue_message_t;


#include <vector> // Add this line
#include <cstdint> // For uint16_t, int16_t, uint8_t if not already included
#define BIOSIG_BUFFER_LEN 600
#define BREATH_RATE_BUFFER_LEN 6
#define ACCEL_BUFFER_LEN 100

typedef struct {
    
    int16_t L_gyro[3];
    int16_t L_accel[3];
    int16_t L_ads = 0;

    int16_t R_gyro[3];
    int16_t R_accel[3];
    int16_t R_ads = 0;

    int16_t outTemp = 25;
    int16_t lmaTemp = 25;
    int16_t boardTemp = 25;
    
    int16_t lmaLength = 100;

    int16_t objDistance = 100;

    int16_t battery = 100;
    
    int16_t latitude = 0;
    int16_t longitude = 0;

    uint8_t breathRate = 0;   // breaths/min x10

    uint32_t stepCount = 0;     // these values are cumulative
    uint32_t lmaCount = 0;      // 

    bool fallDetected = false;      
    
    uint8_t bentDuration = 0;   // in seconds

    int16_t torsoAngleMean = 0; // in degrees
    int16_t torsoAngleMin = 0;
    
    uint8_t tilt_count = 0;
    
    uint8_t leftIMUEvent = 0;
    uint8_t rightIMUEvent = 0;

} allData_t;



typedef struct {
    float sleepTemp;
    float waitingTemp; 
    float operatingTemp;
    float upperTempLimit;
    uint8_t coolingFanLevel;
    uint8_t maxCoolingFanLevel;
    uint8_t speakerVolume;
    uint16_t forceUpTimeout;
    uint16_t forceOnTimeout;
    uint16_t forceDownTimeout;
    uint16_t waitingTimeout;
    uint8_t poseDetectionDelay;
    uint8_t objectDetectionDelay;
    uint8_t gyroActiveAngle;
    uint8_t gyroRelativeAngle;
    uint8_t currentModeValue;
} SystemConfig_t;

class biosigDataClass {
    public:

        void resetBuffers(void);
        uint16_t addData(allData_t allData); // adds new data to buffers including torsoAngle & does tilt calcualtion
        void calculateAngles(void);

        int16_t adsRBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroRXBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroRYBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroRZBuffer[BIOSIG_BUFFER_LEN];

        int16_t adsLBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroLXBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroLYBuffer[BIOSIG_BUFFER_LEN];
        int16_t gyroLZBuffer[BIOSIG_BUFFER_LEN];
        
        // std::vector<uint8_t> breathingRateLBuffer;
        // std::vector<uint8_t> breathingRateRBuffer;
        
        float torsoAngleBuffer[BIOSIG_BUFFER_LEN];

        uint16_t index = 0;

        float maxAccel10sec[ACCEL_BUFFER_LEN];
        float minAccel10sec[ACCEL_BUFFER_LEN];

        uint16_t accelIndex = 0;
};

#include "data.h"
#define BIOSIGNAL_SAMPLING_HZ 10
#define TINYML_BUFFER_LEN 30
#define TOF_THRESHOLD 175
#define ADJUST_TINYML_PREDICTION
class tinyMLDataClass {
    public:

        void resetBuffers(void);
        uint16_t addData(allData_t allData);
        void calculateTofMin(void);
        bool calculateTofSlope(int16_t threshold, int duration);

        int16_t adsRBuffer[TINYML_BUFFER_LEN];
        int16_t gyroRXBuffer[TINYML_BUFFER_LEN];
        int16_t gyroRYBuffer[TINYML_BUFFER_LEN];
        int16_t gyroRZBuffer[TINYML_BUFFER_LEN];

        int16_t adsLBuffer[TINYML_BUFFER_LEN];
        int16_t gyroLXBuffer[TINYML_BUFFER_LEN];
        int16_t gyroLYBuffer[TINYML_BUFFER_LEN];
        int16_t gyroLZBuffer[TINYML_BUFFER_LEN];

        int16_t tofBuffer[TINYML_BUFFER_LEN];
        int16_t tof1SecMin;

        // int16_t stepCount;
        // int16_t gaitCount;
        // int16_t fallDetected;

        int16_t dummy[TINYML_BUFFER_LEN];
    
        uint16_t index = 0;
    
};

#include <Ticker.h>
#include <set>
#include <array>
#include <algorithm>
#include <ArduinoJson.h>
#include <ESP32Time.h>
#include <Preferences.h>


extern TaskHandle_t irqHandlerTask; // sendData, Button
extern TaskHandle_t TaskBle_h;
extern TaskHandle_t TasUsartMaster_h;
extern TaskHandle_t TaskGnss_h;
extern TaskHandle_t TaskLte_h;
extern TaskHandle_t TaskSubscribe_h;
extern TaskHandle_t TaskPublish_h;
extern TaskHandle_t TaskTimer_h;
extern TaskHandle_t TaskBiosigProcess_h;

extern configData_t *cfg;
extern Preferences preferences;
extern SemaphoreHandle_t reportReadySemaphore;
extern SemaphoreHandle_t processBiosignalsSemaphore;


extern allData_t allData;
extern SystemConfig_t systemConfig;
extern tinyMLDataClass predictionInput;
extern biosigDataClass biosigData1Min;


#endif
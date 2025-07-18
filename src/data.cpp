#include "data.h"

static const char TAG[] = __FILE__;

uint8_t breathingRate = 0;
const int BUFFER_SIZE = 6;
uint8_t breathingRateRSmoothed;
uint8_t breathingRateLSmoothed;

uint32_t step_count = 0;
uint8_t walkTimeSec = 0;

void biosigProcessTask(void *pvParameters) {
    while (1) {
        // Wait for a signal that a report is ready to be sent
        if (xSemaphoreTake(processBiosignalsSemaphore, portMAX_DELAY) == pdTRUE) {

#ifdef SENSORS_STM_CONNECTED
            // calculate torsoAngleMean & torsoAngleMin
            getTorsoAngleStats(biosigData1Min.torsoAngleBuffer);

            // calculate bentDuration -> simple multiple of lmaCount?
            
            // calculate stepCount?? walkTimeSec??

/// NOTE: change this to single min & max that refreshes every X seconds
            // get max & min accelerations for last 10 seconds
            auto maxElementIt = std::max_element(std::begin(biosigData1Min.maxAccel10sec), std::end(biosigData1Min.maxAccel10sec));
            float largestAccel = *maxElementIt;
            auto minElementIt = std::max_element(std::begin(biosigData1Min.minAccel10sec), std::end(biosigData1Min.minAccel10sec));
            float smallestAccel = *minElementIt; // Dereference the iterator to get the value

/// NOTE: since this loop runs only once every 60s, we might need to increase the accel thresholds            
            // calculate breathing rate & smooth it
            if (largestAccel < 3 && largestAccel - smallestAccel < 2) {
                uint8_t breathingRateL = calculateBR(biosigData1Min.adsLBuffer);
                // addBreathingRate(breathingRateL, biosigData1Min.breathingRateLBuffer);
                // breathingRateLSmoothed = getAverageBreathingRate(biosigData1Min.breathingRateLBuffer);
                uint8_t breathingRateR = calculateBR(biosigData1Min.adsRBuffer);
                // addBreathingRate(breathingRateR, biosigData1Min.breathingRateRBuffer);
                // breathingRateRSmoothed = getAverageBreathingRate(biosigData1Min.breathingRateRBuffer);
                allData.breathRate = (breathingRateL + breathingRateR) / 2;
            }

            // calculate energyExpenditure?
#endif

            // Signal the publisher task to send the report
            xSemaphoreGive(reportReadySemaphore);
        }
    }
}

void initBiosignalProcessing() {

    // setupBreathingRateBuffer(biosigData1Min.breathingRateLBuffer);
    // setupBreathingRateBuffer(biosigData1Min.breathingRateRBuffer);

    xTaskCreatePinnedToCore(biosigProcessTask, 
                            "biosigProcessTask", 
                            PROCESS_BIOSIG_TASK_STACK, 
                            NULL, 
                            PROCESS_BIOSIG_TASK_PRI, 
                            &TaskBiosigProcess_h, 
                            PROCESS_BIOSIG_TASK_CORE);
    ESP_LOGI(TAG, "Biosignal process task initialized...");
}

void biosigDataClass::resetBuffers(void) {
    memset(adsRBuffer, 0, sizeof(adsRBuffer));  // Set all elements to 0
    memset(gyroRXBuffer, 0, sizeof(gyroRXBuffer)); 
    memset(gyroRYBuffer, 0, sizeof(gyroRYBuffer)); 
    memset(gyroRZBuffer, 0, sizeof(gyroRZBuffer)); 
    memset(adsLBuffer, 0, sizeof(adsLBuffer));  // Set all elements to 0
    memset(gyroLXBuffer, 0, sizeof(gyroLXBuffer)); 
    memset(gyroLYBuffer, 0, sizeof(gyroLYBuffer)); 
    memset(gyroLZBuffer, 0, sizeof(gyroLZBuffer)); 
    index = 0;  
}

uint16_t biosigDataClass::addData(allData_t allData) {
    if (index > BIOSIG_BUFFER_LEN) {
        index = BIOSIG_BUFFER_LEN;
        for (int i = 0; i < BIOSIG_BUFFER_LEN-1; i++) {
            adsRBuffer[i] = adsRBuffer[i + 1];
            gyroRXBuffer[i] = gyroRXBuffer[i + 1];
            gyroRYBuffer[i] = gyroRYBuffer[i + 1];
            gyroRZBuffer[i] = gyroRZBuffer[i + 1];
            adsLBuffer[i] = adsLBuffer[i + 1];
            gyroLXBuffer[i] = gyroLXBuffer[i + 1];
            gyroLYBuffer[i] = gyroLYBuffer[i + 1];
            gyroLZBuffer[i] = gyroLZBuffer[i + 1];
            torsoAngleBuffer[i] = torsoAngleBuffer[i + 1];
        }
    }    
    
    adsRBuffer[index] = allData.R_ads;
    gyroRXBuffer[index] = allData.R_gyro[0];
    gyroRYBuffer[index] = allData.R_gyro[1];
    gyroRZBuffer[index] = allData.R_gyro[2];
    
    adsLBuffer[index] = allData.L_ads;
    gyroLXBuffer[index] = allData.L_gyro[0];
    gyroLYBuffer[index] = allData.L_gyro[1];
    gyroLZBuffer[index] = allData.L_gyro[2];

    // calculate tilt angle
    float accelL_x_g = allData.L_accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accelL_y_g = allData.L_accel[1] / 8192.0f;  // 16g range -> 2048
    float accelL_z_g = allData.L_accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyroL_y_dps = allData.L_gyro[1] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float accelR_x_g = allData.R_accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accelR_y_g = allData.R_accel[1] / 8192.0f;  // 16g range -> 2048
    float accelR_z_g = allData.R_accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyroR_y_dps = allData.R_gyro[1] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float torsoAngle = tilt_calculation(accelL_x_g, accelL_y_g, accelL_z_g, gyroL_y_dps, accelR_x_g, accelR_y_g, accelR_z_g, gyroR_y_dps);
    
    torsoAngleBuffer[index] = torsoAngle;

    std::vector<float> accelValues;
    accelValues.push_back(accelL_x_g);
    accelValues.push_back(accelL_y_g);
    accelValues.push_back(accelL_z_g);
    accelValues.push_back(accelR_x_g);
    accelValues.push_back(accelR_y_g);
    accelValues.push_back(accelR_z_g);
    // Find the minimum and maximum elements using std::minmax_element
    // std::minmax_element returns a pair of iterators to the min and max elements
    auto minMaxIt = std::minmax_element(accelValues.begin(), accelValues.end());
    float minAccel = *minMaxIt.first;  // Dereference the iterator to get the value
    float maxAccel = *minMaxIt.second; // Dereference the iterator to get the value

    if (accelIndex > ACCEL_BUFFER_LEN) {
        accelIndex = ACCEL_BUFFER_LEN;
        for (int i = 0; i < ACCEL_BUFFER_LEN-1; i++) {
            maxAccel10sec[i] = maxAccel10sec[i + 1];
            minAccel10sec[i] = minAccel10sec[i + 1];
        }
    }    
    maxAccel10sec[accelIndex] = maxAccel;
    minAccel10sec[accelIndex] = minAccel;
    accelIndex += 1;
    
    // Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", adsRBuffer[index], gyroRXBuffer[index], gyroRYBuffer[index], gyroRZBuffer[index], adsLBuffer[index], gyroLXBuffer[index], gyroLYBuffer[index], gyroLZBuffer[index], tofBuffer[index]);
    index += 1;

    return index;
}

void tinyMLDataClass::resetBuffers(void) {
    
    memset(adsRBuffer, 0, sizeof(adsRBuffer));  // Set all elements to 0
    memset(gyroRXBuffer, 0, sizeof(gyroRXBuffer)); 
    memset(gyroRYBuffer, 0, sizeof(gyroRYBuffer)); 
    memset(gyroRZBuffer, 0, sizeof(gyroRZBuffer)); 
        
    memset(adsLBuffer, 0, sizeof(adsLBuffer));  // Set all elements to 0
    memset(gyroLXBuffer, 0, sizeof(gyroLXBuffer)); 
    memset(gyroLYBuffer, 0, sizeof(gyroLYBuffer)); 
    memset(gyroLZBuffer, 0, sizeof(gyroLZBuffer)); 

    memset(tofBuffer, 0, sizeof(tofBuffer)); 
    memset(dummy, 0, sizeof(dummy)); 

    index = 0;  
}

uint16_t tinyMLDataClass::addData(allData_t allData) {
    // if (index > TINYML_BUFFER_LEN) { index = TINYML_BUFFER_LEN; }

    if (index > TINYML_BUFFER_LEN) {
        index = TINYML_BUFFER_LEN;
        for (int i = 0; i < TINYML_BUFFER_LEN-1; i++) {
            adsRBuffer[i] = adsRBuffer[i + 1];
            gyroRXBuffer[i] = gyroRXBuffer[i + 1];
            gyroRYBuffer[i] = gyroRYBuffer[i + 1];
            gyroRZBuffer[i] = gyroRZBuffer[i + 1];
            adsLBuffer[i] = adsLBuffer[i + 1];
            gyroLXBuffer[i] = gyroLXBuffer[i + 1];
            gyroLYBuffer[i] = gyroLYBuffer[i + 1];
            gyroLZBuffer[i] = gyroLZBuffer[i + 1];
            tofBuffer[i] = tofBuffer[i + 1];
        }
    }    
    
    adsRBuffer[index] = allData.R_ads;
    gyroRXBuffer[index] = allData.R_gyro[0];
    gyroRYBuffer[index] = allData.R_gyro[1];
    gyroRZBuffer[index] = allData.R_gyro[2];
    
    adsLBuffer[index] = allData.L_ads;
    gyroLXBuffer[index] = allData.L_gyro[0];
    gyroLYBuffer[index] = allData.L_gyro[1];
    gyroLZBuffer[index] = allData.L_gyro[2];

    tofBuffer[index] = allData.lmaLength;

    ESP_LOGI(TAG, "%d, %d, %d, %d, %d, %d, %d, %d, %d", adsRBuffer[index], gyroRXBuffer[index], gyroRYBuffer[index], gyroRZBuffer[index], adsLBuffer[index], gyroLXBuffer[index], gyroLYBuffer[index], gyroLZBuffer[index], tofBuffer[index]);
    index += 1;

    return index;
}

void tinyMLDataClass::calculateTofMin() {

    int16_t minVal = tofBuffer[TINYML_BUFFER_LEN-10];
    for (int i = TINYML_BUFFER_LEN-10+1; i < TINYML_BUFFER_LEN; i++) {
        if (tofBuffer[i] < minVal) {
            minVal = tofBuffer[i];
        }
    }
    tof1SecMin = minVal;
}

bool tinyMLDataClass::calculateTofSlope(int16_t threshold, int duration) {
    bool prediction = false;
    int16_t slope = 0;
    for (int i = TINYML_BUFFER_LEN-1-duration; i < TINYML_BUFFER_LEN-1; i++) {
        slope += tofBuffer[i+1]-tofBuffer[i];
        ESP_LOGI(TAG, "d_i+1 - d_i: %d", tofBuffer[i+1]-tofBuffer[i]);
    }
    slope /= duration;
    if (slope >= threshold) {
        prediction = true;
    }
    ESP_LOGI(TAG, "slope: %d", slope);

    return prediction;
}
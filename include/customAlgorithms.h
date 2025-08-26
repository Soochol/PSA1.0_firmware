#ifndef _ALGORITHMS_H
#define _ALGORITHMS_H

#include "globals.h"
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <vector> // For std::vector
#include <numeric> // For std::accumulate

/* Test Settings */
#ifdef DEV_MODE
    #ifdef TEST_MODE
        // #define ALGORITHM_TEST
    #endif
#endif

#define BR_BUFFER_SIZE 6

uint8_t calculateBR(int16_t inputArray[600]);  // breathing rate, motion range
int sort_desc(const void *cmp1, const void *cmp2);
uint8_t convert_int16_to_uint8(int16_t value);
float tilt_calculation(float ax1, float ay1, float az1, float gyro_y1, float ax2, float ay2, float az2, float gyro_y2);

typedef struct {
    float angle; // The filtered angle
    float bias;
    float rate;
    float P[2][2];
} Kalman_t;

void initKalman(Kalman_t* kf);

// void setupBreathingRateBuffer(std::vector<uint8_t>& breathingRateBuffer);
// void addBreathingRate(uint8_t newRate, std::vector<uint8_t>& breathingRateBuffer);
uint8_t getAverageBreathingRate(std::vector<uint8_t>& breathingRateBuffer);
void getTorsoAngleStats(float torsoAngleBuffer[]);

#endif

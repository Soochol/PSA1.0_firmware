#include "customAlgorithms.h"

static const char TAG[] = __FILE__;

// breathing rate algorithm
// original array length 60s
// 1) find smoothing window average (window size = 12s, slowest breathing rate at 5breaths/min)
// 2) subtract result of 1 from original array in the middle (new array has length 48s)
// 3) find first zero crossing of new array (for loop to find element[i-1]<0 and element[i]>0)
// 4) find maximum of the array segment after the first zero crossing, and before the second zero crossing
// 5) repeat for all zero crossings with element[i] > 0
// 6) find the distance between the local maxima, get median distance, convert to breathing rate

uint8_t calculateBR(int16_t inputArray[BIOSIG_BUFFER_LEN]) {

    int dummyHz = BIOSIGNAL_SAMPLING_HZ;
    
    int minPeakDistance = 1.5*dummyHz; // 50BPM
    int maxPeakDistance = 12*dummyHz; // 5BPM

    int16_t maximum = -32765;
    int16_t minimum = 32765;
    int16_t range = 0;
    int smoothingFactor = minPeakDistance; // int(IMU_BUFFER_HZ*2 /2);      // to remove high frequency noise
    int decimationFactor = 1; // int(IMU_BUFFER_HZ/2); // apply decimation of approx 0.5s
    int smoothedArrayLength = (BIOSIG_BUFFER_LEN)/decimationFactor;
    int16_t smoothedArray[smoothedArrayLength];         // middle 45s array is 100 samples long
    for (size_t i = 0; i < smoothedArrayLength; i++) {
		smoothedArray[i] = 0;
	}
    int newIndexCounter = smoothingFactor;
    
    // for (size_t i=0; i<BIOSIG_BUFFER_LEN; i+=decimationFactor) {
    //     Serial.printf("%d,", inputArray[i]);
    // }
    // Serial.println("");
    
    for (size_t i=0; i<BIOSIG_BUFFER_LEN; i+=decimationFactor) {
        maximum = std::max(inputArray[i], maximum);
        minimum = std::min(minimum, inputArray[i]);
        if (i < smoothingFactor) {
            smoothedArray[i] = 0;
        }
        else if (i >= smoothingFactor && i < smoothedArrayLength - smoothingFactor) {
// smoothing between i-window to i+window
            int smoothedValue = 0;
            for (int j=i-smoothingFactor; j<=i+smoothingFactor; j++) {
                smoothedValue += inputArray[j];
            }
            smoothedValue = smoothedValue / (smoothingFactor*2);
            smoothedArray[newIndexCounter++] = smoothedValue; // - movingWindowValue;
        }
        else {
            smoothedArray[i] = 0;
        }
    }
    range = maximum - minimum;
    
    float breathingRate = 0;
    if (range > 50) {
    // Creating a buffer to store the distances (maximum 30 per minute)
        uint8_t cyclesDistances[60];
        uint8_t cyclesCount = 0;
        for (int i=smoothingFactor+2; i<smoothedArrayLength-2; i++) {
            if (smoothedArray[i] > smoothedArray[i-1] && smoothedArray[i] >= smoothedArray[i-2] && smoothedArray[i] >= smoothedArray[i+1] && smoothedArray[i] >= smoothedArray[i+2]) {
                for (int j=i+1; j<smoothedArrayLength; j++) {
                    if (smoothedArray[j] > smoothedArray[j-1] && smoothedArray[j] >= smoothedArray[j-2] && smoothedArray[j] >= smoothedArray[j+1] && smoothedArray[j] >= smoothedArray[j+2]) {
                        if ((j - i)*decimationFactor > minPeakDistance && (j - i)*decimationFactor < maxPeakDistance) {
                            cyclesDistances[cyclesCount++] = (j - i)*decimationFactor;
                            // Serial.print(i);
                            // Serial.print(", ");
                            // Serial.println(j);	
                            i = j;
                            break;
                        }
                    }
                }
            }
        }

    // getting a portion of the above buffer that is filled with values
        uint8_t cyclesDistances2[cyclesCount];
        Serial.print('\t');
        Serial.print("distances: ");
        for (int i=0; i<cyclesCount; i++) {
            cyclesDistances2[i] = cyclesDistances[i];
            Serial.print(cyclesDistances2[i]);
            Serial.print(",");
        }
        Serial.println("");
        // Number of items in the array
        int lt_length = sizeof(cyclesDistances2) / sizeof(cyclesDistances2[0]);
        // qsort - last parameter is a function pointer to the sort function
        qsort(cyclesDistances2, lt_length, sizeof(cyclesDistances2[0]), sort_desc);

        float medianDistance = cyclesDistances2[int(cyclesCount/2)];
        if (medianDistance > minPeakDistance) {
            breathingRate = 60/(medianDistance/dummyHz);
        }
    }
    
    printf("max %d, min %d, range %d, range_uint8: %u\n", maximum, minimum, range, convert_int16_to_uint8(range));
    printf("Breathing rate: %.2f\n", breathingRate);

    return (uint8_t)(breathingRate + 0.5f);
}

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

uint8_t convert_int16_to_uint8(int16_t value) {
    return (uint8_t)((float(value) / 65535.0) * 255);
    // return (uint8_t)((float(value) / 16384.0) * 255);
}

extern Kalman_t kalmanL;
extern Kalman_t kalmanR;
float dt = 0.1f; // 100ms loop
// uint8_t tilt_count = 0;
int tilted = 0; // 0 = neutral, 1 = tilted

void initKalman(Kalman_t* kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

float Kalman_Update(Kalman_t* kf, float newAngle, float newRate, float dt) {
    // Predict
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + 0.001f);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += 0.003f * dt;

    // Measurement update
    float S = kf->P[0][0] + 0.03f;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

float tilt_calculation(float ax1, float ay1, float az1, float gyro_y1, float ax2, float ay2, float az2, float gyro_y2) {
    // Estimate pitch from accelerometer
    float accel_pitchL = atan2f(ax1, sqrtf(ay1 * ay1 + az1 * az1)) * 180.0f / M_PI;
    float accel_pitchR = atan2f(ax2, sqrtf(ay2 * ay2 + az2 * az2)) * 180.0f / M_PI;
    // Gyro is in deg/sec
    float filtered_pitchL = Kalman_Update(&kalmanL, accel_pitchL, gyro_y1, dt);
    float filtered_pitchR = Kalman_Update(&kalmanR, accel_pitchR, gyro_y2, dt);

    float averagePitch = (filtered_pitchL + filtered_pitchR) / 2;

    // Threshold logic
    float TILT_THRESHOLD = 60.0f;

    // printf("Kalman Pitch: %.2f\n", filtered_pitch);

    if (fabs(averagePitch) < TILT_THRESHOLD && !tilted) {
        allData.tilt_count++;
        tilted = 1;    
        printf("Tilt: %.2f, Count: %d\n", averagePitch, allData.tilt_count);
    } else if (fabs(averagePitch) > 80.0f) {
        tilted = 0; // Reset state when close to neutral
    }
    return averagePitch;
}

void setupBreathingRateBuffer(std::vector<uint8_t>& breathingRateBuffer) {
    breathingRateBuffer.reserve(BR_BUFFER_SIZE); // Pre-allocate memory
    // Optionally, fill with zeros initially
    for (int i = 0; i < BR_BUFFER_SIZE; ++i) {
        breathingRateBuffer.push_back(0);
    }
}

void addBreathingRate(uint8_t newRate, std::vector<uint8_t>& breathingRateBuffer) {
    // If the buffer is full, remove the oldest element
    if (breathingRateBuffer.size() >= BR_BUFFER_SIZE) {
        breathingRateBuffer.erase(breathingRateBuffer.begin()); // Remove the first element
    }
    // Add the new element to the end
    breathingRateBuffer.push_back(newRate);
}


uint8_t getAverageBreathingRate(std::vector<uint8_t>& breathingRateBuffer) {
    // Step 1: Collect all non-zero breathing rates
    std::vector<uint8_t> nonZeroRates;
    Serial.print('\t');
    Serial.print("BR values: ");
    for (uint8_t rate : breathingRateBuffer) {
        if (rate != 0) {
            nonZeroRates.push_back(rate);
            Serial.print(rate);
            Serial.print(",");
        }
    }
    Serial.println("");

    // Step 2: Sort the non-zero rates in ascending order
    std::sort(nonZeroRates.begin(), nonZeroRates.end());

    // Step 3: Check if there are enough non-zero values to average the middle 2
    if (nonZeroRates.size() < 2) {
        // If there are fewer than 2 non-zero values, we cannot average the "middle 2".
        return 0;
    }

    uint32_t sum = 0; // Use uint32_t to prevent overflow for sum
    int count = 2;    // We are always averaging 2 values if we proceed

    // Step 4: Determine the indices for the middle 2 values
    size_t size = nonZeroRates.size();
    size_t index1;
    size_t index2;

    if (size % 2 != 0) {
        // If size is odd (e.g., 3, 5, 7): median and median+1
        // Median index for odd size N is N/2 (integer division)
        index1 = size / 2;
        index2 = index1 + 1;
        if (nonZeroRates[size-1] < nonZeroRates[size/2]) {
            index2 = index1 - 1;
        }
        // Example: size = 5 -> median index = 2. index1 = 2, index2 = 3.
        // Takes elements at index 2 and 3.
    } else {
        // If size is even (e.g., 2, 4, 6): the two middle values
        // Middle two indices for even size N are N/2 - 1 and N/2
        index1 = (size / 2) - 1;
        index2 = size / 2;
        // Example: size = 4 -> index1 = 1, index2 = 2.
        // Takes elements at index 1 and 2.
    }

    // Step 5: Sum the 2 middle values
    sum += nonZeroRates[index1];
    sum += nonZeroRates[index2];

    // Step 6: Calculate the average (always divide by 3 now)
    float average = static_cast<float>(sum) / 2.0f;

    // Step 7: Return the result as uint8_t (truncates decimals)
    return static_cast<uint8_t>(average);
}


void getTorsoAngleStats(float torsoAngleBuffer[]) {
    std::vector<float> nonZeroRates;
    Serial.print('\t');
    Serial.print("BR values: ");
    for (int i=0; i < BIOSIG_BUFFER_LEN; i++) {
        if (torsoAngleBuffer[i] != 0) {
            nonZeroRates.push_back(torsoAngleBuffer[i]);
            Serial.print(torsoAngleBuffer[i]);
            Serial.print(",");
        }
        if (torsoAngleBuffer[i] == 0) { break; }
    }
    Serial.println("");

    // Step 2: Sort the non-zero rates in ascending order
    std::sort(nonZeroRates.begin(), nonZeroRates.end());

    // Step 3: Check if there are enough non-zero values to average the middle 2
    if (nonZeroRates.empty()) {
        // If there are fewer than 2 non-zero values, we cannot average the "middle 2".
        allData.torsoAngleMean = 0;
        allData.torsoAngleMin = 0;  
    }
    
    float sum = 0;
    float minimum = 1000;
    for (int i=0; i<nonZeroRates.size(); i++) {
        minimum = std::min(minimum, nonZeroRates[i]);
        sum += nonZeroRates[i];
    }
    float mean = sum / nonZeroRates.size();

    allData.torsoAngleMean = static_cast<int16_t>(mean);
    allData.torsoAngleMin = static_cast<int16_t>(minimum);    
}
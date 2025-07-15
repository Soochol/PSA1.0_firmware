#ifndef _TINYML_H
#define _TINYML_H

#include "globals.h"
#include <vector>
#include <Arduino.h>
#include "class_should_model2D_20250311.h"
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

#define ARENA_SIZE 30000


void predictSample(const char *classLabel, float *input, uint8_t expectedOutput);
void tinyMLExample();

void initTinyML();
float scaleValue(float x, float minVal, float maxVal, float targetMin, float targetMax);
std::vector<float> normalization(int16_t* data);
uint8_t tinyMLInference(tinyMLDataClass predictionInput);

#endif
#ifndef _SAMM10Q_H
#define _SAMM10Q_H

#include "globals.h"
#include "Wire.h"
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

void initGNSS();
void fetchLocation();

int16_t getScaledLatitude_int16(int32_t raw_latitude_scaled_by_10e7);

#endif

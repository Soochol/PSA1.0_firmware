#ifndef _CONFIGMANAGER_H
#define _CONFIGMANAGER_H

#include "globals.h"
#include "resetCheck.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "string.h"
#include "type1scMqttLte.h"

void makeThingName();
void setEndpoints(configData_t *configData);

#endif
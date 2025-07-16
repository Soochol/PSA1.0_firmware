#ifndef _TYPE1SC_H
#define _TYPE1SC_H

#include "TYPE1SC.h"
#include "aws_credentials.h"
#include <Arduino.h>
#include "payload.h"

#define MQTT_HOST "a34xzhba8w36lz-ats.iot.ap-northeast-2.amazonaws.com"
#define MQTT_PORT 8883
#define MQTT_PORT_srt "8883"

#define MQTT_SUBSCRIBE_TOPIC    "/subTopic"

#define MQTT_PUBLISH_POWERON    "/pubPowerOnTopic"
#define MQTT_PUBLISH_REPORT     "/pubReportTopic"
#define MQTT_PUBLISH_EMERGENCY  "/pubEmergencyTopic"

// firmware to server
#define METHOD_UNDEFINED_MSG 20
#define METHOD_POWERON 21
#define METHOD_REPORT 22
#define METHOD_EMERGENCY 23

// Server to firmware
#define METHOD_RESET 11
#define METHOD_CHECK 12
#define METHOD_UPDATE 13
#define METHOD_ACTIVATE 14  // NOTE: THIS NUMBER IS NOT ACTUALLY USED IN THE SERVER, the message itself is picked up, not the number
#define METHOD_DEACTIVATE 15

/* EXT_ANT_ON 0 : Use an internal antenna.
 * EXT_ANT_ON 1 : Use an external antenna.
 */
#define EXT_ANT_ON 0

#define PWR_PIN 5
#define RST_PIN 18
#define WAKEUP_PIN 19
#define EXT_ANT 4
#define EXT_ANT_ON 0

void setSerials();
void extAntenna();
void initType1sc();
void type1scSetAPN();
void type1scSetCertificates();
void type1scPrintStrength();
void type1scBasicTest();
void type1scHttpTest();
void type1scConnectAws();
void type1scRegisterCallback();

bool type1scReport();
bool type1scPowerOn();
bool type1scEmergency();

#endif
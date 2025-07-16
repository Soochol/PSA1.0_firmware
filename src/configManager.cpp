#include "configManager.h"

static const char TAG[] = __FILE__;

extern configData_t *cfg;

void makeThingName() {
    byte mac[6];
    byte macBle[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    esp_read_mac(macBle, ESP_MAC_BT);
    sprintf(cfg->thing_name, "WFPSA%02X%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], macBle[5]);
    DebugSerial.printf("\n\ndevice id: %s\n", cfg->thing_name);
}

void setEndpoints(configData_t *configData) {
    sprintf(cfg->subTopic, "%s%s", cfg->thing_name, MQTT_SUBSCRIBE_TOPIC);
    sprintf(cfg->pubPowerOnTopic, "%s%s", cfg->thing_name, MQTT_PUBLISH_POWERON);
    sprintf(cfg->pubReportTopic, "%s%s", cfg->thing_name, MQTT_PUBLISH_REPORT);
    sprintf(cfg->pubEmergencyTopic, "%s%s", cfg->thing_name, MQTT_PUBLISH_EMERGENCY);
}
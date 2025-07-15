#include "resetCheck.h"
#include <rom/rtc.h>

static const char TAG[] = __FILE__;

JsonObject reset_json;

void restartApp(reset_reason_t reason) {
    vTaskDelay(1500 / portTICK_PERIOD_MS);

	ESP_LOGE(TAG, "restart_app reason: [%d]", reason);
	ESP.restart();
}

void deviceResetManu() {
    restartApp(rst_device_reset_manu);
}
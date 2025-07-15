#include "samM10Q.h"

static const char TAG[] = __FILE__;

SFE_UBLOX_GNSS myGNSS;
long lastTime = 0;

void gnssHandler(void *pvParameters) {

    while (1) {
        fetchLocation();
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

void initGNSS() {

    // myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    if (myGNSS.begin(Wire1) == false) //Connect to the u-blox module using Wire port
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  
    xTaskCreatePinnedToCore(gnssHandler,
                            "gnssHandler",
                            GNSS_TASK_STACK,
                            NULL,
                            GNSS_TASK_PRI,
                            &TaskGnss_h,
                            GNSS_TASK_CORE);
    ESP_LOGI(TAG, "GNSS initialized...");
}

void fetchLocation() {
    // Request (poll) the position, velocity and time (PVT) information.
    // The module only responds when a new position is available. Default is once per second.
    // getPVT() returns true when new data is received.
    if (myGNSS.getPVT() == true)
    {
        int32_t rawLatitude = myGNSS.getLatitude();
        // Serial.print(F("Lat: "));
        allData.latitude = getScaledLatitude_int16(rawLatitude);
        // Serial.println(latitude);

        int32_t rawLongitude = myGNSS.getLongitude();
        // Serial.print(F(" Long: "));
        allData.longitude = getScaledLatitude_int16(rawLongitude);
        // Serial.println(longitude);
    }
}

int16_t getScaledLatitude_int16(int32_t raw_latitude_scaled_by_10e7) {
    // Step 1: Convert the raw int32_t to a float and
    //         scale it down from 10^7 to 10^2 (divide by 10^5 = 100,000).
    // Example: 327944333 / 100000.0f = 3279.44333
    // Example: -101234567 / 100000.0f = -1012.34567
    float temp_float_val_100x = static_cast<float>(raw_latitude_scaled_by_10e7) / 100000.0f;

    // Step 2: Round to the nearest whole number.
    // Example:
    //   - std::round(3279.44333) = 3279.0f
    //   - std::round(3279.50000) = 3280.0f
    //   - std::round(-1012.34567) = -1012.0f
    //   - std::round(-1012.50000) = -1013.0f (rounds away from zero for .5)
    float rounded_value_float = std::round(temp_float_val_100x);

    // Step 3: Cast to int16_t.
    // Ensure the value is within the int16_t range (-32768 to +32767).
    // Latitude (degrees * 100) range: -9000 to +9000.
    // This fits perfectly within int16_t.
    return static_cast<int16_t>(rounded_value_float);
}

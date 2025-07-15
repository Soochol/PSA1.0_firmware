#include "timer.h"

static const char TAG[] = __FILE__;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
    // ESP_LOGI(TAG, "Timer triggered...");
}

void timerHandler(void *pvParameters) {

    while (1) {
        if (interruptCounter > 0) {
            portENTER_CRITICAL(&timerMux);
            interruptCounter--;
            portEXIT_CRITICAL(&timerMux);

            // ESP_LOGI(TAG, "Entering work block at %lu ms", millis());
            // // ... critical section ...
            // unsigned long start_report = millis();
            // if (type1scReport() == true) {
            //     ESP_LOGI(TAG, "6.Published report to broker");
            // } else {
            //     ESP_LOGI(TAG, "6.Something went wrong");
            // }
            // ESP_LOGI(TAG, "type1scReport() completed in %lu ms", millis() - start_report);

            ESP_LOGI(TAG, "Entering work block at %lu ms", millis());

            // Prepare payload here or signal the publisher task to do so
            // For example, if payload involves sensor data, collect it here.
            // Then, send a message to a queue that the publisher task listens to.
            // xQueueSend(publisherQueue, &payloadData, portMAX_DELAY); // Or just a flag/semaphore

            
            xSemaphoreGive(processBiosignalsSemaphore); 
            
            // xSemaphoreGive(reportReadySemaphore); // Assuming you've created a semaphore

            // This task now completes quickly
            ESP_LOGI(TAG, "timerHandler work block completed.");

            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void initTimer() {
   
    timer = timerBegin(0, TIMER_DIVIDER_MICRO, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000*60, true);
    timerAlarmEnable(timer);
  
    xTaskCreatePinnedToCore(timerHandler,
                            "timerHandler",
                            TIMER_TASK_STACK,
                            NULL,
                            TIMER_TASK_PRI,
                            &TaskTimer_h,
                            TIMER_TASK_CORE);
    ESP_LOGI(TAG, "Timer initialized...");
}

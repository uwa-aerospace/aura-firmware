#include "gnss.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TAG "GNSS"

SFE_UBLOX_GNSS neo;
SemaphoreHandle_t gnssIrqSemaphore;

void setupGNSS(HardwareSerial &serialPort) {
  if (!neo.begin(serialPort)) {
    ESP_LOGE(TAG, "Failed to initialize GNSS");
    while (1);
  }

  /* SETTINGS WILL PERSIST ONCE INITIALLY SET UP */

  // // Increase baud rate to accommodate higher data rate
  // neo.setSerialRate(460800);
  // serialPort.updateBaudRate(460800);

  // // UBX is more efficient
  // neo.setUART1Output(COM_TYPE_UBX);

  // // Set 25Hz data rate (maximum possible for NEO-M9N)
  // neo.setMeasurementRate(40);
  // neo.setNavigationRate(1);

  // neo.saveConfiguration();

  gnssIrqSemaphore = xSemaphoreCreateBinary();
  if (gnssIrqSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to initialize GNSS");
    while (1);
  }

  serialPort.onReceive(gnssInterrupt);

  ESP_LOGI(TAG, "GNSS module setup successful");
}

void gnssInterrupt() {
  xSemaphoreGive(gnssIrqSemaphore);
}

void GnssTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(gnssIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      if (neo.getPVT()) {
        uint32_t gpsTimeMs = neo.getTimeOfWeek();
        static uint32_t lastTime = 0;

        int year = neo.getYear();
        int month = neo.getMonth();
        int day = neo.getDay();
        int hour = neo.getHour();
        int minute = neo.getMinute();
        int second = neo.getSecond();

        uint32_t delta = gpsTimeMs - lastTime;
        // if (delta != 40)
        ESP_LOGI("GNSS", "Date & Time: %04d-%02d-%02d %02d:%02d:%02d UTC, GPS Time: %lu ms, Delta: %lu ms", year, month, day, hour, minute, second, gpsTimeMs, delta);

        lastTime = gpsTimeMs;
      }
    }
  }
}
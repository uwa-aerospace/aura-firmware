#include "gnss.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "GNSS"

SFE_UBLOX_GNSS neo;

void setupGNSS(HardwareSerial &serialPort) {
  if (!neo.begin(serialPort)) {
    ESP_LOGE(TAG, "Failed to initialize GNSS");
    while (1);
  }

  // Increase baud rate to accommodate higher data rate
  neo.setSerialRate(460800);
  serialPort.updateBaudRate(460800);

  // UBX is more efficient
  neo.setUART1Output(COM_TYPE_UBX);

  // Set 25Hz data rate (maximum possible for NEO-M9N)
  neo.setMeasurementRate(40);
  neo.setNavigationRate(1);

  neo.saveConfiguration();

  neo.setAutoPVTcallbackPtr(&processPvtData);

  ESP_LOGI(TAG, "GNSS module setup successful");
}

void GNSSTask(void *pvParameters) {
  while (1) {
    neo.checkUblox();
    neo.checkCallbacks();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void processPvtData(UBX_NAV_PVT_data_t *ubxData) {
  // int32_t lat = ubxData->lat;
  // int32_t lon = ubxData->lon;
  
  // float latitude = lat / 10000000;
  // float longitude = lon / 10000000;

  uint32_t gpsTimeMs = ubxData->iTOW;
  static uint32_t lastTime = 0;

  int year = ubxData->year;
  int month = ubxData->month;
  int day = ubxData->day;
  int hour = ubxData->hour;
  int minute = ubxData->min;
  int second = ubxData->sec;

  ESP_LOGI("GNSS", "Date & Time: %04d-%02d-%02d %02d:%02d:%02d UTC, GPS Time: %lu ms, Delta: %lu ms", year, month, day, hour, minute, second, gpsTimeMs, gpsTimeMs - lastTime);
  
  lastTime = gpsTimeMs;
}
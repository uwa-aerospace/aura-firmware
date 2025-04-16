#include "gnss.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TAG "GNSS"

SFE_UBLOX_GNSS neo;
SemaphoreHandle_t gnssIrqSemaphore;

volatile bool gnssTaskStarted = false;
void IRAM_ATTR gnssInterrupt(void) {
  // Clear UART FIFO until GnssTask starts, prevents FIFO overflow
  if (!gnssTaskStarted) {
    uart_flush_input(UART_NUM_2);
  }
  xSemaphoreGive(gnssIrqSemaphore);
}

void IRAM_ATTR gnssRxErrorCallback(hardwareSerial_error_t error) {
  // In case GnssTask gets blocked after startup and FIFO overflows
  if (error == UART_FIFO_OVF_ERROR || error == UART_BUFFER_FULL_ERROR)
    uart_flush_input(UART_NUM_2);
}

SetupStatus setupGNSS(HardwareSerial &serialPort) {
  if (!neo.begin(serialPort)) {
    ESP_LOGE(TAG, "Could not connect to GNSS through UART");
    return GNSS_ERROR;
  }

  /* No setup required, GNSS settings are saved to BBR and flash, and persist between startups */
  /* Setup reqs:
   * 460800 baudrate
   * UBX transmission
   * 40ms MEASurement rate, 1 Navigation rate - achieves 25Hz ODR
   * 4 constellation tracking (GPS, GLONASS, Galileo, BeiDou)
   * AIRBORNE_4g dynamic model - CFG-NAV5
  */

  /* Cannot validate constellations tracked, all settings not checked below are self-validated */
  uint16_t measurementRate = neo.getMeasurementRate();
  if (measurementRate != 40) {
    ESP_LOGE(TAG, "GNSS update rate is not 25Hz");
    return GNSS_ERROR;
  }

  uint8_t dynamicModel = neo.getDynamicModel();
  if (dynamicModel != DYN_MODEL_AIRBORNE4g) {
    ESP_LOGE(TAG, "Incorrect dynamic model");
    return GNSS_ERROR;
  }
  
  gnssIrqSemaphore = xSemaphoreCreateBinary();
  if (gnssIrqSemaphore == NULL) {
    ESP_LOGE(TAG, "Could not initialize GNSS semaphore");
    return GNSS_ERROR;
  }

  serialPort.onReceive(gnssInterrupt);
  serialPort.onReceiveError(gnssRxErrorCallback);

  ESP_LOGI(TAG, "GNSS module setup successful");

  return SETUP_OK;
}

void GnssTask(void *pvParameters) {
  // Task is ready to receive data, tell RX interrupt to stop flushing FIFO
  gnssTaskStarted = true;

  while (1) {
    if (xSemaphoreTake(gnssIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      // Allows processing of multiple messages at once
      while (neo.checkUblox()) {
        if (neo.getPVT()) {
          uint32_t gpsTimeMs = neo.getTimeOfWeek();
          static uint32_t lastTime = 0;
  
          int year = neo.getYear();
          int month = neo.getMonth();
          int day = neo.getDay();
          int hour = neo.getHour();
          int minute = neo.getMinute();
          int second = neo.getSecond();
          int32_t lt = neo.getLatitude();
          int32_t ln = neo.getLongitude();
          int32_t al = neo.getAltitudeMSL();
          float lat = lt / 10000000.0;
          float lng = ln / 10000000.0;
          float alt = al / 1000.0;
  
          uint32_t delta = gpsTimeMs - lastTime;
          // if (delta != 40)
          // ESP_LOGI(TAG, "Date & Time: %04d-%02d-%02d %02d:%02d:%02d UTC, GPS Time: %lu ms, Delta: %lu ms", year, month, day, hour, minute, second, gpsTimeMs, delta);
          ESP_LOGI(TAG, "Lat: %.6f, Lng: %.6f, Alt: %.2f, Delta: %lu ms", lat, lng, alt, delta);
          lastTime = gpsTimeMs;
        }
      }
    }
  }
}
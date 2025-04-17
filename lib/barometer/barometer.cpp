#include "barometer.h"
#include <Wire.h>
#include <MS5611.h>
#include "esp_log.h"

#define TAG "BAROMETER"

#define BARO_READ_RATE 20

MS5611 barometer;

TimerHandle_t baroReadTimer;
SemaphoreHandle_t baroReadSemaphore;
void baroTimerCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(baroReadSemaphore);
}

SetupStatus setupBarometer(uint8_t sdaPin, uint8_t sclPin) {
  Wire.begin(sdaPin, sclPin);

  if (!barometer.begin()) {
    ESP_LOGE(TAG, "Could not connect to barometer over I2C");
    return BAROMETER_ERROR;
  }

  barometer.reset(1);
  barometer.setOversampling(OSR_HIGH);

  baroReadSemaphore = xSemaphoreCreateBinary();
  if (baroReadSemaphore == NULL) {
    ESP_LOGE(TAG, "Could not create barometer read semaphore");
    return BAROMETER_ERROR;
  }

  baroReadTimer = xTimerCreate("BaroReadTimer", pdMS_TO_TICKS(BARO_READ_RATE), pdTRUE, (void*)0, baroTimerCallback);
  if (baroReadTimer == NULL) {
    ESP_LOGE(TAG, "Failed to create barometer read timer");
    return BAROMETER_ERROR;
  }
  xTimerStart(baroReadTimer, 0);

  ESP_LOGI(TAG, "Barometer setup successful");

  return SETUP_OK;
}

void BarometerTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(baroReadSemaphore, portMAX_DELAY) == pdTRUE) {
      barometer.read();
      float pressure = barometer.getPressurePascal();
      float temperature = barometer.getTemperature();
  
      float altitude = 44330.0 * (1.0 - pow(pressure / 101325, 0.1903));
  
      ESP_LOGI(TAG, "Temp: %.2f C, Pressure: %.2f hPa, Altitude: %.2f m\n", temperature, pressure, altitude);
    }
  }
}
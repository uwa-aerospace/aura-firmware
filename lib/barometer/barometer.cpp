#include "barometer.h"
#include <Wire.h>
#include <MS5611.h>
#include "esp_log.h"

#define TAG "BAROMETER"

MS5611 barometer;

SetupStatus setupBarometer(uint8_t sdaPin, uint8_t sclPin) {
  Wire.begin(sdaPin, sclPin);

  if (!barometer.begin()) {
    ESP_LOGE(TAG, "Could not connect to barometer over I2C");
  }

  barometer.setOversampling(OSR_ULTRA_HIGH);

  ESP_LOGI(TAG, "Barometer setup successful");

  return SETUP_OK;
}

void BarometerTask(void *pvParameters) {
  if (barometer.read() == MS5611_READ_OK) {
    float pressure = barometer.getPressure();
    float temperature = barometer.getTemperature();

    float altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));

    printf("Temp: %.2f C, Pressure: %.2f hPa, Altitude: %.2f m\n", temperature, pressure, altitude);
  }
  
  vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz sample rate
}
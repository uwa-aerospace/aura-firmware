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
    return BAROMETER_ERROR;
  }

  barometer.reset(1);
  barometer.setOversampling(OSR_HIGH);

  ESP_LOGI(TAG, "Barometer setup successful");

  return SETUP_OK;
}

void BarometerTask(void *pvParameters) {
  while (1) {
    barometer.read();
    float pressure = barometer.getPressurePascal();
    float temperature = barometer.getTemperature();

    float altitude = 44330.0 * (1.0 - pow(pressure / 101325, 0.1903));

    printf("Temp: %.2f C, Pressure: %.2f hPa, Altitude: %.2f m\n", temperature, pressure, altitude);

    // A read at OSR_HIGH will take 11ms on average, add 9ms extra delay to achieve 20ms (50Hz)
    vTaskDelay(pdMS_TO_TICKS(9)); 
  }
}
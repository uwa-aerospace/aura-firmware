#include "barometer.h"
#include <Wire.h>
#include <MS5611.h>
#include "esp_log.h"
#include "Kalman2D.h"
#include "data.h"
#include "vector_type.h"

#define TAG "BAROMETER"

#define BARO_READ_RATE 20

MS5611 barometer;

Kalman2D kfBaro;
#define BARO_MEAS_ERR 0.008
#define PROCESS_VAR   1
#define MEAS_DT       0.02
#define VEL_EMA_ALPHA 0.15

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

  baroReadTimer = xTimerCreate("BaroReadTimer", pdMS_TO_TICKS(BARO_READ_RATE), pdTRUE, NULL, baroTimerCallback);
  if (baroReadTimer == NULL) {
    ESP_LOGE(TAG, "Failed to create barometer read timer");
    return BAROMETER_ERROR;
  }
  xTimerStart(baroReadTimer, 0);

  kfBaro.init(0.0f, 0.0f, BARO_MEAS_ERR, BARO_MEAS_ERR, PROCESS_VAR, BARO_MEAS_ERR, MEAS_DT);

  ESP_LOGI(TAG, "Barometer setup successful");

  return SETUP_OK;
}

uint16_t baroSamplesRequired = 100;
uint16_t baroSamplesCollected = 0;
float padAltitudeSum = 0;

bool shouldCalBaro = true;
bool baroInitialCalibration = false;
uint32_t baroCalCount = 0;
#define BARO_RECAL_THRESHOLD 250 // Recalibrate every 5 seconds whilst armed on the pad

void BarometerTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(baroReadSemaphore, portMAX_DELAY) == pdTRUE) {
      barometer.read();
      baroPressure = barometer.getPressurePascal();
      baroAltitudeMSL = 44330.0 * (1.0 - pow(baroPressure / 101325.0f, 0.1903));

      // Measure pad altitude (0ft) by averaging MSL altitudes
      if (flightState == FLIGHT_ARMED && shouldCalBaro) {
        if (baroSamplesCollected < baroSamplesRequired) {
          padAltitudeSum += baroAltitudeMSL;
          baroSamplesCollected++;
        }
        else if (flightState == FLIGHT_ARMED && accelRaw.mag() < CALIBRATION_APPLY_THRESHOLD) {
          baroInitialCalibration = true;
          baroCalibrationCycle = true;

          baroPadAltitude = padAltitudeSum / baroSamplesRequired;
          padAltitudeSum = 0;

          baroSamplesCollected = 0;
          shouldCalBaro = false;
        }
        else if (baroInitialCalibration) shouldCalBaro = false;
      }

      if (!baroInitialCalibration) continue;

      rawAltitudeAGL = baroAltitudeMSL - baroPadAltitude;

      kfBaro.dt = MEAS_DT;
      kfBaro.predict();
      kfBaro.update(rawAltitudeAGL);

      baroAltitudeAGL = kfBaro.x[0];
      kalmanBaroVel = kfBaro.x[1];
      baroVertVel = VEL_EMA_ALPHA * kalmanBaroVel + (1 - VEL_EMA_ALPHA) * baroVertVel;

      xEventGroupSetBits(sensorEventGroup, BARO_SENSOR_EVENT);
      newBaroValues = true;

      // Only re-calibrate if launch has not been detected and will not be detected soon (i.e. < 1.5g)
      if (baroCalCount >= BARO_RECAL_THRESHOLD && flightState == FLIGHT_ARMED && accelRaw.mag() < CALIBRATION_APPLY_THRESHOLD) {
        shouldCalBaro = true;
        padAltitudeSum = 0;
        baroSamplesCollected = 0;
        baroCalCount = 0;
      }

      if (flightState == FLIGHT_ARMED && !shouldCalBaro) baroCalCount++;

      // static uint64_t lastRead = millis();
      // uint64_t now = millis();
      // int dt = now-lastRead;
      // lastRead = now;

      // ESP_LOGI(TAG, "%d", dt);
      // printf(">RAGL:%.2f\n>KAGL:%.2f\n>KVEL:%.2f\n>SVEL:%.2f\n", rawAltitudeAGL, baroAltitudeAGL, kalmanBaroVel, baroVertVel);
    }
  }
}
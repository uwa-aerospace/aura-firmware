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

Kalman2D kfBaro;
uint64_t lastBaroMeas;

float kfSetupAlt = 0;
bool kfSetup = false;

uint16_t samplesRequired = 100;
uint16_t samplesCollected = 0;

bool shouldCal = true;
float padAltitudeSum = 0;

void setupKfBaro() {
  float baroMeasErr = 850; // +/- 3.5mbar error at 300-1100 mbar from -20C to 85C
  float processVar = 0.5f;
  float dt = 0.02f;
  kfBaro.init(kfSetupAlt, 0.0f, baroMeasErr, baroMeasErr, processVar, baroMeasErr, dt);
  kfSetup = true;
}

void BarometerTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(baroReadSemaphore, portMAX_DELAY) == pdTRUE) {
      barometer.read();
      float pressure = barometer.getPressurePascal();
      float altitude = 44330.0 * (1.0 - pow(pressure / 101325, 0.1903));

      // Average out altitudes for Kalman filter setup
      if (flightState == FLIGHT_ARMED && !kfSetup) {
        if (samplesCollected < samplesRequired) {
          kfSetupAlt += altitude;
          samplesCollected++;
        }
        else {
          kfSetupAlt /= samplesRequired;
          setupKfBaro();
          samplesCollected = 0;
        }
      }

      if (!kfSetup) continue;

      uint64_t now = micros();
      float dt = (now - lastBaroMeas) * 1e-6;
      lastBaroMeas = now;

      kfBaro.dt = dt;

      kfBaro.predict();
      kfBaro.update(altitude);

      baroAltitudeMSL = kfBaro.x[0];
      baroVertVel = kfBaro.x[1];

      // Average out altitudes to find pad altitude, used for AGL altitude
      if (flightState == FLIGHT_ARMED && shouldCal) {
        if (samplesCollected < samplesRequired) {
          padAltitudeSum += baroAltitudeMSL;
          samplesCollected++;
        }
        else {
          padAltitude = padAltitudeSum / samplesRequired;
          shouldCal = false;
          samplesCollected = 0;
        }
      }

      baroAltitudeAGL = baroAltitudeMSL - padAltitude;

      printf(">RA:%.2f\n>KA:%.2f\n>KV:%.2f\n", altitude, baroAltitudeAGL, baroVertVel);
    }
  }
}
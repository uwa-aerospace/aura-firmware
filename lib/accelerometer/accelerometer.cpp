#include <Arduino.h>
#include "SPI.h"
#include "lsm6dsm_reg.h"

#include "accelerometer.h"
#include "sdcard.h"
#include "data.h"
#include "vector_type.h"
#include "quaternion_type.h"

#define TAG "ACCELEROMETER"
#define GRAVITY_ACCEL 9.80665
#define TO_RADIANS (PI / 180)

SPIClass* accelSpi = &SPI;
uint8_t accelCsPin;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  digitalWrite(accelCsPin, LOW);
  accelSpi->transfer(reg & 0x7F);
  while (len--) {
    accelSpi->transfer(*bufp++);
  }
  digitalWrite(accelCsPin, HIGH);
  return 0;
}

int32_t platform_read(void *ctx, uint8_t reg, uint8_t *bufp, uint16_t len) {
  digitalWrite(accelCsPin, LOW);
  accelSpi->transfer(reg | 0x80);
  while (len--) {
    *bufp++ = accelSpi->transfer(0x00);
  }
  digitalWrite(accelCsPin, HIGH);
  return 0;
}

SemaphoreHandle_t accelIrqSemaphore;
void IRAM_ATTR accelInterrupt(void) {
  xSemaphoreGive(accelIrqSemaphore);
}

stmdev_ctx_t dev_ctx;

SetupStatus setupAccelerometer(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t interrupt) {
  accelCsPin = cs;

  pinMode(accelCsPin, OUTPUT);
  digitalWrite(accelCsPin, HIGH);

  accelSpi->begin(sck, miso, mosi, cs);

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;

  uint8_t whoami = 0;
  lsm6dsm_device_id_get(&dev_ctx, &whoami);

  if (whoami != LSM6DSM_ID) {
    ESP_LOGE(TAG, "Could not detect accelerometer");
    return ACCEL_ERROR;
  }

  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);
  vTaskDelay(100);

  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_416Hz);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_416Hz);

  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_16g);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);

  lsm6dsm_xl_power_mode_set(&dev_ctx, LSM6DSM_XL_HIGH_PERFORMANCE);
  lsm6dsm_gy_power_mode_set(&dev_ctx, LSM6DSM_GY_HIGH_PERFORMANCE);

  lsm6dsm_int2_route_t int2_route;
  lsm6dsm_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_drdy_g = PROPERTY_ENABLE;
  lsm6dsm_pin_int2_route_set(&dev_ctx, int2_route);
  lsm6dsm_int_notification_set(&dev_ctx, LSM6DSM_INT_PULSED);

  accelIrqSemaphore = xSemaphoreCreateBinary();
  if (accelIrqSemaphore == NULL) {
    ESP_LOGE(TAG, "Could not initialize accelerometer semaphore");
    return ACCEL_ERROR;
  }

  pinMode(interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt), accelInterrupt, RISING);

  ESP_LOGI(TAG, "Accelerometer setup successful");
  return SETUP_OK;
}

int16_t accel_raw[3];
int16_t gyro_raw[3];

vec3_t accelCalibrationSums(0,0,0);
vec3_t gyroCalibrationSums(0,0,0);
uint16_t samplesRequired = 832;
uint16_t samplesCollected = 0;

/* Represents how to get from sensor-frame gravity (which can be spread across all 3 axes, depending on mounting)
 * To world-frame gravity (which is only on the Z axis)
 * Allows easy measurement of vertical velocity
*/
quat_t gravityRotQuatn;
vec3_t gyroBiases(0,0,0);
vec3_t gyroRadsPerSec;
float accelGravityOffset = 0;

bool shouldCal = true;
bool initialCalibration = false;
uint32_t calCount = 0;

void setGravityRotQuatn() {
  accelCalibrationSums /= samplesRequired;

  vec3_t sensorGravityVec(accelCalibrationSums);
  sensorGravityVec = sensorGravityVec.norm();

  vec3_t worldGravityVec(0.0f, 0.0f, 1.0f);

  float dotProd = sensorGravityVec.dot(worldGravityVec);
  if (dotProd >= 0.9999f) {
    gravityRotQuatn = quat_t(1.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  if (dotProd <= -0.9999f) {
    vec3_t ortho = sensorGravityVec.cross(vec3_t(1, 0, 0));
    if (ortho.mag() < 1e-3f) 
      ortho = sensorGravityVec.cross(vec3_t(0, 1, 0));
    ortho = ortho.norm();
    gravityRotQuatn = quat_t(0.0f, ortho.x, ortho.y, ortho.z);
    return;
  }

  vec3_t rotAxis = sensorGravityVec.cross(worldGravityVec);
  gravityRotQuatn = quat_t(dotProd+1.0f, rotAxis);
  gravityRotQuatn = gravityRotQuatn.norm();

  accelCorrected = gravityRotQuatn.rotate(accelMs, false);
  accelGravityOffset = accelCorrected.z - GRAVITY_ACCEL;
}

uint64_t lastMeasurement;

void AccelerometerTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(accelIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      lsm6dsm_acceleration_raw_get(&dev_ctx, accel_raw);
      lsm6dsm_angular_rate_raw_get(&dev_ctx, gyro_raw);

      accelMs.x = lsm6dsm_from_fs16g_to_mg(accel_raw[0]) * 1e-3 * GRAVITY_ACCEL;
      accelMs.y = lsm6dsm_from_fs16g_to_mg(accel_raw[1]) * 1e-3 * GRAVITY_ACCEL;
      accelMs.z = lsm6dsm_from_fs16g_to_mg(accel_raw[2]) * 1e-3 * GRAVITY_ACCEL;

      gyroDps.x = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[0]) * 1e-3;
      gyroDps.y = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[1]) * 1e-3;
      gyroDps.z = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[2]) * 1e-3;

      if (flightState == FLIGHT_ARMED && shouldCal) {
        if (samplesCollected < samplesRequired) {
          accelCalibrationSums += accelMs;
          gyroCalibrationSums += gyroDps;
          samplesCollected++;
        }
        // Only apply calibrations if launch has not been detected and will not be detected soon (i.e. < 2g, < 3m/s)
        else if (flightState == FLIGHT_ARMED && accelVertVel < 3 && accelCorrected.z < 5) {
          setGravityRotQuatn();
          accelCalibrationSums = vec3_t(0,0,0);
          accelVertVel = 0;
          
          gyroCalibrationSums /= samplesRequired;
          gyroBiases = gyroCalibrationSums;
          gyroCalibrationSums = vec3_t(0,0,0);
          attitudeQuatn = quat_t(1,0,0,0);

          shouldCal = false;
          samplesCollected = 0;
          initialCalibration = true;

          lastMeasurement = micros();
        }
      }

      // Do not process if accel has not been calibrated
      if (!initialCalibration) continue;
      
      uint64_t now = micros();
      float dt = (now - lastMeasurement) * 1e-6;
      lastMeasurement = now;

      // Accel integration to vertical velocity (does not use gyroscope)
      accelCorrected = gravityRotQuatn.rotate(accelMs, false);
      accelCorrected.z -= accelGravityOffset; // Make sure gravity is as close to 9.8 as possible
      accelCorrected.z -= GRAVITY_ACCEL; // Remove gravity

      accelVertVel += accelCorrected.z * dt;

      // Gyro integration to orientation quaternion
      gyroCorrected = gyroDps - gyroBiases;
      gyroRadsPerSec = gyroCorrected * TO_RADIANS;

      vec3_t deltaAngle = gyroRadsPerSec * dt;
      float angle = deltaAngle.mag();

      if (angle > 0.0f) {
        quat_t deltaQ;
        
        vec3_t axis = deltaAngle / angle;
        deltaQ.setRotation(axis, angle, LARGE_ANGLE); 
    
        attitudeQuatn = attitudeQuatn * deltaQ; 
        attitudeQuatn = attitudeQuatn.norm();
      }

      xEventGroupSetBits(loggingEventGroup, IMU_LOGGING_BIT);

      // Only re-calibrate if launch has not been detected and will not be detected soon (i.e. < 2g, < 3m/s)
      if (calCount >= 4160 && flightState == FLIGHT_ARMED && accelVertVel < 3 && accelCorrected.z < 8) {
        shouldCal = true;
        calCount = 0;
      }

      if (flightState == FLIGHT_ARMED && !shouldCal) calCount++;

      // printf("X:%.2f\tY:%.2f\tZ:%.2f\n", accelMs.x, accelMs.y, accelMs.z);
      // printf("X:%.2f\tY:%.2f\tZ:%.2f\tV:%.2f\n", accelCorrected.x, accelCorrected.y, accelCorrected.z, accelVertVel);
      // printf(">Vvel:%.2f\n", accelVertVel);
      // printf("X:%.2f\tY:%.2f\tZ:%.2f\n", gyroDps.x, gyroDps.y, gyroDps.z);
      // printf("W:%2f\tX:%.2f\tY:%.2f\tZ:%.2f\n", attitudeQuatn.w, attitudeQuatn.v.x, attitudeQuatn.v.y, attitudeQuatn.v.z);
      // printf("Quaternion: %2f,%.2f,%.2f,%.2f\n", attitudeQuatn.w, attitudeQuatn.v.x, attitudeQuatn.v.y, attitudeQuatn.v.z);
      // printf("dt:%.2f\n", dt*1e+3);
    }
  }
}
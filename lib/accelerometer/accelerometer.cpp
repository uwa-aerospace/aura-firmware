#include <Arduino.h>
#include "SPI.h"
// #include "lsm6dsm_reg.h"
#include "lsm6dsox_reg.h"

#include "accelerometer.h"
#include "sdcard.h"
#include "data.h"
#include "vector_type.h"
#include "quaternion_type.h"
#include "SimpleKalmanFilter.h"

#define TAG "ACCELEROMETER"
#define GRAVITY_ACCEL 9.8
#define TO_RADIANS (PI / 180)
#define TO_DEGREES (180 / PI)

uint8_t accelCsPin;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  digitalWrite(accelCsPin, LOW);
  SPI.transfer(reg);
  for (uint16_t i = 0; i < len; i++) {
    SPI.transfer(bufp[i]);
  }
  digitalWrite(accelCsPin, HIGH);

  return 0;
}

int32_t platform_read(void *ctx, uint8_t reg, uint8_t *bufp, uint16_t len) {
  reg |= 0x80;
  digitalWrite(accelCsPin, LOW);
  SPI.transfer(reg);
  for (uint16_t i = 0; i < len; i++) {
    bufp[i] = SPI.transfer(0x00);
  }
  digitalWrite(accelCsPin, HIGH);
  
  return 0;
}

// Reduces any float to 2dp, e.g. 2.04932 -> 2.04
float reduce2DP(float val) {
  return (int)(val * 100.0) / 100.0;
}

SemaphoreHandle_t accelIrqSemaphore;
void IRAM_ATTR accelInterrupt(void) {
  xSemaphoreGive(accelIrqSemaphore);
}

stmdev_ctx_t dev_ctx;

float accMeasErr = 6.76 * 1e-4;
float gyroMeasErr = 0.00601;
float processVar = 0.001;

SimpleKalmanFilter kfAccX(accMeasErr, accMeasErr, processVar);
SimpleKalmanFilter kfAccY(accMeasErr, accMeasErr, processVar);
SimpleKalmanFilter kfAccZ(accMeasErr, accMeasErr, processVar);

SimpleKalmanFilter kfGyroX(gyroMeasErr, gyroMeasErr, processVar);
SimpleKalmanFilter kfGyroY(gyroMeasErr, gyroMeasErr, processVar);
SimpleKalmanFilter kfGyroZ(gyroMeasErr, gyroMeasErr, processVar);

SetupStatus setupAccelerometer(uint8_t cs, uint8_t interrupt) {
  accelCsPin = cs;

  pinMode(accelCsPin, OUTPUT);
  digitalWrite(accelCsPin, HIGH);

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;

  uint8_t whoami = 0;
  lsm6dsox_device_id_get(&dev_ctx, &whoami);

  if (whoami != LSM6DSOX_ID) {
    ESP_LOGE(TAG, "Could not detect accelerometer");
    return ACCEL_ERROR;
  }

  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  vTaskDelay(100);

  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_417Hz);
  
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_16g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);

  lsm6dsox_xl_power_mode_set(&dev_ctx, LSM6DSOX_HIGH_PERFORMANCE_MD);
  lsm6dsox_gy_power_mode_set(&dev_ctx, LSM6DSOX_GY_HIGH_PERFORMANCE);

  lsm6dsox_pin_int2_route_t int2_route;
  lsm6dsox_pin_int2_route_get(&dev_ctx, NULL, &int2_route);
  int2_route.drdy_g = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, NULL, int2_route);

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

int16_t accel_unprocessed[3];
int16_t gyro_unprocessed[3];

vec3_t accelFiltered(0,0,0);
vec3_t gyroFiltered(0,0,0);

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
#define IMU_RECAL_THRESHOLD 2085 // Recalibrate every 5 seconds whilst armed on the pad

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

  accelCorrected = gravityRotQuatn.rotate(accelCalibrationSums, false);
  accelGravityOffset = accelCorrected.z - 1.0f; // Determine sensor offset from gravity
}

uint64_t lastMeasurement;

void AccelerometerTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(accelIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      // Wait until radio finishes using SPI bus
      if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
        lsm6dsox_acceleration_raw_get(&dev_ctx, accel_unprocessed);
        lsm6dsox_angular_rate_raw_get(&dev_ctx, gyro_unprocessed);
        xSemaphoreGive(spiMutex); // Allow radio to use SPI bus
      }

      accelRaw.x = lsm6dsox_from_fs16_to_mg(accel_unprocessed[0]) * 1e-3;
      accelRaw.y = lsm6dsox_from_fs16_to_mg(accel_unprocessed[1]) * 1e-3;
      accelRaw.z = lsm6dsox_from_fs16_to_mg(accel_unprocessed[2]) * 1e-3;

      gyroRaw.x = lsm6dsox_from_fs2000_to_mdps(gyro_unprocessed[0]) * 1e-3;
      gyroRaw.y = lsm6dsox_from_fs2000_to_mdps(gyro_unprocessed[1]) * 1e-3;
      gyroRaw.z = lsm6dsox_from_fs2000_to_mdps(gyro_unprocessed[2]) * 1e-3;

      accelFiltered.x = kfAccX.updateEstimate(accelRaw.x);
      accelFiltered.y = kfAccY.updateEstimate(accelRaw.y);
      accelFiltered.z = kfAccZ.updateEstimate(accelRaw.z);

      gyroFiltered.x = kfGyroX.updateEstimate(gyroRaw.x);
      gyroFiltered.y = kfGyroY.updateEstimate(gyroRaw.y);
      gyroFiltered.z = kfGyroZ.updateEstimate(gyroRaw.z);

      if (flightState == FLIGHT_ARMED && shouldCal) {
        if (samplesCollected < samplesRequired) {
          // If the IMU is moving, do not add samples to calibration average
          if (accelFiltered.mag() > 0.9 && accelFiltered.mag() < 1.1 && gyroFiltered.mag() < 1.0) {
            accelCalibrationSums += accelFiltered;
            gyroCalibrationSums += gyroFiltered;
            samplesCollected++;
          }
        }
        // Only apply calibrations if launch has not been detected and will not be detected soon (i.e. accel < 1.5g)
        else if (flightState == FLIGHT_ARMED && accelFiltered.mag() < CALIBRATION_APPLY_THRESHOLD) {
          accelCalibrationCycle = true;
          
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
        // Prevents deadlock where accel fails on first calibration and cannot re-calibrate unless powercycled
        else if (initialCalibration) shouldCal = false;
      }

      // Do not process if accel has not been calibrated
      if (!initialCalibration) continue;
      
      uint64_t now = micros();
      float dt = (now - lastMeasurement) * 1e-6;
      lastMeasurement = now;

      // Accel integration to vertical velocity (does not use gyroscope)
      accelCorrected = gravityRotQuatn.rotate(accelFiltered, false);
      accelCorrected.z -= accelGravityOffset; // Make sure gravity is as close to 1G as possible
      accelCorrected.z -= 1; // Subtract gravity (1G)
      accelCorrected.z = reduce2DP(accelCorrected.z);
      float accelZ_mSec = accelCorrected.z * GRAVITY_ACCEL;

      accelVertVel += accelZ_mSec * dt;

      if (flightState > FLIGHT_ARMED)
        maxAccelVertVel = max(maxAccelVertVel, accelVertVel);

      // Gyro integration to orientation quaternion
      gyroCorrected = gyroFiltered - gyroBiases;
      gyroCorrected.x = reduce2DP(gyroCorrected.x);
      gyroCorrected.y = reduce2DP(gyroCorrected.y);
      gyroCorrected.z = reduce2DP(gyroCorrected.z);
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

      vec3_t vertical(0,0,1);
      vec3_t rotatedVert = attitudeQuatn.rotate(vertical, false);
      float vertDot = vertical.dot(rotatedVert);
      vertDot = max(-1.0f, min(1.0f, vertDot));
      tiltAngle = acos(vertDot) * TO_DEGREES;

      xEventGroupSetBits(sensorEventGroup, IMU_SENSOR_EVENT);
      xEventGroupSetBits(loggingEventGroup, IMU_SENSOR_EVENT);

      // Only re-calibrate if launch has not been detected and will not be detected soon (i.e. < 1.5g)
      if (calCount >= IMU_RECAL_THRESHOLD && flightState == FLIGHT_ARMED && accelFiltered.mag() < CALIBRATION_APPLY_THRESHOLD) {
        shouldCal = true;
        accelCalibrationSums = vec3_t(0,0,0);
        gyroCalibrationSums = vec3_t(0,0,0);
        samplesCollected = 0;
        calCount = 0;
      }

      if (flightState == FLIGHT_ARMED && !shouldCal) calCount++;

      // printf("X:%.2f\tY:%.2f\tZ:%.2f\n", accelMs.x, accelMs.y, accelMs.z);
      // printf(">X:%.2f\n>Y:%.2f\n>Z:%.2f\n>V:%.2f\n", accelCorrected.x, accelCorrected.y, accelCorrected.z, accelVertVel);
      // printf(">Vvel:%.2f\n", accelVertVel);
      // printf(">X:%.2f\n>Y:%.2f\n>Z:%.2f\n", gyroRaw.x, gyroRaw.y, gyroRaw.z);
      // printf(">X:%.2f\n>Y:%.2f\n>Z:%.2f\n", gyroCorrected.x, gyroCorrected.y, gyroCorrected.z);
      // printf("W:%2f\tX:%.2f\tY:%.2f\tZ:%.2f\n", attitudeQuatn.w, attitudeQuatn.v.x, attitudeQuatn.v.y, attitudeQuatn.v.z);
      // printf("Quaternion: %2f,%.2f,%.2f,%.2f\n", attitudeQuatn.w, attitudeQuatn.v.x, attitudeQuatn.v.y, attitudeQuatn.v.z);
      // printf("Quaternion: %2f,%.2f,%.2f,%.2f,%.2f\n", attitudeQuatn.w, attitudeQuatn.v.x, attitudeQuatn.v.y, attitudeQuatn.v.z, tiltAngle);
      // printf("tiltAngle:%.2f\n", tiltAngle);
      // printf("AccelMag:%.2f\tGyromag:%.2f\n", accelRaw.mag(), gyroRaw.mag());
      // printf(">dt:%.2f\n", dt*1e+3);
    }
  }
}
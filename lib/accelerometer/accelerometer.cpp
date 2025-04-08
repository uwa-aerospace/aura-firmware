#include "accelerometer.h"
#include "lsm6dsm_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Arduino.h>
#include "SPI.h"

#define TAG "ACCELEROMETER"

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
void IRAM_ATTR accelInterrupt() {
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

  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_104Hz);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_104Hz);

  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_16g);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);

  lsm6dsm_int2_route_t int2_route;
  lsm6dsm_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_drdy_xl = PROPERTY_ENABLE;
  lsm6dsm_pin_int2_route_set(&dev_ctx, int2_route);

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

float accel_mg[3];
float gyro_mdps[3];

uint16_t calCount = 0;
uint16_t calThreshold = 104;
bool hasCalibrated = false;
float gyro_bias_sum[3] = {0.0, 0.0, 0.0};
float gyro_zero_bias[3];

void AccelerometerTask(void* pvParameters) {
  while (1) {
    if (xSemaphoreTake(accelIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      lsm6dsm_acceleration_raw_get(&dev_ctx, accel_raw);
      lsm6dsm_angular_rate_raw_get(&dev_ctx, gyro_raw);

      accel_mg[0] = lsm6dsm_from_fs16g_to_mg(accel_raw[0]);
      accel_mg[1] = lsm6dsm_from_fs16g_to_mg(accel_raw[1]);
      accel_mg[2] = lsm6dsm_from_fs16g_to_mg(accel_raw[2]);

      gyro_mdps[0] = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[0]);
      gyro_mdps[1] = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[1]);
      gyro_mdps[2] = lsm6dsm_from_fs2000dps_to_mdps(gyro_raw[2]);

      if (calCount < calThreshold) {
        gyro_bias_sum[0] += gyro_mdps[0];
        gyro_bias_sum[1] += gyro_mdps[1];
        gyro_bias_sum[2] += gyro_mdps[2];

        calCount++;
        continue;
      }

      if (!hasCalibrated) {
        gyro_zero_bias[0] = gyro_bias_sum[0] / calCount;
        gyro_zero_bias[1] = gyro_bias_sum[1] / calCount;
        gyro_zero_bias[2] = gyro_bias_sum[2] / calCount;
        hasCalibrated = true;
      }

      gyro_mdps[0] -= gyro_zero_bias[0];
      gyro_mdps[1] -= gyro_zero_bias[1];
      gyro_mdps[2] -= gyro_zero_bias[2];

      printf("Accel [mg]: X=%.2f Y=%.2f Z=%.2f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
      printf("Gyro [mdps]: X=%.2f Y=%.2f Z=%.2f\n\n", gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
    }
  }
}
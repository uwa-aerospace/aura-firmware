#include <Arduino.h>

// #include <WiFi.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
// #include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Constants
#include "status.h"

// Flight computer peripherals
#include "barometer.h"
#include "sdcard.h"
#include "buzzer.h"
#include "accelerometer.h"
#include "radio.h"

// General defines
#define SETUP_TAG "MAINSETUP"

// Pin and MCU function declarations
#define GNSS_RX_PIN 21
#define GNSS_TX_PIN 18
TaskHandle_t GnssTaskHandle;

#define I2C_SCL 39
#define I2C_SDA 40
TaskHandle_t BarometerTaskHandle;

#define SDIO_CMD 13
#define SDIO_CLK 12
#define SDIO_D0 11
#define SDIO_D1 10
#define SDIO_D2 17
#define SDIO_D3 14
TaskHandle_t LoggingTaskHandle;

#define SPI_SCK 2
#define SPI_MISO 4
#define SPI_MOSI 1

#define ACCEL_CS 0
#define ACCEL_INT 6
TaskHandle_t AccelerometerTaskHandle;

#define RADIO_CS 9
#define RADIO_INT 8
#define RADIO_BUSY 7

#define PYRO1 37
#define PYRO2 35
#define PYRO3 34
#define PYRO4 33

#define BUZZER_PIN 45

#define PROGRAMMABLE_LED 46

void setup()
{
  Serial.begin(460800);

  SetupStatus setupStatus = SETUP_OK;

  esp_wifi_stop();
  esp_wifi_deinit();

  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

  // // Barometer SETUP
  // setupStatus = static_cast<SetupStatus>(setupStatus | setupBarometer(I2C_SDA, I2C_SCL));

  // // SD card SETUP
  setupStatus = static_cast<SetupStatus>(setupStatus | setupSdCard(SDIO_CMD, SDIO_CLK, SDIO_D0, SDIO_D1, SDIO_D2, SDIO_D3));

  // // Accelerometer SETUP
  setupStatus = static_cast<SetupStatus>(setupStatus | setupAccelerometer(SPI_SCK, SPI_MISO, SPI_MOSI, ACCEL_CS, ACCEL_INT));

  if (setupStatus != SETUP_OK)
  {
    ESP_LOGE(SETUP_TAG, "%d", setupStatus);

    // Notify user that setup failed
    shortBeepXTimes(5);

    while (1)
      ; // Do not proceed with execution
  }

  // shortBeepXTimes(1);

  // Peripheral/component tasks
  // xTaskCreate(GnssTask, "GnssTask", 4096, NULL, 2, &GnssTaskHandle);
  // xTaskCreate(BarometerTask, "BarometerTask", 4096, NULL, 2, &BarometerTaskHandle);
  xTaskCreatePinnedToCore(AccelerometerTask, "AccelerometerTask", 32767, NULL, 2, &AccelerometerTaskHandle, 1);
  // xTaskCreate(RadioTask, "RadioTask", 4096, NULL, 2, NULL);

  xTaskCreatePinnedToCore(LoggingTask, "LoggingTask", 65535 * 2, NULL, 3, &LoggingTaskHandle, 1);
}

void loop()
{
}
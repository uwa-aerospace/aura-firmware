#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Constants
#include "status.h"

// Flight computer peripherals
#include "gnss.h"
#include "barometer.h"
#include "sdcard.h"

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

#define PROGRAMMABLE_LED 46

bool ledState = false;
void BlinkTask(void *pvParameters) {
  while (1) {
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void PrintTask(void *pvParameters) {
  while (1) {
    printf("Hello platformio\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  // Debug essentials
  // pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  SetupStatus setupStatus = SETUP_OK;

  // GNSS SETUP
  // Serial2.begin(460800, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  // setupStatus = static_cast<SetupStatus>(setupStatus | setupGNSS(Serial2));

  // Barometer SETUP
  // setupStatus = static_cast<SetupStatus>(setupStatus | setupBarometer(I2C_SDA, I2C_SCL));

  // SD card SETUP
  // setupStatus = static_cast<SetupStatus>(setupStatus | setupSdCard(SDIO_CMD, SDIO_CLK, SDIO_D0, SDIO_D1, SDIO_D2, SDIO_D3));

  if (setupStatus != SETUP_OK) {
    ESP_LOGE(SETUP_TAG, "%d", setupStatus);
    
    // TODO: Properly notify the user by sounding buzzer, blinking LED

    while (1);
  }

  // Peripheral/component tasks
  // xTaskCreate(GnssTask, "GnssTask", 2048, NULL, 1, &GnssTaskHandle);
  // xTaskCreate(BarometerTask, "BarometerTask", 2560, NULL, 1, &BarometerTaskHandle);

  // For alive testing, temporary
  // xTaskCreate(BlinkTask, "BlinkTask", 2048, NULL, 1, NULL);
  // xTaskCreate(PrintTask, "PrintTask", 2048, NULL, 1, NULL);
}

void loop() {

}
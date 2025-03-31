#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Flight computer peripherals
#include "gnss.h"

// Pin and MCU function declarations
#define GNSS_RX_PIN 18
#define GNSS_TX_PIN 21

bool ledState = false;
void BlinkTask(void *pvParameters) {
  digitalWrite(LED_BUILTIN, 0);
  while (1) {
    // digitalWrite(LED_BUILTIN, ledState);
    // ledState = !ledState;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void PrintTask(void *pvParameters) {
  while (1) {
    ESP_LOGI("MAIN", "hello platformio");
    vTaskDelay(pdMS_TO_TICKS(495));
  }
}

void setup() {
  // Debug essentials
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // GNSS SETUP
  Serial2.begin(460800, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  setupGNSS(Serial2);

  // Peripheral/component tasks
  xTaskCreate(GNSSTask, "GNSSTask", 8192, NULL, 1, NULL);

  // For alive testing, temporary
  // xTaskCreate(BlinkTask, "BlinkTask", 2048, NULL, 1, NULL);
  // xTaskCreate(PrintTask, "PrintTask", 2048, NULL, 1, NULL);
}

void loop() {

}
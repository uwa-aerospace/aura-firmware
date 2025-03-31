#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

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
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  xTaskCreate(BlinkTask, "BlinkTask", 2048, NULL, 1, NULL);
  xTaskCreate(PrintTask, "PrintTask", 2048, NULL, 2, NULL);
}

void loop() {

}
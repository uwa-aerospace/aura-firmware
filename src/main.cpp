#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool ledState = false;

void BlinkTask(void *pvParameters) {
  while (1) {
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void PrintTask(void *pvParameters) {
  while (1) {
    Serial.println("PLatformio test");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);
  btStop();

  xTaskCreatePinnedToCore(BlinkTask, "BlinkTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(PrintTask, "PrintTask", 2048, NULL, 2, NULL, 1);
}

void loop() {

}
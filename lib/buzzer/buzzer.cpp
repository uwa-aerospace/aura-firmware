#include "buzzer.h"
#include <Arduino.h>

uint8_t bPin;
QueueHandle_t buzzerQueue;
TaskHandle_t buzzerTaskHandle;

typedef struct {
  uint16_t duration;
  uint8_t repeat;
  uint16_t pause;
} BuzzerCommand;

void buzzerTask(void *pvParameters) {
  BuzzerCommand cmd;

  while (1) {
    if (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY)) {
      for (uint8_t i = 0; i < cmd.repeat; i++) {
        digitalWrite(bPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(cmd.duration));
        digitalWrite(bPin, LOW);

        if (i < cmd.repeat - 1) {
          vTaskDelay(pdMS_TO_TICKS(cmd.pause));
        }
      }
    }
  }
}

void setupBuzzer(uint8_t pin) {
  bPin = pin;
  pinMode(bPin, OUTPUT);
  digitalWrite(bPin, LOW);

  buzzerQueue = xQueueCreate(5, sizeof(BuzzerCommand));
  xTaskCreate(buzzerTask, "BuzzerTask", 2048, NULL, 2, &buzzerTaskHandle);
}

void shortBeep() {
  BuzzerCommand cmd = {75, 1, 0};
  xQueueSend(buzzerQueue, &cmd, 0);
}

void longBeep() {
  BuzzerCommand cmd = {500, 1, 0};
  xQueueSend(buzzerQueue, &cmd, 0);
}

void shortBeepXTimes(uint8_t times) {
  BuzzerCommand cmd = {75, times, 200};
  xQueueSend(buzzerQueue, &cmd, 0);
}

void longBeepXTimes(uint8_t times) {
  BuzzerCommand cmd = {500, times, 400};
  xQueueSend(buzzerQueue, &cmd, 0);
}
#include "buzzer.h"
#include <Arduino.h>

uint8_t bPin;

void shortBeep() {
  digitalWrite(bPin, 1);
  
  yield();
  vTaskDelay(75);

  digitalWrite(bPin, 0);
}

void longBeep() {
  digitalWrite(bPin, 1);

  yield();
  vTaskDelay(500);

  digitalWrite(bPin, 0);
}

void shortBeepXTimes(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    shortBeep();

    yield();
    vTaskDelay(200);
  }
}

void setupBuzzer(uint8_t buzzerPin) {
  delay(20);
  bPin = buzzerPin;
  pinMode(buzzerPin, OUTPUT);
}
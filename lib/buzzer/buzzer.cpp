#include "buzzer.h"
#include <Arduino.h>

uint8_t bPin;

void shortBeep() {
  digitalWrite(bPin, 1);
  
  uint32_t start = millis();
  while (millis() - start < 75) {
    yield();
    delay(5);
  }

  digitalWrite(bPin, 0);
}

void longBeep() {
  digitalWrite(bPin, 1);

  uint32_t start = millis();
  while (millis() - start < 500) {
    yield();
    delay(20);
  }

  digitalWrite(bPin, 0);
}

void shortBeepXTimes(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    shortBeep();

    uint32_t start = millis();
    while (millis() - start < 200) {
      yield();
      delay(20);
    }
  }
}

void setupBuzzer(uint8_t buzzerPin) {
  delay(20);
  bPin = buzzerPin;
  pinMode(buzzerPin, OUTPUT);
}
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void shortBeep();
void longBeep();
void shortBeepXTimes(uint8_t times);
void setupBuzzer(uint8_t buzzerPin);

#endif
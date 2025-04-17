#include "pyro.h"
#include <Arduino.h>

uint8_t channels[4];
hw_timer_t* timers[4];
volatile bool active[4] = {false};

#define FIRE_DURATION_US 150000
#define TIMER_PRESCALER 80

void IRAM_ATTR onTimer0() { digitalWrite(channels[0], 0); active[0] = false; }
void IRAM_ATTR onTimer1() { digitalWrite(channels[1], 0); active[1] = false; }
void IRAM_ATTR onTimer2() { digitalWrite(channels[2], 0); active[2] = false; }
void IRAM_ATTR onTimer3() { digitalWrite(channels[3], 0); active[3] = false; }

void setupPyros(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4) {
  channels[0] = c1;
  channels[1] = c2;
  channels[2] = c3;
  channels[3] = c4;

  for (int i = 0; i < 4; i++) {
    pinMode(channels[i], OUTPUT);
    digitalWrite(channels[i], 0);
  }

  timers[0] = timerBegin(0, TIMER_PRESCALER, true);
  timers[1] = timerBegin(1, TIMER_PRESCALER, true);
  timers[2] = timerBegin(2, TIMER_PRESCALER, true);
  timers[3] = timerBegin(3, TIMER_PRESCALER, true);

  timerAttachInterrupt(timers[0], &onTimer0, true);
  timerAttachInterrupt(timers[1], &onTimer1, true);
  timerAttachInterrupt(timers[2], &onTimer2, true);
  timerAttachInterrupt(timers[3], &onTimer3, true);
}

// 0 INDEXED
void firePyro(uint8_t channel) {
  if (channel > 3 || active[channel]) return;

  digitalWrite(channels[channel], 1);
  active[channel] = true;

  timerRestart(timers[channel]);
  timerAlarmWrite(timers[channel], FIRE_DURATION_US, false);
  timerAlarmEnable(timers[channel]);
}

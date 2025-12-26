#include <Arduino.h>

uint8_t channels[4];
volatile uint16_t pyroTimers[4] = {0};
volatile bool active[4] = {false};
hw_timer_t* pyroTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define FIRE_DURATION_MS 150
#define TIMER_PRESCALER 80

void IRAM_ATTR onPyroTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  for (int i = 0; i < 4; i++) {
    if (active[i] && pyroTimers[i] > 0) {
      pyroTimers[i]--;
      if (pyroTimers[i] == 0) {
        digitalWrite(channels[i], 0);
        active[i] = false;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setupPyros(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4) {
  channels[0] = c1;
  channels[1] = c2;
  channels[2] = c3;
  channels[3] = c4;

  for (int i = 0; i < 4; i++) {
    pinMode(channels[i], OUTPUT);
    digitalWrite(channels[i], 0);
  }

  pyroTimer = timerBegin(0, TIMER_PRESCALER, true);
  timerAttachInterrupt(pyroTimer, &onPyroTimer, true);
  timerAlarmWrite(pyroTimer, 1000, true);
  timerAlarmEnable(pyroTimer);
}

void firePyro(uint8_t channel) {
  if (channel > 3) return;

  portENTER_CRITICAL(&timerMux);
  if (!active[channel]) {
    digitalWrite(channels[channel], 1);
    pyroTimers[channel] = FIRE_DURATION_MS;
    active[channel] = true;
  }
  portEXIT_CRITICAL(&timerMux);
}

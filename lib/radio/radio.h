#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define RADIO_IDLE_TX_RATE 1000
#define RADIO_ARMED_TX_RATE 500
#define RADIO_FLIGHT_TX_RATE 67

extern TimerHandle_t radioTransmitTimer;
SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy, long freqHz);
void RadioTask(void *pvParameters);

#endif
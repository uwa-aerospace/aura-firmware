#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define ARM_CMD "ARM8MkEewq7"
#define DISARM_CMD "DISARM8MkEewq7"
#define FIRE_PYRO_CMD "FIRE8MkEewq7"
#define RADIO_FREQ_CMD "FREQ8MkEewq7"
#define CAM_CTL_CMD "CAMCTL8MkEewq7"

SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy, long freqHz);
void RadioTask(void *pvParameters);

#endif
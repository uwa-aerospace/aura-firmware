#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "status.h"

SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy);
void RadioTask(void *pvParameters);

#endif
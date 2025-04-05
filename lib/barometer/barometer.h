#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>
#include "errors.h"

SetupStatus setupBarometer(uint8_t sdaPin, uint8_t sclPin);
void BarometerTask(void *pvParameters);

#endif
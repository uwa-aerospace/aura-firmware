#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>
#include "status.h"

SetupStatus setupAccelerometer(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t interrupt);
void AccelerometerTask(void* pvParameters);

#endif
#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "status.h"

bool writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);
SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

#endif
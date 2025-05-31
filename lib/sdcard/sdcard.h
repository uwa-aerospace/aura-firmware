#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "status.h"
#include "FS.h"

extern bool resetLogTimeAtLaunch;

SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void LoggingTask(void* pvParameters);
void flushLogFile();

#endif
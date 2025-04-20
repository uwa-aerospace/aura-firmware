#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "status.h"
#include "FS.h"

#define IMU_LOGGING_BIT  (1 << 0)
#define BARO_LOGGING_BIT (1 << 1)
#define GNSS_LOGGING_BIT (1 << 2)

extern EventGroupHandle_t loggingEventGroup;

bool writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);
SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void LoggingTask(void* pvParameters);

#endif
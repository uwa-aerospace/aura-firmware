#ifndef GNSS_H
#define GNSS_H

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

extern SFE_UBLOX_GNSS neo;

void setupGNSS(HardwareSerial &serialPort);
void GNSSTask(void *pvParameters);
void processPvtData(UBX_NAV_PVT_data_t *ubxData);

#endif
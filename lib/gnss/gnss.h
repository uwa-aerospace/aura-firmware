#ifndef GNSS_H
#define GNSS_H

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

void setupGNSS(HardwareSerial &serialPort);
void gnssInterrupt();
void GnssTask(void *pvParameters);

#endif
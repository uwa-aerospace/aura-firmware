#ifndef GNSS_H
#define GNSS_H

#include <Arduino.h>
#include "status.h"

SetupStatus setupGNSS(HardwareSerial &serialPort);
void gnssInterrupt();
void GnssTask(void *pvParameters);

#endif
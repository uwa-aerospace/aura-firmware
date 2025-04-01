#ifndef GNSS_H
#define GNSS_H

#include <Arduino.h>

void setupGNSS(HardwareSerial &serialPort);
void gnssInterrupt();
void GnssTask(void *pvParameters);

#endif
#include "gnss.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "data.h"
#include "vector_type.h"

#define TAG "GNSS"

SFE_UBLOX_GNSS neo;
SemaphoreHandle_t gnssIrqSemaphore;

volatile bool gnssTaskStarted = false;
void IRAM_ATTR gnssInterrupt(void) {
  // Clear UART FIFO until GnssTask starts, prevents FIFO overflow
  if (!gnssTaskStarted) {
    uart_flush_input(UART_NUM_2);
  }
  xSemaphoreGive(gnssIrqSemaphore);
}

void IRAM_ATTR gnssRxErrorCallback(hardwareSerial_error_t error) {
  // In case GnssTask gets blocked after startup and FIFO overflows
  if (error == UART_FIFO_OVF_ERROR || error == UART_BUFFER_FULL_ERROR)
    uart_flush_input(UART_NUM_2);
}

SetupStatus setupGNSS(HardwareSerial &serialPort) {
  if (!neo.begin(serialPort)) {
    ESP_LOGE(TAG, "Could not connect to GNSS through UART");
    return GNSS_ERROR;
  }

  /* No setup required, GNSS settings are saved to BBR and flash, and persist between startups */
  /* Setup reqs:
   * 460800 baudrate
   * UBX transmission
   * 40ms MEASurement rate, 1 Navigation rate - achieves 25Hz ODR
   * 4 constellation tracking (GPS, GLONASS, Galileo, BeiDou)
   * AIRBORNE_4g dynamic model - CFG-NAV5
  */

  /* Cannot validate constellations tracked, all settings not checked below are self-validated */
  uint16_t measurementRate = neo.getMeasurementRate();
  if (measurementRate != 40) {
    ESP_LOGE(TAG, "GNSS update rate is not 25Hz");
    return GNSS_ERROR;
  }

  uint8_t dynamicModel = neo.getDynamicModel();
  if (dynamicModel != DYN_MODEL_AIRBORNE4g) {
    ESP_LOGE(TAG, "Incorrect dynamic model");
    return GNSS_ERROR;
  }
  
  gnssIrqSemaphore = xSemaphoreCreateBinary();
  if (gnssIrqSemaphore == NULL) {
    ESP_LOGE(TAG, "Could not initialize GNSS semaphore");
    return GNSS_ERROR;
  }

  serialPort.onReceive(gnssInterrupt);
  serialPort.onReceiveError(gnssRxErrorCallback);

  ESP_LOGI(TAG, "GNSS module setup successful");

  return SETUP_OK;
}

bool shouldCal = true;
float gnssPadAltitudeSum = 0;
uint32_t calCount = 0;
#define GNSS_RECAL_THRESHOLD 250 // Recalibrate once every 10 seconds whilst armed on the pad

uint16_t samplesRequired = 50;
uint16_t samplesCollected = 0;

void GnssTask(void *pvParameters) {
  // Task is ready to receive data, tell RX interrupt to stop flushing FIFO
  gnssTaskStarted = true;

  while (1) {
    if (xSemaphoreTake(gnssIrqSemaphore, portMAX_DELAY) == pdTRUE) {
      // Allows processing of multiple messages at once
      while (neo.checkUblox()) {
        if (neo.getPVT()) {
          UBX_NAV_PVT_data_t pvt = neo.packetUBXNAVPVT->data;

          gnssHasFix = pvt.fixType == 3; // Must have a 3D fix
          if (!gnssHasFix) continue;
          
          gnssLatitude = pvt.lat / 1e-7;
          gnssLongitude = pvt.lon / 1e-7;
          
          // Altitude and vert vel are measured in mm (mm/s)
          gnssAltitudeMSL = pvt.hMSL / 1000.0;
          gnssVertVel = pvt.velD / 1000.0;

          float gnssPdop = pvt.pDOP / 100.0;
          gnssValidReadings = (gnssPdop < 3);

          if (flightState == FLIGHT_ARMED && shouldCal) {
            if (samplesCollected < samplesRequired && gnssValidReadings) {
              gnssPadAltitudeSum += gnssAltitudeMSL;
              samplesCollected++;
            }
            // Only apply calibrations if launch has not been detected and will not be detected soon (i.e. < 2g, < 3m/s)
            else if (flightState == FLIGHT_ARMED && accelVertVel < 3 && accelCorrected.z < 5) {
              gnssPadAltitude = gnssPadAltitudeSum / samplesRequired;
              shouldCal = false;
              samplesCollected = 0;
            }
          }

          gnssAltitudeAGL = gnssAltitudeMSL - gnssPadAltitude;

          // Only re-calibrate if launch has not been detected and will not be detected soon (i.e. < 2g, < 3m/s)
          if (calCount >= GNSS_RECAL_THRESHOLD && flightState == FLIGHT_ARMED && accelVertVel < 3 && accelCorrected.z < 8) {
            shouldCal = true;
            calCount = 0;
          }

          if (flightState == FLIGHT_ARMED && !shouldCal) calCount++;

          // printf(">MSL:%.2f\n>AGL:%.2f\n>VV:%.2f\n", gnssAltitudeMSL, gnssAltitudeAGL, gnssVertVel);
        }
      }
    }
  }
}
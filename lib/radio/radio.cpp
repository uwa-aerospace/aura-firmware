#include "radio.h"
#include "LoraSx1262.h"

#include "data.h"
#include "vector_type.h"

#define TAG "RADIO"

#define RADIO_IDLE_TX_RATE 1000
#define RADIO_ARMED_TX_RATE 500
#define RADIO_FLIGHT_TX_RATE 100
#define TRANSMIT_BIT (1 << 0)
#define RECEIVE_BIT  (1 << 1)

LoraSx1262* radio;

EventGroupHandle_t radioEventGroup;
TimerHandle_t radioTransmitTimer;
volatile bool shouldTransmit = false;

void transmitTimerCallback(TimerHandle_t xTimer) {
  shouldTransmit = true;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, TRANSMIT_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR radioInterrupt(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, RECEIVE_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

long freqHz = 0;

SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy, long freqHz) {
  radio = new LoraSx1262(sck, miso, mosi, cs, -1, dio1, busy);

  if (!radio->begin()) {
    ESP_LOGE(TAG, "Failed to initialize radio");
    return RADIO_ERROR;
  }

  radio->configSetPreset(PRESET_DEFAULT);
  radio->configSetFrequency(freqHz);
  radio->setModeReceive();
  attachInterrupt(digitalPinToInterrupt(dio1), radioInterrupt, HIGH);

  radioEventGroup = xEventGroupCreate();
  if (radioEventGroup == NULL) {
    ESP_LOGE(TAG, "Could not initialize radio event group");
    return RADIO_ERROR;
  }

  radioTransmitTimer = xTimerCreate("RadioTransmitTimer", pdMS_TO_TICKS(RADIO_FLIGHT_TX_RATE), pdTRUE, (void*)0, transmitTimerCallback);
  if (radioTransmitTimer == NULL) {
    ESP_LOGE(TAG, "Could not create radio transmit timer");
    return RADIO_ERROR;
  }
  xTimerStart(radioTransmitTimer, 0);

  ESP_LOGI(TAG, "Radio setup successful");

  return SETUP_OK;
}

char receiveBuff[255];

char transmitBuf[255];
int txBufStrPos;
const uint8_t base = 10;
const char cs = ',';

void writeIntDataRadioStr(int dataValue) {
  itoa(dataValue, transmitBuf + txBufStrPos, base);
  while(transmitBuf[txBufStrPos]!= '\0'){txBufStrPos++;}
  transmitBuf[txBufStrPos] = cs;
  txBufStrPos++;
}

void writeFloatDataRadioStr(float dataValue, byte decimals){
  long fracInt;
  float partial;

  //sign portion
  if (dataValue < 0) {
    transmitBuf[txBufStrPos] = '-'; 
    txBufStrPos++;
    dataValue *= -1;
  }
  
  //integer portion
  itoa((int)dataValue, transmitBuf + txBufStrPos, base);
  while(transmitBuf[txBufStrPos]!= '\0'){ txBufStrPos++; }
  transmitBuf[txBufStrPos]='.'; txBufStrPos++;
  
  //fractional portion
  partial = dataValue - (int)(dataValue);
  fracInt = (long)(partial*powf(10,decimals));
  if (fracInt == 0) {
    transmitBuf[txBufStrPos] = '0'; 
    txBufStrPos++; 
    transmitBuf[txBufStrPos] = cs; 
    txBufStrPos++;
  }
  else {
    decimals--;
    while(fracInt < powf(10, decimals)){ 
      transmitBuf[txBufStrPos]='0';
      txBufStrPos++;
      decimals--; 
    }
    ltoa(fracInt, transmitBuf + txBufStrPos, base);
    while(transmitBuf[txBufStrPos]!= '\0'){ txBufStrPos++; }
    transmitBuf[txBufStrPos]=','; txBufStrPos++;
  }
}

// RADIO TO SEND
// 500 MS (2 HZ) ON THE GROUND: flightState, baroAlt, gnssAlt, accelVel, gnssVel, baroVel, tiltAng, lat, lon, accelXYZ, gyroXYZ
// 100 MS (10HZ) IN THE AIR: flightState, baroAlt, gnssAlt, accelVel, gnssVel, baroVel, tiltAng, lat, lon

void RadioTask(void *pvParameters) {
  while (1) {
    EventBits_t bits = xEventGroupWaitBits(
      radioEventGroup,
      TRANSMIT_BIT | RECEIVE_BIT,
      pdTRUE,
      pdFALSE,
      portMAX_DELAY
    );

    if (bits & TRANSMIT_BIT) {
      // Write all data (except lat/lon) as ints to save payload space
      writeIntDataRadioStr(flightState);
      
      writeIntDataRadioStr((int) baroAltitudeAGL);
      writeIntDataRadioStr((int) gnssAltitudeAGL);
      
      writeIntDataRadioStr((int) accelVertVel);
      writeIntDataRadioStr((int) baroVertVel);
      writeIntDataRadioStr((int) gnssVertVel);
      writeIntDataRadioStr((int) tiltAngle);
      
      // 5 dp lat/lon is sufficient precision
      writeFloatDataRadioStr(gnssLatitude, 5);
      writeFloatDataRadioStr(gnssLongitude, 5);

      // Send accel and gyro XYZ values when not launched for debugging/validation
      if (flightState == FLIGHT_ARMED) {
        writeFloatDataRadioStr(accelCorrected.x, 2);
        writeFloatDataRadioStr(accelCorrected.y, 2);
        writeFloatDataRadioStr(accelCorrected.z, 2);
        writeFloatDataRadioStr(gyroCorrected.x, 2);
        writeFloatDataRadioStr(gyroCorrected.y, 2);
        writeFloatDataRadioStr(gyroCorrected.z, 2);
      }
      txBufStrPos--;
      transmitBuf[txBufStrPos] = '\0';
      
      radio->transmit(transmitBuf, strlen(transmitBuf));
      radio->setModeReceive(); // Allows receiving messages between transmits
      ESP_LOGI(TAG,"TRANSMITED");
      txBufStrPos = 0;
    }

    if (bits & RECEIVE_BIT) {
      int bytesRead = radio->lora_receive_async(receiveBuff, sizeof(receiveBuff));
      if (bytesRead > -1) {
        printf("Received: %s\n", receiveBuff);
        printf("RSSI: %d\n", radio->rssi);
        printf("SNR: %d\n", radio->snr);
        printf("Signal RSSI: %d\n\n", radio->rssi);
      }
    }
  }
}
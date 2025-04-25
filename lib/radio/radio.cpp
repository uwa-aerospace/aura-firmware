#include "radio.h"
#include "SX126x-Arduino.h"

#include "data.h"
#include "vector_type.h"

#define TAG "RADIO"

#define RADIO_IDLE_TX_RATE 1000
#define RADIO_ARMED_TX_RATE 500
#define RADIO_FLIGHT_TX_RATE 100
#define TRANSMIT_BIT (1 << 0)
#define RECEIVE_BIT  (1 << 1)

#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 1		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 12  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

hw_config hwConfig;
EventGroupHandle_t radioEventGroup;
SemaphoreHandle_t txDoneSemaphore;
TimerHandle_t radioTransmitTimer;

void transmitTimerCallback(TimerHandle_t xTimer) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, TRANSMIT_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void OnTxDone(void);
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr);

long freq = 0;

SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy, long freqHz) {
  hwConfig.CHIP_TYPE = SX1262_CHIP;
  hwConfig.PIN_LORA_RESET = -1;
  hwConfig.PIN_LORA_NSS = cs;
  hwConfig.PIN_LORA_SCLK = sck;
  hwConfig.PIN_LORA_MISO = miso;
  hwConfig.PIN_LORA_DIO_1 = dio1;
  hwConfig.PIN_LORA_BUSY = busy;
  hwConfig.PIN_LORA_MOSI = mosi;
  hwConfig.USE_DIO2_ANT_SWITCH = true;
  hwConfig.USE_DIO3_TCXO = false;
  hwConfig.USE_LDO = true;
  freq = freqHz;

  if (lora_hardware_init(hwConfig) != 0) {
    ESP_LOGE(TAG, "Board init failed");
    return RADIO_ERROR;
  }

  radioEventGroup = xEventGroupCreate();
  txDoneSemaphore = xSemaphoreCreateBinary();
  radioTransmitTimer = xTimerCreate(
    "RadioTxTimer",
    pdMS_TO_TICKS(RADIO_FLIGHT_TX_RATE),
    pdTRUE, NULL,
    transmitTimerCallback
  );
  xTimerStart(radioTransmitTimer, 0);

  // 4. Register callbacks and initialize radio
  static RadioEvents_t events;
  events.TxDone = OnTxDone;
  events.RxDone = OnRxDone;
  events.TxTimeout = nullptr;
  events.RxTimeout = nullptr;
  events.RxError   = nullptr;
  events.CadDone   = nullptr;

  Radio.Init(&events);
  Radio.SetChannel(freq);
  Radio.SetTxConfig(
    MODEM_LORA, 
    TX_OUTPUT_POWER, 
    0, 
    LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, 
    LORA_CODINGRATE,
    LORA_PREAMBLE_LENGTH, 
    LORA_FIX_LENGTH_PAYLOAD_ON,
    true, 
    0, 
    0, 
    LORA_IQ_INVERSION_ON, 
    TX_TIMEOUT_VALUE
  );
  Radio.SetRxConfig(
    MODEM_LORA, 
    LORA_BANDWIDTH, 
    LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 
    0,
    LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT, 
    LORA_FIX_LENGTH_PAYLOAD_ON,
    0, 
    true, 
    0, 
    0, 
    LORA_IQ_INVERSION_ON, 
    true
  );
  Radio.RxBoosted(0);

  ESP_LOGI(TAG, "Radio setup successful");

  return SETUP_OK;
}

static char receiveBuff[255];
static int16_t lastRssi;
static int8_t  lastSnr;
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(receiveBuff, payload, size);
  receiveBuff[size] = '\0';
  lastRssi = rssi;
  lastSnr  = snr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, RECEIVE_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

char transmitBuf[255];
int txBufStrPos;
const uint8_t base = 10;
const char cs = ',';
void OnTxDone(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(txDoneSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
void buildDataString() {
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
}

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
      buildDataString();

      Radio.Send((uint8_t*) transmitBuf, strlen(transmitBuf));

      if (xSemaphoreTake(txDoneSemaphore, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "TX");
        Radio.RxBoosted(0);
        txBufStrPos = 0;
      }
    }

    if (bits & RECEIVE_BIT) {
      ESP_LOGI(TAG, "RX: %s (RSSI=%d, SNR=%d)",
               receiveBuff, lastRssi, lastSnr);
      
      Radio.RxBoosted(0);
    }
  }
}
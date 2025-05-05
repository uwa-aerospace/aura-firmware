#include "radio.h"
#include "SX126x-Arduino.h"
#include "pyro.h"
#include "buzzer.h"

#include "data.h"
#include "vector_type.h"

#define TAG "RADIO"

#define TRANSMIT_BIT (1 << 0)
#define RECEIVE_BIT  (1 << 1)

#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 1		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 12  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD false
#define LORA_IQ_INVERSION false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

#define ARM_CMD "ARM8MkEewq7"
#define DISARM_CMD "DISARM8MkEewq7"
#define FIRE_PYRO_CMD "FIRE8MkEewq7"

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
    ESP_LOGE(TAG, "Radio init failed");
    return RADIO_ERROR;
  }

  radioEventGroup = xEventGroupCreate();
  txDoneSemaphore = xSemaphoreCreateBinary();
  radioTransmitTimer = xTimerCreate(
    "RadioTxTimer",
    pdMS_TO_TICKS(RADIO_IDLE_TX_RATE),
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
    LORA_FIX_LENGTH_PAYLOAD,
    true, 
    0, 
    0, 
    LORA_IQ_INVERSION, 
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
    LORA_FIX_LENGTH_PAYLOAD,
    0, 
    true, 
    0, 
    0, 
    LORA_IQ_INVERSION, 
    true
  );
  Radio.RxBoosted(0);

  ESP_LOGI(TAG, "Radio setup successful");

  return SETUP_OK;
}

static char receiveBuff[255];
static int16_t lastRssi;
static int8_t lastSnr;
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(receiveBuff, payload, size);
  receiveBuff[size] = '\0';
  lastRssi = rssi;
  lastSnr = snr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, RECEIVE_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t transmitBuf[127];
uint8_t txBufIndex = 0;
void OnTxDone(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(txDoneSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void writeInt16(int16_t value) {
  transmitBuf[txBufIndex++] = value >> 8;
  transmitBuf[txBufIndex++] = value & 0xFF;
}

void writeInt32(int32_t value) {
  transmitBuf[txBufIndex++] = (value >> 24) & 0xFF;
  transmitBuf[txBufIndex++] = (value >> 16) & 0xFF;
  transmitBuf[txBufIndex++] = (value >> 8) & 0xFF;
  transmitBuf[txBufIndex++] = value & 0xFF;
}

// RADIO TO SEND
// 500 MS (2 HZ) ON THE GROUND: flightState, baroAlt, gnssAlt, accelVel, gnssVel, baroVel, tiltAng, lat, lon, accelXYZ, gyroXYZ
// 66.67 MS (15Hz) IN THE AIR: flightState, baroAlt, gnssAlt, accelVel, gnssVel, baroVel, tiltAng, lat, lon, accelXYZ, gyroXYZ
void buildDataString() {
  // Write all data (except lat/lon) as ints to save payload space
  transmitBuf[txBufIndex++] = flightState;
        
  writeInt16((int16_t) baroAltitudeAGL);
  writeInt16((int16_t) gnssAltitudeAGL);

  writeInt32((int32_t)(accelVertVel * 100));
  writeInt32((int32_t)(baroVertVel * 100));
  writeInt32((int32_t)(gnssVertVel * 100));
  writeInt16((int16_t)(tiltAngle * 100));

  writeInt32((int32_t)(gnssLatitude * 1e5));
  writeInt32((int32_t)(gnssLongitude * 1e5));

  writeInt16((int16_t)(accelCorrected.x * 100));
  writeInt16((int16_t)(accelCorrected.y * 100));
  writeInt16((int16_t)(accelCorrected.z * 100));
  writeInt16((int16_t)(gyroCorrected.x * 10));
  writeInt16((int16_t)(gyroCorrected.y * 10));
  writeInt16((int16_t)(gyroCorrected.z * 10));
}

void processRadioCommands(char* command, int data) {
  if (flightState > FLIGHT_ARMED) return; // No more commands accepted after launch detected

  if (strcmp(command, ARM_CMD) == 0) {
    shortBeepXTimes(3);
    delay(1000);
    xTimerChangePeriod(radioTransmitTimer, pdMS_TO_TICKS(RADIO_ARMED_TX_RATE), 0);
    flightState = FLIGHT_ARMED;
  }
  else if (strcmp(command, DISARM_CMD) == 0) {
    shortBeepXTimes(3);
    xTimerChangePeriod(radioTransmitTimer, pdMS_TO_TICKS(RADIO_IDLE_TX_RATE), 0);
    flightState = FLIGHT_IDLE;
  }
  // Do not fire pyros when system is ARMED, could cause launch detection
  else if (strcmp(command, FIRE_PYRO_CMD) == 0 && data >= 0 && data <= 3 && flightState == FLIGHT_IDLE) {
    longBeep();
    delay(1000);
    firePyro(data);
  }
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

      Radio.Send(transmitBuf, txBufIndex);

      if (xSemaphoreTake(txDoneSemaphore, portMAX_DELAY) == pdTRUE) {
        Radio.RxBoosted(0);
        txBufIndex = 0;
      }
    }

    if (bits & RECEIVE_BIT) {
      char command[64] = {0};
      int number = -1;
      sscanf(receiveBuff, "%64[^,],%d", command, &number);
      processRadioCommands(command, number);
      
      Radio.RxBoosted(0);
    }
  }
}
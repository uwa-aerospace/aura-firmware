#include <BLEDevice.h>
#include "ble.h"
#include <Preferences.h>
#include "prefs.h"
#include "radio.h"
#include "SX126x-Arduino.h"

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

hw_config hwConfig;
SemaphoreHandle_t rxDoneSemaphore;
SemaphoreHandle_t txDoneSemaphore;
TimerHandle_t radioTransmitTimer;

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
  freq = freqHz * 1e5;

  if (lora_hardware_init(hwConfig) != 0) {
    ESP_LOGE(TAG, "Radio init failed");
    return RADIO_ERROR;
  }

  rxDoneSemaphore = xSemaphoreCreateBinary();
  txDoneSemaphore = xSemaphoreCreateBinary();

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
static unsigned long lastReceive = 0;

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(receiveBuff, payload, size);
  receiveBuff[size] = '\0';
  lastRssi = rssi;
  lastSnr = snr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(rxDoneSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void OnTxDone(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(txDoneSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void RadioTask(void *pvParameters) {
  while (1) {
    xSemaphoreTake(rxDoneSemaphore, portMAX_DELAY);
    // TODO
    unsigned long now = millis();
    int delta = now - lastReceive;
    lastReceive = now;

    char telemetryStr[256];
    snprintf(telemetryStr, sizeof(telemetryStr), "%s,%d,%d,%d,%d\n", receiveBuff, lastRssi, lastSnr, lastRssi, delta);
    pTelemetryChar->setValue(telemetryStr);
    pTelemetryChar->notify();

    char command[50];
    if (xQueueReceive(commandQueue, (void *)command, 0) == pdTRUE) {
      Radio.Send((uint8_t *) command, strlen(command));

      if (xSemaphoreTake(txDoneSemaphore, portMAX_DELAY) == pdTRUE) {
        Radio.RxBoosted(0);
      }
    }
  }
}
#include "radio.h"
#include "LoraSx1262.h"

#define TAG "RADIO"

#define RADIO_SLOW_TX_RATE 1000
#define RADIO_FAST_TX_RATE 100
#define TRANSMIT_BIT (1 << 0)
#define RECEIVE_BIT  (1 << 1)

LoraSx1262* radio;

EventGroupHandle_t radioEventGroup;
TimerHandle_t radioTransmitTimer;
volatile bool shouldTransmit = false;

void transmitTimerCallback(TimerHandle_t xTimer) {
  shouldTransmit = true;
  xEventGroupSetBits(radioEventGroup, TRANSMIT_BIT);
}

void IRAM_ATTR radioInterrupt(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(radioEventGroup, RECEIVE_BIT, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

SetupStatus setupRadio(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t dio1, uint8_t busy) {
  radio = new LoraSx1262(sck, miso, mosi, cs, -1, dio1);

  if (!radio->begin()) {
    ESP_LOGE(TAG, "Failed to initialize radio");
    return RADIO_ERROR;
  }

  radio->configSetPreset(PRESET_DEFAULT);
  radio->setModeReceive();
  attachInterrupt(digitalPinToInterrupt(dio1), radioInterrupt, HIGH);

  radioEventGroup = xEventGroupCreate();
  if (radioEventGroup == NULL) {
    ESP_LOGE(TAG, "Could not initialize radio event group");
    return RADIO_ERROR;
  }

  radioTransmitTimer = xTimerCreate("RadioTransmitTimer", pdMS_TO_TICKS(RADIO_SLOW_TX_RATE), pdTRUE, (void*)0, transmitTimerCallback);
  if (radioTransmitTimer == NULL) {
    ESP_LOGE(TAG, "Could not create radio transmit timer");
    return RADIO_ERROR;
  }
  xTimerStart(radioTransmitTimer, 0);

  ESP_LOGI(TAG, "Radio setup successful");

  return SETUP_OK;
}

char receiveBuff[255];

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
      ESP_LOGI(TAG, "Transmitting...");
      radio->transmit((char*)"Hello from aura", strlen((char*)"Hello from aura"));
      ESP_LOGI(TAG, "Done!\n\n");
      radio->setModeReceive(); // Allows receiving messages between transmits
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
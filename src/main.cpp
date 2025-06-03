#include <Arduino.h>
#include "SPI.h"
#include <Preferences.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Constants
#include "status.h"
#include "data.h"

// Flight computer peripherals
#include "ble.h"
#include "radio.h"
#include "buzzer.h"

// General defines
#define SETUP_TAG "MAINSETUP"

// Pin and MCU function declarations
#define SPI_SCK 2
#define SPI_MISO 4
#define SPI_MOSI 1

#define RADIO_CS 9
#define RADIO_INT 8
#define RADIO_BUSY 7

#define BUZZER_PIN 45
#define PROGRAMMABLE_LED 46
#define MEM_CS 3
#define ACCEL_CS 0

SemaphoreHandle_t spiMutex;
Preferences prefs;

void setup() {
  Serial.begin(460800);

  prefs.begin("aura-settings", false);
  int radioFreq = prefs.getInt("radioFreq", 9190);
  
  SetupStatus setupStatus = SETUP_OK;

  // Reset all SPI CS pins to prevent multi-access
  pinMode(RADIO_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  pinMode(MEM_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(MEM_CS, HIGH);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(10000000);

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex != NULL) {
    setupStatus = static_cast<SetupStatus>(setupStatus | setupRadio(SPI_SCK, SPI_MISO, SPI_MOSI, RADIO_CS, RADIO_INT, RADIO_BUSY, radioFreq));
  }
  else {
    setupStatus = static_cast<SetupStatus>(setupStatus | RADIO_ERROR | ACCEL_ERROR);
  }

  // No status checks required because they will generally always succeed
  setupBuzzer(BUZZER_PIN);
  setupBLE();

  if (setupStatus != SETUP_OK) {
    ESP_LOGE(SETUP_TAG, "%d", setupStatus);
    
    // Notify user that setup failed
    shortBeepXTimes(5);

    while (1); // Do not proceed with execution
  }

  delay(1000);
  shortBeepXTimes(1);

  xTaskCreate(RadioTask, "RadioTask", 4096, NULL, 2, NULL);
}

void loop() {

}
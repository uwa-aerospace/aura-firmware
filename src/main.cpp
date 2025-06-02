#include <Arduino.h>
#include "SPI.h"

#include <WiFi.h>
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Constants
#include "status.h"
#include "data.h"

// Flight computer peripherals
#include "gnss.h"
#include "barometer.h"
#include "sdcard.h"
#include "buzzer.h"
#include "accelerometer.h"
#include "radio.h"
#include "pyro.h"
#include "flightlogic.h"

// General defines
#define SETUP_TAG "MAINSETUP"

// IMPORTANT SETTINGS
#define MAIN_DEPLOY_ALT 300 // in M
#define RADIO_FREQ 919000000 // in Hz

// Pin and MCU function declarations
#define GNSS_RX_PIN 21
#define GNSS_TX_PIN 18
TaskHandle_t GnssTaskHandle;

#define I2C_SCL 39
#define I2C_SDA 40
TaskHandle_t BarometerTaskHandle;

#define SDIO_CMD 13
#define SDIO_CLK 12
#define SDIO_D0 11
#define SDIO_D1 10
#define SDIO_D2 17
#define SDIO_D3 14
TaskHandle_t LoggingTaskHandle;

#define SPI_SCK 2
#define SPI_MISO 4
#define SPI_MOSI 1

#define ACCEL_CS 0
#define ACCEL_INT 6
TaskHandle_t AccelerometerTaskHandle;

#define RADIO_CS 9
#define RADIO_INT 8
#define RADIO_BUSY 7

#define PYRO1 37
#define PYRO2 35
#define PYRO3 34
#define PYRO4 33
TaskHandle_t FlightLogicTaskHandle;

#define BUZZER_PIN 45
#define PROGRAMMABLE_LED 46
#define MEM_CS 3

EventGroupHandle_t sensorEventGroup;
SemaphoreHandle_t spiMutex;

void setup() {
  Serial.begin(460800);

  // Disable WiFi and BLE to save power
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_deinit();

  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  // Disable WDT to prevent the possibility of in flight restarts
  esp_task_wdt_deinit();

  SetupStatus setupStatus = SETUP_OK;

  // Reset all SPI CS pins to prevent multi-access
  pinMode(RADIO_CS, OUTPUT);
  pinMode(ACCEL_CS, OUTPUT);
  pinMode(MEM_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);
  digitalWrite(ACCEL_CS, HIGH);
  digitalWrite(MEM_CS, HIGH);

  Serial2.begin(460800, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(10000000);

  setupStatus = static_cast<SetupStatus>(setupStatus | setupGNSS(Serial2));
  setupStatus = static_cast<SetupStatus>(setupStatus | setupBarometer(I2C_SDA, I2C_SCL));
  setupStatus = static_cast<SetupStatus>(setupStatus | setupSdCard(SDIO_CMD, SDIO_CLK, SDIO_D0, SDIO_D1, SDIO_D2, SDIO_D3));

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex != NULL) {
    setupStatus = static_cast<SetupStatus>(setupStatus | setupRadio(SPI_SCK, SPI_MISO, SPI_MOSI, RADIO_CS, RADIO_INT, RADIO_BUSY, RADIO_FREQ));
    setupStatus = static_cast<SetupStatus>(setupStatus | setupAccelerometer(ACCEL_CS, ACCEL_INT));
  }
  else {
    setupStatus = static_cast<SetupStatus>(setupStatus | RADIO_ERROR | ACCEL_ERROR);
  }

  // No status checks required because they will generally always succeed
  setupPyros(PYRO1, PYRO2, PYRO3, PYRO4);
  setupBuzzer(BUZZER_PIN);
  setupFlightLogic(MAIN_DEPLOY_ALT);

  sensorEventGroup = xEventGroupCreate();
  if (sensorEventGroup == NULL) {
    ESP_LOGE(SETUP_TAG, "Could not initialize logging/sensor event groups");
    setupStatus = SDCARD_ERROR;
  }

  if (setupStatus != SETUP_OK) {
    ESP_LOGE(SETUP_TAG, "%d", setupStatus);
    
    // Notify user that setup failed
    shortBeepXTimes(5);

    while (1); // Do not proceed with execution
  }

  delay(1000);
  shortBeepXTimes(1);

  xTaskCreate(GnssTask, "GnssTask", 4096, NULL, 2, &GnssTaskHandle);
  xTaskCreate(BarometerTask, "BarometerTask", 4096, NULL, 2, &BarometerTaskHandle);
  xTaskCreatePinnedToCore(AccelerometerTask, "AccelerometerTask", 8192, NULL, 2, &AccelerometerTaskHandle, 1);
  xTaskCreate(RadioTask, "RadioTask", 4096, NULL, 2, NULL);

  xTaskCreatePinnedToCore(LoggingTask, "LoggingTask", 8192, NULL, 3, &LoggingTaskHandle, 1);
  xTaskCreatePinnedToCore(FlightLogicTask, "FlightLogicTask", 8192, NULL, 4, &FlightLogicTaskHandle, 1);
}

void loop() {

}
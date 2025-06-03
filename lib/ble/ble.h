#ifndef BLE_H
#define BLE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

extern QueueHandle_t commandQueue;
extern BLECharacteristic *pTelemetryChar;

#define TELEMETRY_SERVICE_UUID "7103fa9a-0447-427d-93b1-42eccc6b5018"
#define COMMAND_CHARACTERISTIC_UUID "5b5e2121-6a0b-483b-b90d-21a6abbcfeff"
#define TELEMETRY_CHARACTERISTIC_UUID "a9aa7873-ca44-4243-a446-b8eee777c8a3"

void setupBLE();

#endif
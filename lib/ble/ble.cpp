#include "ble.h"
#include "radio.h"

BLECharacteristic *pTelemetryChar;
BLECharacteristic *pCommandChar;
BLEServer *pServer;
BLEService *pService;

QueueHandle_t commandQueue;

#define MAX_MSG_LEN 50

class DisconnectCallback : public BLEServerCallbacks {
  void onDisconnect(BLEServer *pServer) override {
    pServer->getAdvertising()->start();
  }
};

class BLEReceiveCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    std::string command;

    if (value == "ARM")
      command = ARM_CMD;
    else if (value == "DISARM")
      command = DISARM_CMD;
    else if (value.rfind("PYRO,", 0) == 0) {
      int pyroOutput = std::stoi(value.substr(5));
      command = FIRE_PYRO_CMD + ("," + std::to_string(pyroOutput - 1));
    }
    else if (value.rfind("FREQ,", 0) == 0) {
      int frequency = std::stoi(value.substr(5, 4));
      command = RADIO_FREQ_CMD + ("," + std::to_string(frequency));
    }

    if (!command.empty()) {
      char commandCharArray[MAX_MSG_LEN] = {0};
      strncpy(commandCharArray, command.c_str(), MAX_MSG_LEN - 1);
      xQueueSend(commandQueue, (void*) commandCharArray, 0);
    }
  }
};

void setupBLE() {
  BLEDevice::init("AURA_01_AUGR");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(TELEMETRY_SERVICE_UUID);

  pServer->setCallbacks(new DisconnectCallback());

  pTelemetryChar = pService->createCharacteristic(
    TELEMETRY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTelemetryChar->addDescriptor(new BLE2902());
  BLEDevice::setMTU(256);

  pCommandChar = pService->createCharacteristic(
    COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCommandChar->setCallbacks(new BLEReceiveCallback());

  pService->start();
  pServer->getAdvertising()->start();

  commandQueue = xQueueCreate(5, MAX_MSG_LEN);
}
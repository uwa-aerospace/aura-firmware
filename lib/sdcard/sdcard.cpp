#include "sdcard.h"
#include "SD_MMC.h"

#define TAG "SDCARD"
#define BASE_DIR_NAME "/aura-log-"

bool createDir(fs::FS &fs, const char *path) {
  if (fs.mkdir(path)) {
    ESP_LOGI(TAG, "Dir %s created", path);
    return true;
  } else {
    ESP_LOGI(TAG, "mkdir failed");
    return false;
  }
}

bool dirExists(fs::FS &fs, const char *dirname) {
  File root = fs.open(dirname);

  if (!root)
    return false;

  bool isDir = root.isDirectory();
  root.close();
  return isDir;
}

bool writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file)
    return false;

  if (file.print(message))
    return true;
  else
    return false;
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for appending");
    return false;
  }
  if (file.print(message)) {
    return true;
  } else {
    ESP_LOGE(TAG, "Append failed");
    return false;
  }
}

void incrementLogPath(char* logPath) {
  size_t baseDirNameLen = strlen(BASE_DIR_NAME);
  int number;
  sscanf(logPath + baseDirNameLen, "%4d", &number);
  sprintf(logPath + baseDirNameLen, "%04d", ++number);
}

char logPath[sizeof(BASE_DIR_NAME) + 4] = BASE_DIR_NAME "0000";
char filePath[2 * sizeof(logPath) + 3];

SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
  if (!SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)) {
    ESP_LOGE(TAG, "Failed to change SD card pins");
    return SDCARD_ERROR;
  }

  if (!SD_MMC.begin()) {
    ESP_LOGE(TAG, "Failed to mount SD card");
    return SDCARD_ERROR;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    ESP_LOGE(TAG, "No SD card inserted");
    return SDCARD_ERROR;
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  printf("SD_MMC Card Size: %lluMB\n", cardSize);

  // Create base folder for this logging instance
  for (uint32_t i = 0; i < 10000; i++) {
    if (!dirExists(SD_MMC, logPath)) {
      if (!createDir(SD_MMC, logPath)) {
        ESP_LOGE(TAG, "Failed to create base folder");
        return SDCARD_ERROR;
      }
      // Folder has been created, stop searching
      break;
    }

    incrementLogPath(logPath);

    if (i == 9999) {
      ESP_LOGE(TAG, "Folder count limit reached");
      return SDCARD_ERROR;
    }
  }

  snprintf(filePath, sizeof(filePath), "%s%s.csv", logPath, logPath);
  if (!writeFile(SD_MMC, filePath, "test,hello,aura,firmware,works")) {
    ESP_LOGE(TAG, "Could not create log file");
    return SDCARD_ERROR;
  }

  return SETUP_OK;
}
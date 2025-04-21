#include "sdcard.h"
#include "SD_MMC.h"

#include "vector_type.h"
#include "data.h"

#define TAG "SDCARD"
#define BASE_DIR_NAME "/aura-log-"

File logFile;

bool createDir(fs::FS &fs, const char *path)
{
  if (fs.mkdir(path))
  {
    ESP_LOGI(TAG, "Dir %s created", path);
    return true;
  }
  else
  {
    ESP_LOGI(TAG, "mkdir failed");
    return false;
  }
}

bool dirExists(fs::FS &fs, const char *dirname)
{
  File root = fs.open(dirname);

  if (!root)
    return false;

  bool isDir = root.isDirectory();
  root.close();
  return isDir;
}

bool writeFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_WRITE);
  if (!file)
    return false;

  if (file.print(message))
    return true;
  else
    return false;
}

bool openLogFile(fs::FS &fs, const char *path)
{
  logFile = fs.open(path, FILE_APPEND);
  if (!logFile)
  {
    ESP_LOGE(TAG, "Failed to open file for appending");
    return false;
  }
  return true;
}

bool writeRawHeader()
{
  if (!logFile)
    return false;

  if (logFile.write((uint8_t *)&fileHeader, 1))
    return true;

  ESP_LOGE(TAG, "header failed");
}

bool appendRawToOpenFile(const EntryType dtype, const uint8_t *buf, const uint8_t n)
{
  if (!logFile)
    return false;

  if (logFile.write(static_cast<uint8_t>(dtype)) && logFile.write(buf, n))
    return true;

  ESP_LOGE(TAG, "Raw append failed");
  return false;
}

bool appendToOpenFile(const char *message)
{
  if (!logFile)
    return false;
  if (logFile.print(message))
    return true;
  ESP_LOGE(TAG, "Append failed");
  return false;
}

void flushLogFile()
{
  if (logFile)
    logFile.flush();
}

void incrementLogPath(char *logPath)
{
  size_t baseDirNameLen = strlen(BASE_DIR_NAME);
  int number;
  sscanf(logPath + baseDirNameLen, "%4d", &number);
  sprintf(logPath + baseDirNameLen, "%04d", ++number);
}

char logPath[sizeof(BASE_DIR_NAME) + 4] = BASE_DIR_NAME "0000";
char filePath[2 * sizeof(logPath) + 3];

SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
  if (!SD_MMC.setPins(clk, cmd, d0, d1, d2, d3))
  {
    ESP_LOGE(TAG, "Failed to change SD card pins");
    return SDCARD_ERROR;
  }

  if (!SD_MMC.begin())
  {
    ESP_LOGE(TAG, "Failed to mount SD card");
    return SDCARD_ERROR;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    ESP_LOGE(TAG, "No SD card inserted");
    return SDCARD_ERROR;
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  printf("SD_MMC Card Size: %lluMB\n", cardSize);

  // Create base folder for this logging instance
  for (uint32_t i = 0; i < 10000; i++)
  {
    if (!dirExists(SD_MMC, logPath))
    {
      if (!createDir(SD_MMC, logPath))
      {
        ESP_LOGE(TAG, "Failed to create base folder");
        return SDCARD_ERROR;
      }
      // Folder has been created, stop searching
      break;
    }

    incrementLogPath(logPath);

    if (i == 9999)
    {
      ESP_LOGE(TAG, "Folder count limit reached");
      return SDCARD_ERROR;
    }
  }

  snprintf(filePath, sizeof(filePath), "%s%s.bin", logPath, logPath);
  // const char *header = "timestamp,verticalVelocity,accelX,accelY,accelZ,rawAccelX,rawAccelY,rawAccelZ,gyroX,gyroY,gyroZ,rawGyroX,rawGyroY,rawGyroZ,quatnW,quatnX,quatnY,quatnZ\n";
  // if (!writeFile(SD_MMC, filePath, header))
  // {
  //   ESP_LOGE(TAG, "Could not create log file");
  //   return SDCARD_ERROR;
  // }

  ESP_LOGI(TAG, "SD card setup successful");
  return SETUP_OK;
}

char dataString[1024];
int strPosn = 0;
const uint8_t base = 10;
const char cs = ',';

void writeIntData(int dataValue)
{
  itoa(dataValue, dataString + strPosn, base);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = cs;
  strPosn++;
}

void writeULongData(unsigned long dataValue)
{
  ultoa(dataValue, dataString + strPosn, base);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = cs;
  strPosn++;
}

void writeLongData(long dataValue)
{
  ltoa(dataValue, dataString + strPosn, base);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = cs;
  strPosn++;
}

void writeFloatData(float dataValue, byte decimals)
{
  long fracInt;
  float partial;

  // sign portion
  if (dataValue < 0)
  {
    dataString[strPosn] = '-';
    strPosn++;
    dataValue *= -1;
  }

  // integer portion
  itoa((int)dataValue, dataString + strPosn, base);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = '.';
  strPosn++;

  // fractional portion
  partial = dataValue - (int)(dataValue);
  fracInt = (long)(partial * powf(10, decimals));
  if (fracInt == 0)
  {
    dataString[strPosn] = '0';
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
  }
  else
  {
    decimals--;
    while (fracInt < powf(10, decimals))
    {
      dataString[strPosn] = '0';
      strPosn++;
      decimals--;
    }
    ltoa(fracInt, dataString + strPosn, base);
    while (dataString[strPosn] != '\0')
    {
      strPosn++;
    }
    dataString[strPosn] = ',';
    strPosn++;
  }
}

float logTime = 0;
uint64_t lastLogTime = 0;
uint64_t numLogs = 0;

float buf[32];

void LoggingTask(void *pvParameters)
{
  openLogFile(SD_MMC, filePath);
  writeRawHeader();

  while (1)
  {
    EventBits_t bits = xEventGroupWaitBits(
        sensorEventGroup,
        IMU_SENSOR_EVENT | BARO_SENSOR_EVENT | GNSS_SENSOR_EVENT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);

    uint64_t now = micros();
    float dt = (now - lastLogTime) * 1e-3;
    lastLogTime = now;

    if (dt < 1000)
      logTime += dt;

    buf[0] = logTime;

    buf[1] = accelVertVel;
    buf[2] = accelCorrected.x;
    buf[3] = accelCorrected.y;
    buf[4] = accelCorrected.z;
    buf[5] = accelMs.x;
    buf[6] = accelMs.y;
    buf[7] = accelMs.z;

    buf[8] = gyroCorrected.x;
    buf[9] = gyroCorrected.y;
    buf[10] = gyroCorrected.z;
    buf[11] = gyroDps.x;
    buf[12] = gyroDps.y;
    buf[13] = gyroDps.z;

    buf[14] = attitudeQuatn.w;
    buf[15] = attitudeQuatn.v.x;
    buf[16] = attitudeQuatn.v.y;
    buf[17] = attitudeQuatn.v.z;

    appendRawToOpenFile(EntryType::IMU, (uint8_t *)&buf, sizeof(float) * 17);

    if (numLogs > 4000)
    { // Save to file every ~10 seconds
      flushLogFile();
      numLogs = 0;
    }

    numLogs++;
  }
}
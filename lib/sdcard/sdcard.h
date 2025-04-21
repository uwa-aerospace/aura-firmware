#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include "status.h"
#include "FS.h"

bool writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);
SetupStatus setupSdCard(uint8_t cmd, uint8_t clk, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void LoggingTask(void *pvParameters);

// this way we won't need to touch the 'extractor' program *at all to add support for more data - did I miss any?
enum class DataType : uint8_t
{
    INT = 0,
    FLOAT = 1,
    BOOL = 2
};

enum class EntryType : uint8_t
{
    IMU = 0,
    BARO = 1,
    SENSOR = 2
};

typedef struct __attribute__((packed))
{
    char name[32]; // data type string identifier (for csv output)
    DataType type;
} DataEntry;

// using struct in case I want to add more settings
typedef struct __attribute__((packed))
{
    char name[32]; // this name is used in extractor program to extract data (--name "imu")
    DataEntry dataOrder[24];
    uint8_t dataOrderSize; // maybe a 'DataType::END' instead?
    EntryType id;          // to link entries to this
} sensor_output_header;

// IMU sensor data
sensor_output_header fileHeader[] = {
    {.name = "imu", .dataOrder = {
                        {"timestamp", DataType::FLOAT}, // timestamp

                        // verticalVelocity
                        {"verticalVelocity", DataType::FLOAT},

                        // accelX, accelY, accelZ
                        {"accelX", DataType::FLOAT},
                        {"accelY", DataType::FLOAT},
                        {"accelZ", DataType::FLOAT},

                        // rawAccelX, rawAccelY, rawAccelZ
                        {"rawAccelX", DataType::FLOAT},
                        {"rawAccelY", DataType::FLOAT},
                        {"rawAccelZ", DataType::FLOAT},

                        // gyroX, gyroY, gyroZ
                        {"gyroX", DataType::FLOAT},
                        {"gyroY", DataType::FLOAT},
                        {"gyroZ", DataType::FLOAT},

                        // rawGyroX, rawGyroY, rawGyroZ
                        {"rawGyroX", DataType::FLOAT},
                        {"rawGyroY", DataType::FLOAT},
                        {"rawGyroZ", DataType::FLOAT},

                        // quatnW, quatnX, quatnY, quatnZ
                        {"quatnW", DataType::FLOAT},
                        {"quatnX", DataType::FLOAT},
                        {"quatnY", DataType::FLOAT},
                        {"quatnZ", DataType::FLOAT},
                    },
     .dataOrderSize = 18,
     .id = EntryType::IMU},
};

#endif
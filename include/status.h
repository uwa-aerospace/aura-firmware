#ifndef STATUS_H
#define STATUS_H

enum SetupStatus : uint8_t {
    SETUP_OK        = 1 << 0,
    GNSS_ERROR      = 1 << 1,
    BAROMETER_ERROR = 1 << 2,
    SDCARD_ERROR    = 1 << 3,
};

#endif
#ifndef ERRORS_H
#define ERRORS_H

enum SetupStatus : uint8_t {
    SETUP_OK        = 0,
    GNSS_ERROR      = 1 << 0,
    BAROMETER_ERROR = 1 << 1
};

#endif
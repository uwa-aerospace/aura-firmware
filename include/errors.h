#ifndef ERRORS_H
#define ERRORS_H

enum SetupStatus : uint8_t {
    SETUP_OK        = 1 << 0,
    GNSS_ERROR      = 1 << 1,
    BAROMETER_ERROR = 1 << 2
};

#endif
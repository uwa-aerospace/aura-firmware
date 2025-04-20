#ifndef DATA_H
#define DATA_H

#include "vector_type.h"
#include "quaternion_type.h"

enum FlightState {
    FLIGHT_IDLE     = 0,
    FLIGHT_ARMED    = 1,
    FLIGHT_BOOST    = 2,
    FLIGHT_BURNOUT  = 3,
    FLIGHT_APOGEE   = 4,
    FLIGHT_MAIN     = 5,
};
extern FlightState flightState;

#define IMU_SENSOR_EVENT  (1 << 0)
#define BARO_SENSOR_EVENT (1 << 1)
#define GNSS_SENSOR_EVENT (1 << 2)

extern EventGroupHandle_t sensorEventGroup;

// Acceleration in m/s^2
extern vec3_t accelMs;

// Gyro angular rate in deg/s
extern vec3_t gyroDps;

// Acceleration in world frame
extern vec3_t accelCorrected;

// Gyro angular rates corrected for zero-rate bias
extern vec3_t gyroCorrected;

extern quat_t attitudeQuatn;
extern float tiltAngle;
extern float accelVertVel;

#endif
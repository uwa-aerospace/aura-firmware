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

// IMU DATA
extern vec3_t accelRaw; // m/s^2, unfiltered
extern vec3_t gyroRaw;  // dps, unfiltered
extern vec3_t accelCorrected; // m/s^2, filtered 1D kalman, rotated to world frame
extern vec3_t gyroCorrected;  // dps, filtered 1D kalman, zero biases subtracted

extern quat_t attitudeQuatn; // result of gyro integration
extern float tiltAngle;      // gyro angle from vertical [0,0,1]
extern float accelVertVel;   // result of Z acceleration integration
extern float maxAccelVertVel;

// BARO DATA
extern float baroAltitudeMSL; // m, filtered 2D kalman
extern float baroAltitudeAGL; // m, filtered 2D kalman
extern float baroVertVel;     // m/s, filtered 2D kalman
extern int baroPressure;      // pascals, unfiltered
extern float baroPadAltitude;

// GNSS DATA
extern float gnssLatitude;
extern float gnssLongitude;
extern float gnssAltitudeMSL;
extern float gnssAltitudeAGL;
extern float gnssVertVel;
extern float maxGnssVertVel;
extern float gnssPadAltitude;
extern float gnssPDOP;
extern bool gnssValidReadings;
extern bool gnssHasFix;
#endif
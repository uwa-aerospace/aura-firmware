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
extern EventGroupHandle_t loggingEventGroup;
extern SemaphoreHandle_t spiMutex;

extern bool accelCalibrationCycle;
extern bool baroCalibrationCycle;
extern bool gnssCalibrationCycle;
#define CALIBRATION_APPLY_THRESHOLD 1.5 // Maximum acceleration acceptable to apply sensor calibrations

// IMU DATA
extern vec3_t accelRaw; // g, unfiltered
extern vec3_t gyroRaw;  // dps, unfiltered
extern vec3_t accelCorrected; // g, rotated to world frame
extern vec3_t gyroCorrected;  // dps, zero biases subtracted, rotated to world frame

extern quat_t attitudeQuatn; // result of gyro integration
extern float tiltAngle;      // gyro angle from rocket axis
extern float accelVertVel;   // result of Z acceleration integration
extern float maxAccelVertVel;

// BARO DATA
extern float baroAltitudeMSL; // m, unfiltered
extern float rawAltitudeAGL;  // m, unfiltered, pad altitude subtracted
extern float baroAltitudeAGL; // m, filtered 2D kalman
extern float kalmanBaroVel;   // m/s, filtered 2D kalman
extern float baroVertVel;     // m/s filtered 2D kalman, smoothed with EMA
extern int baroPressure;      // pascals, unfiltered
extern float baroPadAltitude;

// GNSS DATA
extern float gnssLatitude;
extern float gnssLongitude;
extern float gnssAltitudeMSL;
extern float gnssAltitudeAGL;
extern float gnssVertVel;
extern float gnssPadAltitude;
extern float gnssPDOP;
extern bool gnssValidReadings;
extern bool gnssHasFix;

extern unsigned long flightStartTime;

extern uint16_t accelLaunchCtr;
extern uint16_t falseLaunchCtr;

extern bool canDetectBurnout;
extern uint16_t accelBurnoutCtr;

extern uint16_t baroApogeeCtr;
extern uint16_t gnssApogeeCtr;
extern uint16_t accelApogeeCtr;
extern uint16_t gyroApogeeCtr;

extern uint16_t baroMainCtr;
extern uint16_t gnssMainCtr;

extern uint16_t gyroLandingCtr;
extern uint16_t baroLandingCtr;
extern uint16_t gnssLandingCtr;
#endif
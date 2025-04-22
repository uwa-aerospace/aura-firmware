#include "data.h"

// Start armed for now
FlightState flightState = FLIGHT_ARMED;

vec3_t accelRaw = vec3_t(0,0,0);
vec3_t gyroRaw = vec3_t(0,0,0);
vec3_t accelCorrected = vec3_t(0,0,0);
vec3_t gyroCorrected = vec3_t(0,0,0);

quat_t attitudeQuatn(1.0f, 0.0f, 0.0f, 0.0f);
float tiltAngle = 0;
float accelVertVel = 0;

float baroAltitudeMSL = 0;
float baroAltitudeAGL = 0;
float baroVertVel = 0;
int baroPressure = 0;
float padAltitude = 0;

// GPS DATA
float gpsLatitude = 0;
float gpsLongitude = 0;
float gpsAltitude = 0;
float gpsVertVel = 0;
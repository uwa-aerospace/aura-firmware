#include "data.h"

// Start armed for now
FlightState flightState = FLIGHT_ARMED;

vec3_t accelMs = vec3_t(0,0,0);
vec3_t gyroDps = vec3_t(0,0,0);
vec3_t accelCorrected = vec3_t(0,0,0);
vec3_t gyroCorrected = vec3_t(0,0,0);

quat_t attitudeQuatn(1.0f, 0.0f, 0.0f, 0.0f);
float tiltAngle = 0;
float accelVertVel = 0;
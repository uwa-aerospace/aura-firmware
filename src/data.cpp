#include "data.h"

FlightState flightState = FLIGHT_IDLE;

bool accelCalibrationCycle = false;
bool baroCalibrationCycle = false;
bool gnssCalibrationCycle = false;

vec3_t accelRaw = vec3_t(0,0,0);
vec3_t gyroRaw = vec3_t(0,0,0);
vec3_t accelCorrected = vec3_t(0,0,0);
vec3_t gyroCorrected = vec3_t(0,0,0);

quat_t attitudeQuatn(1.0f, 0.0f, 0.0f, 0.0f);
float tiltAngle = 0;
float accelVertVel = 0;
float maxAccelVertVel = 0;

float baroAltitudeMSL = 0;
float rawAltitudeAGL = 0;
float baroAltitudeAGL = 0;
float kalmanBaroVel = 0;
float baroVertVel = 0;
int baroPressure = 0;
float baroPadAltitude = 0;

// GNSS DATA
float gnssLatitude = 0;
float gnssLongitude = 0;
float gnssAltitudeMSL = 0;
float gnssAltitudeAGL = 0;
float gnssVertVel = 0;
float gnssPadAltitude = 0;
float gnssPDOP = 0;
bool gnssValidReadings = false;
bool gnssHasFix = false;

unsigned long flightStartTime = 0;

uint16_t accelLaunchCtr = 0;
uint16_t falseLaunchCtr = 0;

bool canDetectBurnout = false;
uint16_t accelBurnoutCtr = 0;

uint16_t baroApogeeCtr = 0;
uint16_t gnssApogeeCtr = 0;
uint16_t accelApogeeCtr = 0;
uint16_t gyroApogeeCtr = 0;

uint16_t baroMainCtr = 0;
uint16_t gnssMainCtr = 0;

uint16_t gyroLandingCtr = 0;
uint16_t baroLandingCtr = 0;
uint16_t gnssLandingCtr = 0;
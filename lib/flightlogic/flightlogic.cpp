#include "flightlogic.h"
#include "pyro.h"
#include "buzzer.h"
#include "radio.h"
#include "sdcard.h"

#include "data.h"
#include "vector_type.h"

int mainDeployAltitude = 0;

esp_timer_handle_t apogeeBackupTimer;
esp_timer_handle_t mainBackupTimer;

#define BACKUP_DELAY 2000000 // 2 seconds, 2 million microseconds

void apogeeBackupFire(void* arg) {
  firePyro(1);
}

void mainBackupFire(void* arg) {
  firePyro(3);
}

void setupFlightLogic(int mainDepAlt) {
  mainDeployAltitude = mainDepAlt;

  esp_timer_create_args_t apogeeBackupTimerArgs = {
    .callback = &apogeeBackupFire,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "apogeeBckpTimer"
  };

  esp_timer_create_args_t mainBackupTimerArgs = {
    .callback = &mainBackupFire,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "mainBckpTimer"
  };

  esp_timer_create(&apogeeBackupTimerArgs, &apogeeBackupTimer);
  esp_timer_create(&mainBackupTimerArgs, &mainBackupTimer);
}

void FlightLogicTask(void* pvParameters) {
  while (1) {
    // Process whenever there is new sensor data available
    EventBits_t bits = xEventGroupWaitBits(
      sensorEventGroup,
      IMU_SENSOR_EVENT | BARO_SENSOR_EVENT | GNSS_SENSOR_EVENT,
      pdTRUE,
      pdFALSE,
      portMAX_DELAY
    );

    switch (flightState) {
      case FLIGHT_IDLE: {
        // do nothing, wait for arm
      } break;
      case FLIGHT_ARMED: {
        /* Launch is detected when:
         - Accel-based vertical velocity is > 5m/s AND total acceleration > 3G for 5 readings in a row
        */

        // ONLY UPDATE IF NEW IMU DATA IS AVAILABLE
        if (bits & IMU_SENSOR_EVENT) {
          if (accelVertVel > 5 && accelRaw.mag() > 3)
            accelLaunchCtr++;
          else // Reset counter if condition is no longer true (counters transients)
            accelLaunchCtr = 0;
        }

        // Sound buzzer to indicate that the system is still alive and actively calibrating
        if (accelCalibrationCycle && baroCalibrationCycle && gnssCalibrationCycle) {
          shortBeep();
          accelCalibrationCycle = false;
          baroCalibrationCycle = false;
          gnssCalibrationCycle = false;
        }

        if (accelLaunchCtr > 5) {
          longBeepXTimes(3);
          xTimerChangePeriod(radioTransmitTimer, pdMS_TO_TICKS(RADIO_FLIGHT_TX_RATE), 0);
          flightStartTime = millis();
          flightState = FLIGHT_BOOST;
        }

      } break;
      case FLIGHT_BOOST: {
        /* Burnout is detected when:
         - Accel velocity has dropped 3m/s below the max accel velocity for 5 readings in a row
         - Rocket velocity has exceeded 20m/s at any point in the flight
        */

        /* False launch is detected when:
         - Flight time is greater than 250ms (ignores initial motor noise transients)
         - Acceleration drops below 3G
         - Rocket velocity has not exceeded 20m/s at any point in the flight, i.e. burnout cannot be detected
         And all 3 conditions are true for 25 readings in a row (ignores further transients)

         If false launch is detected, reset IMU values and return to FLIGHT_ARMED state
        */
        
        // ONLY UPDATE IF NEW IMU DATA IS AVAILABLE
        if (bits & IMU_SENSOR_EVENT) {
          if (millis() - flightStartTime > 250 && accelRaw.mag() < 3 && !canDetectBurnout)
            falseLaunchCtr++;
          else
            falseLaunchCtr = 0;

          if (maxAccelVertVel > 20)
            canDetectBurnout = true;

          float accelDelta = maxAccelVertVel - accelVertVel;

          if (accelDelta > 3 && canDetectBurnout)
            accelBurnoutCtr++;
          else
            accelBurnoutCtr = 0;
        }

        if (falseLaunchCtr > 25) {
          shortBeepXTimes(2);
          
          // Reset integrated sensor values (IMU only) to prevent error accumulation
          accelVertVel = 0;
          maxAccelVertVel = 0;
          
          attitudeQuatn = quat_t(1,0,0,0);
          tiltAngle = 0;

          falseLaunchCtr = 0; // Reset false launch counter to prevent system from redetecting false launch on actual launch
          accelLaunchCtr = 0; // Reset launch counter to prevent system from immediately going back to FLIGHT_BOOST
          
          xTimerChangePeriod(radioTransmitTimer, pdMS_TO_TICKS(RADIO_ARMED_TX_RATE), 0);
          flightState = FLIGHT_ARMED;
        }

        if (accelBurnoutCtr > 5) {
          flightState = FLIGHT_BURNOUT;
        }

      } break;
      case FLIGHT_BURNOUT: {
        /* Apogee is detected when 2/4 conditions are true:
         - Accel velocity is < 0 for 20 readings in a row
         - Gyro tilt angle is > 90 degrees for 20 readings in a row
         - Barometric velocity < 0 for 50 readings in a row AND velocity is < 150m/s (Mach lockout)
         - GNSS velocity < 0 for 25 readings in a row, valid readings AND accel velocity < 150m/s (pDOP fault tolerance)
        */

        if (bits & IMU_SENSOR_EVENT) {
          if (accelVertVel < 0)
            accelApogeeCtr++;
          else
            accelApogeeCtr = 0;

          if (tiltAngle > 90)
            gyroApogeeCtr++;
          else
            gyroApogeeCtr = 0;
        }

        if (bits & BARO_SENSOR_EVENT) {
          if (baroVertVel < 0 && accelVertVel < 150)
            baroApogeeCtr++;
          else
            baroApogeeCtr = 0;
        }

        if (bits & GNSS_SENSOR_EVENT) {
          if (gnssVertVel < 0 && gnssValidReadings && accelVertVel < 150)
            gnssApogeeCtr++;
          else
            gnssApogeeCtr = 0;
        }

        int apogeeConditionsCtr = (accelApogeeCtr > 20) + (gyroApogeeCtr > 20) + (baroApogeeCtr > 50) + (gnssApogeeCtr > 25);
        if (apogeeConditionsCtr >= 2) {
          firePyro(0);
          esp_timer_start_once(apogeeBackupTimer, BACKUP_DELAY);
          flightState = FLIGHT_APOGEE;
        }

      } break;
      case FLIGHT_APOGEE: {
        /* Main deployment is detected when 1/2 conditions are true:
         - Barometric AGL altitude is below threshold for 50 readings in a row
         - GNSS AGL altitude is below threshold for 15 readings in a row
        */
        if (bits & BARO_SENSOR_EVENT) {
          if (baroAltitudeAGL < mainDeployAltitude)
            baroMainCtr++;
          else
            baroMainCtr = 0;
        }

        if (bits & GNSS_SENSOR_EVENT) {
          if (gnssAltitudeAGL < mainDeployAltitude && gnssValidReadings)
            gnssMainCtr++;
          else
            gnssMainCtr = 0;
        }

        if (baroMainCtr > 50 || gnssMainCtr > 15) {
          firePyro(2);
          esp_timer_start_once(mainBackupTimer, BACKUP_DELAY);
          flightState = FLIGHT_MAIN;
        }
        
      } break;
      case FLIGHT_MAIN: {
        /* Landing is detected when 1/4 conditions are true:
         - Barometric velocity is between -0.5 and 0.5 for 50 readings in a row
         - GNSS velocity is between -0.5 and 0.5 for 25 readings in a row
         - Magnitude of gyro rates less than 2.0 deg/sec for 200 readings in a row
        */
        if (bits & IMU_SENSOR_EVENT) {
          float gyroMag = gyroRaw.mag();
          if (gyroMag < 2.0)
            gyroLandingCtr++;
          else
            gyroLandingCtr = 0;
        }

        if (bits & BARO_SENSOR_EVENT) {
          if (baroVertVel > -0.5 && baroVertVel < 0.5)
            baroLandingCtr++;
          else
            baroLandingCtr = 0;
        }

        if (bits & GNSS_SENSOR_EVENT) {
          if (gnssVertVel > -0.5 && gnssVertVel < 0.5 && gnssValidReadings)
            gnssLandingCtr++;
          else
            gnssLandingCtr = 0;
        }

        if (baroLandingCtr > 50 || gnssLandingCtr > 50 || gyroLandingCtr > 200) {
          flushLogFile();
          xTimerChangePeriod(radioTransmitTimer, pdMS_TO_TICKS(RADIO_ARMED_TX_RATE), 0);
          flightState = FLIGHT_IDLE;
        }
        
      } break;
      default: {
      } break;
    }
  }
}
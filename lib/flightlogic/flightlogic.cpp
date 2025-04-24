#include "flightlogic.h"
#include "data.h"
#include "pyro.h"
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
        /* Launch is detected when 1/2 conditions are true:
         - Accel-based vertical velocity is > 5m/s for 10 readings in a row (20ms delay)
         - GNSS vertical velocity > 5m/s for 5 readings in a row (200ms delay)
        */

        // ONLY UPDATE IF NEW IMU DATA IS AVAILABLE
        if (bits & IMU_SENSOR_EVENT) {
          if (accelVertVel > 5)
            accelLaunchCtr++;
          else // Reset counter if condition is no longer true (counters transients)
            accelLaunchCtr = 0;
        }

        // ONLY UPDATE IF NEW GNSS DATA IS AVAILABLE
        if (bits & GNSS_SENSOR_EVENT) {
          if (gnssVertVel > 5 && gnssValidReadings) // Make sure GNSS is reading correct values based on PDOP
            gnssLaunchCtr++;
          else
            gnssLaunchCtr = 0;
        }

        if (accelLaunchCtr > 10 || gnssLaunchCtr > 5) {
          // TODO: Beep nonblocking to indicate state change
          flightState = FLIGHT_BOOST;
        }

      } break;
      case FLIGHT_BOOST: {
        /* Burnout is detected when 1/2 conditions are true:
         - Accel velocity has dropped 3m/s below the max accel velocity for 10 readings in a row
         - GNSS vertical velocity has dropped 3m/s below the max GNSS velocity for 5 readings in a row
        */
        
        // ONLY UPDATE IF NEW IMU DATA IS AVAILABLE
        if (bits & IMU_SENSOR_EVENT) {
          float accelDelta = maxAccelVertVel - accelVertVel;
          if (accelDelta > 3)
            accelBurnoutCtr++;
          else
            accelBurnoutCtr = 0;
        }
        
        // ONLY UPDATE IF NEW GNSS DATA IS AVAILABLE
        if (bits & GNSS_SENSOR_EVENT) {
          float gnssDelta = maxGnssVertVel - gnssVertVel;
          if (gnssDelta > 3 && gnssValidReadings) // High chance of invalid readings at high velocities
            gnssBurnoutCtr++;
          else
            gnssBurnoutCtr = 0;
        }

        if (accelBurnoutCtr > 10 || gnssBurnoutCtr > 5) {
          flightState = FLIGHT_BURNOUT;
        }

      } break;
      case FLIGHT_BURNOUT: {
        /* Apogee is detected when 2/4 conditions are true:
         - Accel velocity is < 0 for 50 readings in a row
         - Gyro tilt angle is > 90 degrees for 50 readings in a row
         - Barometric velocity < 0 for 30 readings in a row AND velocity is < 250m/s (Mach lockout)
         - GNSS velocity < 0 for 15 readings in a row
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
          if (baroVertVel < 0 && accelVertVel < 250)
            baroApogeeCtr++;
          else
            baroApogeeCtr = 0;
        }

        if (bits & GNSS_SENSOR_EVENT) {
          if (gnssVertVel < 0 && gnssValidReadings)
            gnssApogeeCtr++;
          else
            gnssApogeeCtr = 0;
        }

        int apogeeConditionsCtr = (accelApogeeCtr > 50) + (gyroApogeeCtr > 50) + (baroApogeeCtr > 30) + (gnssApogeeCtr > 15);
        if (apogeeConditionsCtr >= 2) {
          firePyro(0);
          esp_timer_start_once(apogeeBackupTimer, BACKUP_DELAY);
          flightState = FLIGHT_APOGEE;
        }

      } break;
      case FLIGHT_APOGEE: {
        /* Main deployment is detected when 1/2 conditions are true:
         - Barometric AGL altitude is below threshold for 25 readings in a row
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

        if (baroMainCtr > 25 || gnssMainCtr > 15) {
          firePyro(2);
          esp_timer_start_once(mainBackupTimer, BACKUP_DELAY);
          flightState = FLIGHT_MAIN;
        }
        
      } break;
      case FLIGHT_MAIN: {
        /* Landing is detected when 1/4 conditions are true:
         - Barometric velocity is between -0.5 and 0.5 for 50 readings in a row
         - GNSS velocity is between -0.5 and 0.5 for 25 readings in a row
         - Magnitude of accel is between 0.9-1.1G for 50 readings in a row
         - Magnitude of gyro rates is between 0 and 1.5 deg/sec for 50 readings in a row
        */
        if (bits & IMU_SENSOR_EVENT) {
          float accelMag = accelRaw.mag();
          if (accelMag > 0.9 && accelMag < 1.1)
            accelLandingCtr++;
          else
            accelLandingCtr = 0;

          float gyroMag = gyroRaw.mag();
          if (gyroMag < 1.5)
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
          if (gnssVertVel < -0.5 && gnssValidReadings > 0.5 && gnssValidReadings)
            gnssLandingCtr++;
          else
            gnssLandingCtr = 0;
        }

        if (baroLandingCtr > 25 || gnssLandingCtr > 50 || accelLandingCtr > 50 || gyroLandingCtr > 50) {
          flightState = FLIGHT_IDLE;
        }
        
      } break;
      default: {
      } break;
    }
  }
}
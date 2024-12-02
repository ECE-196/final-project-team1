#include "IMUCalibration.h"
#include <Arduino.h>

CalibrationOffsets IMUCalibration::calibrateIMU(BNO08x& imu) {
    Serial.println("Starting IMU calibration. Keep the device still...");
    
    // Enable the reports we need
    if (imu.enableRawAccelerometer(1) == true) {
        Serial.println(F("Raw Accelerometer enabled"));
    } else {
        Serial.println("Could not enable raw accelerometer");
    }

    if (imu.enableRawGyro(1) == true) {
        Serial.println(F("Raw Gyro enabled"));
    } else {
        Serial.println("Could not enable raw gyro");
    }
    
    delay(100); // Give time for the sensor to start reporting
    
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int accelSamples = 0;
    int gyroSamples = 0;
    CalibrationOffsets offsets = {0, 0, 0, 0, 0, 0};

    // Collect samples for 1 second
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {  // 1 second calibration
        if (imu.getSensorEvent()) {
            uint8_t reportID = imu.getSensorEventID();
            
            switch (reportID) {
                case SENSOR_REPORTID_RAW_ACCELEROMETER:
                    sum_ax += imu.getRawAccelX();
                    sum_ay += imu.getRawAccelY();
                    sum_az += imu.getRawAccelZ();
                    accelSamples++;
                    break;
                case SENSOR_REPORTID_RAW_GYROSCOPE:
                    sum_gx += imu.getRawGyroX();
                    sum_gy += imu.getRawGyroY();
                    sum_gz += imu.getRawGyroZ();
                    gyroSamples++;
                    break;
            }
        }
        delayMicroseconds(10);
    }

    // Calculate and store offsets
    if (accelSamples > 0 && gyroSamples > 0) {
        offsets.accel_x = sum_ax / accelSamples;
        offsets.accel_y = sum_ay / accelSamples;
        offsets.accel_z = sum_az / accelSamples;
        offsets.gyro_x = sum_gx / gyroSamples;
        offsets.gyro_y = sum_gy / gyroSamples;
        offsets.gyro_z = sum_gz / gyroSamples;
        
        Serial.println("Calibration complete! Offsets:");
        Serial.printf("Accel samples: %d\n", accelSamples);
        Serial.printf("Gyro samples: %d\n", gyroSamples);
        Serial.printf("Accel: X=%d, Y=%d, Z=%d\n", offsets.accel_x, offsets.accel_y, offsets.accel_z);
        Serial.printf("Gyro: X=%d, Y=%d, Z=%d\n", offsets.gyro_x, offsets.gyro_y, offsets.gyro_z);
    } else {
        Serial.println("Calibration failed! No samples collected.");
    }

    return offsets;
}

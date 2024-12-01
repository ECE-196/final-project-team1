#include "IMUCalibration.h"
#include <Arduino.h>

CalibrationOffsets IMUCalibration::calibrateIMU(BNO08x& imu) {
    Serial.println("Starting IMU calibration. Keep the device still...");
    
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int validSamples = 0;
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
                    break;
                case SENSOR_REPORTID_RAW_GYROSCOPE:
                    sum_gx += imu.getRawGyroX();
                    sum_gy += imu.getRawGyroY();
                    sum_gz += imu.getRawGyroZ();
                    validSamples++;
                    break;
            }
        }
        delayMicroseconds(10);
    }

    // Calculate and store offsets
    if (validSamples > 0) {
        offsets.accel_x = sum_ax / validSamples;
        offsets.accel_y = sum_ay / validSamples;
        offsets.accel_z = sum_az / validSamples;
        offsets.gyro_x = sum_gx / validSamples;
        offsets.gyro_y = sum_gy / validSamples;
        offsets.gyro_z = sum_gz / validSamples;
        
        Serial.println("Calibration complete! Offsets:");
        Serial.printf("Accel: X=%d, Y=%d, Z=%d\n", offsets.accel_x, offsets.accel_y, offsets.accel_z);
        Serial.printf("Gyro: X=%d, Y=%d, Z=%d\n", offsets.gyro_x, offsets.gyro_y, offsets.gyro_z);
    }

    return offsets;
}

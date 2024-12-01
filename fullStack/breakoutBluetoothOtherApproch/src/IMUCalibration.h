#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "SparkFun_BNO08x_Arduino_Library.h"

struct CalibrationOffsets {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

class IMUCalibration {
public:
    static CalibrationOffsets calibrateIMU(BNO08x& imu);
};

#endif

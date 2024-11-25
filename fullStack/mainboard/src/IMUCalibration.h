#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Arduino.h>

class IMUCalibration {
private:
    // Offset values
    int16_t accel_offset_x = 0;
    int16_t accel_offset_y = 0;
    int16_t accel_offset_z = 0;
    int16_t gyro_offset_x = 0;
    int16_t gyro_offset_y = 0;
    int16_t gyro_offset_z = 0;

    // Number of samples for calibration
    static const int CALIBRATION_SAMPLES = 100;
    
    // Moving average filter buffer size
    static const int FILTER_SIZE = 5;
    
    // Circular buffers for moving average
    int16_t accel_x_buffer[FILTER_SIZE] = {0};
    int16_t accel_y_buffer[FILTER_SIZE] = {0};
    int16_t accel_z_buffer[FILTER_SIZE] = {0};
    int16_t gyro_x_buffer[FILTER_SIZE] = {0};
    int16_t gyro_y_buffer[FILTER_SIZE] = {0};
    int16_t gyro_z_buffer[FILTER_SIZE] = {0};
    
    int buffer_index = 0;

public:
    IMUCalibration();
    
    // Calibration function
    void calibrate(int16_t& ax, int16_t& ay, int16_t& az, 
                  int16_t& gx, int16_t& gy, int16_t& gz);
    
    // Apply calibration and filtering to readings
    void processReadings(int16_t& ax, int16_t& ay, int16_t& az,
                        int16_t& gx, int16_t& gy, int16_t& gz);
                        
private:
    // Helper function for moving average filter
    int16_t applyMovingAverage(int16_t newValue, int16_t* buffer);
};

#endif 
#include "IMUCalibration.h"

IMUCalibration::IMUCalibration() {
    // Initialize buffers to 0
    for (int i = 0; i < FILTER_SIZE; i++) {
        accel_x_buffer[i] = 0;
        accel_y_buffer[i] = 0;
        accel_z_buffer[i] = 0;
        gyro_x_buffer[i] = 0;
        gyro_y_buffer[i] = 0;
        gyro_z_buffer[i] = 0;
    }
}

void IMUCalibration::calibrate(int16_t& ax, int16_t& ay, int16_t& az,
                              int16_t& gx, int16_t& gy, int16_t& gz) {
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        delay(10); // Small delay between readings
    }
    
    // Calculate average offsets
    accel_offset_x = sum_ax / CALIBRATION_SAMPLES;
    accel_offset_y = sum_ay / CALIBRATION_SAMPLES;
    accel_offset_z = sum_az / CALIBRATION_SAMPLES;
    gyro_offset_x = sum_gx / CALIBRATION_SAMPLES;
    gyro_offset_y = sum_gy / CALIBRATION_SAMPLES;
    gyro_offset_z = sum_gz / CALIBRATION_SAMPLES;
}

void IMUCalibration::processReadings(int16_t& ax, int16_t& ay, int16_t& az,
                                   int16_t& gx, int16_t& gy, int16_t& gz) {
    // Remove offsets
    ax -= accel_offset_x;
    ay -= accel_offset_y;
    az -= accel_offset_z;
    gx -= gyro_offset_x;
    gy -= gyro_offset_y;
    gz -= gyro_offset_z;
    
    // Apply moving average filter
    ax = applyMovingAverage(ax, accel_x_buffer);
    ay = applyMovingAverage(ay, accel_y_buffer);
    az = applyMovingAverage(az, accel_z_buffer);
    gx = applyMovingAverage(gx, gyro_x_buffer);
    gy = applyMovingAverage(gy, gyro_y_buffer);
    gz = applyMovingAverage(gz, gyro_z_buffer);
}

int16_t IMUCalibration::applyMovingAverage(int16_t newValue, int16_t* buffer) {
    buffer[buffer_index] = newValue;
    
    long sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += buffer[i];
    }
    
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    return sum / FILTER_SIZE;
} 
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

// holds all 6 parsed sensor values after each read
// int16_t because sensor values are signed (-32768 to +32767)
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MpuData;

bool mpu6050_init();
bool mpu6050_read(MpuData *data);

#endif
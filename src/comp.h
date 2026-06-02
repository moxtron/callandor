#pragma once
#include "mpu6050.h"

// buffer for the sum of mpuData added together from 1000 mpu reads
// used to calculate the average 
typedef struct {
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
} sumValesBuf;

bool calibrateBias(MpuData *data);
bool filter(MpuData * data, MpuData bias_values, float * pitch_angle, float * roll_angle);
extern float c_atan2f(float y, float x);
#define atan2f c_atan2f
extern void printNum(int32_t);
extern void print(char *);

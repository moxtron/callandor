#ifndef COMP_H
#define COMP_H

#include "mpu6050.h"
#include <stdbool.h>
#include <stdint.h>
#include "fixedPoint.h"


bool calibrateBias(MpuData *data);
bool filter(MpuData * data, MpuData bias_values, float * pitch_angle, float * roll_angle);
extern float c_atan2f(float y, float x);
#define atan2f c_atan2f
extern void printNum(int32_t);
extern void print(char *);
#endif
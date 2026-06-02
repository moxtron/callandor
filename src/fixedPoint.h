#pragma once
#include "mpu6050.h"
typedef int32_t q23_8_t;


q23_8_t make_q(int32_t integer_part, int32_t fractial_part);
void print_q(q23_8_t x);

q23_8_t subtractQ23_8(q23_8_t minuend, q23_8_t subtrahend);
q23_8_t addQ23_8(q23_8_t addend, q23_8_t added1);
q23_8_t multiplyQ23_8(q23_8_t multiplicand, q23_8_t multiplier);
q23_8_t divide(q23_8_t dividend, q23_8_t divisor);
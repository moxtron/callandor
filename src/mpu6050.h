#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Zig binding, replaces pico/stdlib.h sleep_ms
void sleep_ms(uint32_t ms);
int32_t mpu_i2c_write(uint8_t addr, const uint8_t *data, size_t len);
int32_t mpu_i2c_write_then_read(
    uint8_t addr,
    const uint8_t *write_data,
    size_t write_len,
    uint8_t *read_data,
    size_t read_len
);

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


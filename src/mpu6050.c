#include "mpu6050.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"


// I2C bus configuration
// GP14 = SDA1, GP15 = SCL1 → both belong to i2c1 controller
#define BAUDRATE        400000  // 400kHz fast mode, max supported by MPU6050
#define SDA_PIN         14
#define SCL_PIN         15
#define I2C_INSTANC_RD    i2c1

// MPU6050 I2C bus address (AD0 pin = LOW)
// if AD0 pin is pulled HIGH use 0x69 instead
#define MPU6050_ADDRESS 0x68

// MPU6050 register addresses
#define PWR_MGMT_1_ADDRESS  0x6B  // power management — chip boots in sleep mode
#define SMPRT_DIV_ADDRESS   0x19  // sample rate divider
#define DLPF_CNF_ADDRESS    0x1A  // digital low pass filter config
#define FS_SEL_ADDRESS      0x1B  // gyroscope full scale range
#define AFS_SEL_ADDRESS     0x1C  // accelerometer full scale range
#define SENSOR_DATA_ADDRESS 0x3B  // first register of 14 byte sensor data block
                                  // auto increments through 0x3b to 0x48

// MPU6050 register values
// PWR_MGMT_1: 0x00 = wake up chip, use internal clock
#define PWR_MGMT_1_VALUE    0x00

// SMPLRT_DIV: 0 = 1kHz sample rate (1000 / (1 + 0) = 1000Hz)
// only valid when DLPF is enabled (DLPF_CFG 1-6)
#define SMPRT_DIV_VALUE     0x00

// DLPF_CFG: 2 = 98Hz bandwidth, 2.8ms delay
// good balance between noise filtering and response speed
#define DLPF_CNF_VALUE      0x02

// FS_SEL: bits 4:3 = 01 = ±500°/s range
// covers all normal and aggressive fixed wing maneuvers
// scale factor = 500/32768 = 0.01526 deg/s per raw unit
#define FS_SEL_VALUE        (1 << 3)  // = 0x08

// AFS_SEL: bits 4:3 = 01 = ±4g range
// good resolution for detecting 1g gravity direction
// covers all normal fixed wing maneuvers
// scale factor = 4/32768 = 0.000122 g per raw unit
#define AFS_SEL_VALUE       (1 << 3)  // = 0x08

// returns true if write succeeded, false if failed
// static because it's just an internal helper function
static bool i2c_write_config(const uint8_t *config, int length) {
    int result = i2c_write_blocking(I2C_INSTANC_RD, MPU6050_ADDRESS, config, length, false);
    return result == length;
}

bool mpu6050_init() {
    // initialize I2C1 controller at 400kHz
    i2c_init(I2C_INSTANC_RD, BAUDRATE);

    // connect GP14 and GP15 to I2C1 hardware controller
    // internally disconnects pins from GPIO and routes them to I2C peripheral
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // read register 0x75 — always returns 0x68 if chip is working
    uint8_t who_am_i_reg = 0x75;
    uint8_t who_am_i_val = 0;
    i2c_write_blocking(I2C_INSTANC_RD, MPU6050_ADDRESS, &who_am_i_reg, 1, true);
    i2c_read_blocking(I2C_INSTANC_RD, MPU6050_ADDRESS, &who_am_i_val, 1, false);

    if (who_am_i_val != 0x68) {
    // chip not found — wrong wiring, wrong address, broken chip
        return false;
    }

    // each config array: [register_address, value_to_write]
    // i2c_write_blocking sends both bytes in one I2C transaction

    // wake up MPU6050 — must be done first, chip ignores all other
    // register writes while in sleep mode
    uint8_t wake_config[2] = {PWR_MGMT_1_ADDRESS, PWR_MGMT_1_VALUE};

    // set sample rate to 1kHz — new sensor data every 1ms
    uint8_t smprt_div_config[2] = {SMPRT_DIV_ADDRESS,  SMPRT_DIV_VALUE};

    // configure low pass filter — removes motor vibration noise
    // from gyro and accelerometer readings before they reach the RP2040
    uint8_t dlpf_cnf_config[2]  = {DLPF_CNF_ADDRESS,   DLPF_CNF_VALUE};

    // set gyroscope range to ±500 deg/s
    // bits 4:3 of register 0x1B control the range
    uint8_t fs_sel_config[2]    = {FS_SEL_ADDRESS,      FS_SEL_VALUE};

    // set accelerometer range to ±4g
    // bits 4:3 of register 0x1C control the range
    uint8_t afs_sel_config[2]   = {AFS_SEL_ADDRESS,     AFS_SEL_VALUE};

    // send each config to MPU6050 over I2C
    // false = send STOP condition after each write (normal complete transaction)
    if (!i2c_write_config(wake_config, 2))      return false;
    // if registers are written to while MPU6050 still sleeps writes are ignored
    sleep_ms(100);
    if (!i2c_write_config(smprt_div_config, 2)) return false;
    if (!i2c_write_config(dlpf_cnf_config, 2))  return false;
    if (!i2c_write_config(fs_sel_config, 2))    return false;
    if (!i2c_write_config(afs_sel_config, 2))   return false;

    return true;
}
bool mpu6050_read(MpuData * data){
    // error checking
    int result = 0;
    // buffer to hold all 14 bytes of raw sensor data
    static uint8_t read_buffer[14];
    // register address to start reading from
    uint8_t reg = SENSOR_DATA_ADDRESS;
    // tell MPU6050 which register to start reading from (0x3B)
    // true = nostop, keeps I2C bus open (no stop condition sent)
    // this is required for a repeated start read sequence:
    // without nostop the chip would release the bus and lose the
    // register pointer before it can start reading
    result = i2c_write_blocking(I2C_INSTANC_RD, MPU6050_ADDRESS, &reg, 1, true);
    if(result != 1){
        return false;
    }
    // read 14 bytes starting from register 0x3B
    // MPU6050 auto increments through registers 0x3B to 0x48:
    // bytes 0-1:   accel X  (high byte first, then low byte)
    // bytes 2-3:   accel Y
    // bytes 4-5:   accel Z
    // bytes 6-7:   temperature (not used for flight control)
    // bytes 8-9:   gyro X
    // bytes 10-11: gyro Y
    // bytes 12-13: gyro Z
    // false = send stop condition after read
    result = i2c_read_blocking(I2C_INSTANC_RD, MPU6050_ADDRESS, read_buffer, 14, false);
    if(result != 14){
        return false;
    }
    // combine high and low bytes into signed 16-bit integers
    // cast to int16_t to preserve sign (values range -32768 to +32767)
    data->accel_x = (int16_t) (read_buffer[0] << 8  |  read_buffer[1]);
    data->accel_y = (int16_t) (read_buffer[2] << 8  |  read_buffer[3]);
    data->accel_z = (int16_t) (read_buffer[4] << 8  |  read_buffer[5]);
    data->gyro_x  = (int16_t) (read_buffer[8] << 8  |  read_buffer[9]);
    data->gyro_y  = (int16_t) (read_buffer[10] << 8 | read_buffer[11]);
    data->gyro_z  = (int16_t) (read_buffer[12] << 8 | read_buffer[13]);

    return true;

}

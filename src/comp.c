#include "comp.h"

#define TIMER_BASE      0x40054000
#define TIMER_OFFSET    0x0C
#define GYRO_SCALE      (500.0 / 32768)
#define ACCEL_SCALE     (4.0 / 32768)
#define RAD_TO_DEG      57.2958 // 180 / PI


// gets current time in us
uint32_t getTimeUs(){
    volatile uint32_t* time_lr = (volatile uint32_t *) (TIMER_BASE+TIMER_OFFSET);
    return *time_lr;
}
// run once before flight to get bias values
// plane needs to be in a position where 
// accel_x = 0, accel_y = 0, accel_z = 1
bool calibrateBias(MpuData * data){
    // gets time of calibration start
    const uint32_t calibration_start = getTimeUs();
    // gets time since last mpuRead, Will set
    // to zero because this funtion will be first to 
    // read mpu6050 value and therefore wont read a value
    // twice
    uint32_t last_sample_time = calibration_start; 
    // initialises empty buffer which will hold the sum of all values
    sumValesBuf sum_values_buf = {0};

    // counting how many times values got updated
    uint16_t counter = 0;
    while (true) {
        
        // current time
        uint32_t now = getTimeUs();
        // if current time measurement and time measurement of 
        // last correction is at least 1ms apart
        if(now - last_sample_time >= 1000){
            // get the data from mpu6050
            if(mpu6050_read(data)){
                // update last_sample_time value
                last_sample_time = now;
                // inc counter
                counter++;
                
                // sum all values in the buffer to average
                sum_values_buf.accel_x += data->accel_x;
                sum_values_buf.accel_y += data->accel_y;
                sum_values_buf.accel_z += data->accel_z;
                sum_values_buf.gyro_x += data->gyro_x;
                sum_values_buf.gyro_y += data->gyro_y;
                sum_values_buf.gyro_z += data->gyro_z;
            }
            else {
            // return false if something went wrong
            // in the reading process 
            return false;
            }
        }
        // break while true loop after one second and finish 
        // the calibration process
        if((now - calibration_start) > 1000000){
            break;
        }
    }
    // ensuring no division by 0
    if(counter == 0) return false;
    // average all values by dividing by counter
    data->accel_x = (int16_t) (sum_values_buf.accel_x / counter);
    data->accel_y = (int16_t) (sum_values_buf.accel_y / counter);
    data->accel_z = (int16_t) (sum_values_buf.accel_z / counter);
    data->gyro_x = (int16_t) (sum_values_buf.gyro_x / counter);
    data->gyro_y = (int16_t) (sum_values_buf.gyro_y / counter);
    data->gyro_z = (int16_t) (sum_values_buf.gyro_z / counter);
 
    return true;

}
// must init pitch and roll angle with 0 before first call
bool filter(MpuData * data, MpuData bias_values,  float * pitch_angle, float * roll_angle){   
    // handling first init of last_filtered_time
    // on first call
    static uint32_t last_filtered_time = 0;
    static bool first_call = true;
    if(first_call){
        last_filtered_time = getTimeUs();
        first_call = false;
        return true;
    }

    uint32_t now = getTimeUs();
    // 1ms = 1000 seconds
    const float dt = (float) (now - last_filtered_time) / 1000000.0f;
    // update last filtered time before funtion can return false
    last_filtered_time = now;
    // gyro x corrected level
    const float corrected_gx = (float)data->gyro_x - (float)bias_values.gyro_x;
    // gyro y corrected level 
    const float corrected_gy = (float)data->gyro_y - (float)bias_values.gyro_y;
    // gyro z correction level
    const float corrected_gz = (float)data->gyro_z - (float)bias_values.gyro_z;
    // accel x corrected level
    const float corrected_ax = ((float) data->accel_x - (float) bias_values.accel_x);
    // accel y corrected level
    const float corrected_ay = ((float) data->accel_y - (float) bias_values.accel_y);    // offset of accelerometer z axis from 8192 or 1g
    const int16_t accel_z_offset = 8192 - bias_values.accel_z;
    // accel z corrected level
    const int16_t corrected_az = data->accel_z + accel_z_offset;
    // ensuring no division by 0
    if(bias_values.accel_z == 0) return false;
    // accel z in g level
    // should be corrected_az / 8192
    const float az_g = (float) data->accel_z / bias_values.accel_z;

    // ensuring no division by 0
    if(az_g < 0.01f && az_g > -0.01f) return false;
    // angle change per second x axis
    const float gyro_roll = (*roll_angle) + (corrected_gx * GYRO_SCALE) * dt; 
    // angle change per secnod y axis
    const float gyro_pitch = (*pitch_angle) + (corrected_gy * GYRO_SCALE) * dt;
    // real angle measured from accelerometer on x axis
    const float accel_roll = atan2f((corrected_ax * ACCEL_SCALE), az_g) * RAD_TO_DEG;
    // real angle measured from accelerometer on y axis
    const float accel_pitch = atan2f((corrected_ay * ACCEL_SCALE), az_g) * RAD_TO_DEG;
    // updated roll angle
    *roll_angle = (0.98f * gyro_roll + 0.02f * accel_roll);
    // updated pitch angle
    *pitch_angle = (0.98f * gyro_pitch + 0.02f * accel_pitch);
    
    // TODO: update data struct to improved values
    data->accel_x = (int16_t) corrected_ax;
    data->accel_y = (int16_t) corrected_ay;
    data->accel_z = (int16_t) corrected_az;
    data->gyro_x = (int16_t) corrected_gx;
    data->gyro_y = (int16_t) corrected_gy;
    data->gyro_z = (int16_t) corrected_gz;
    return true;
}
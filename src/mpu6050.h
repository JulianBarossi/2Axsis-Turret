#ifndef _MPU6050_
#define _MPU6050_

#include <stdint.h>

class mpu6050
{
private:
    uint8_t address;
    float accel_scale_factor = 16384.0f;
    float gyro_scale_factor = 131.072;

    //raw data processed
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    //fine tune later
    float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f;
    float ax_offset = 0.0f, ay_offset = 0.0f, az_offset = 0.0f;
    
    //holds calculated servo change
    float pitch = 0, roll = 0, yaw = 0;
    float baseline_pitch = 0.0f, baseline_yaw, baseline_roll = 0.0f;
    
    //raw values from mpu
    int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
    int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;

    //
    float gyro_alpha = 0.98f;
    float accel_alpha = 0.20f;

    bool aFirst =true;
    bool gFrist = true;


public:
    void init_mpu();
    bool getRaw();
    void raw_to_g();
    void getAcclAngles();
    void calibrate();
    bool update(float delta_t);
    float getPitch();
    float getRoll();
    float getYaw();

    //for telemtry
    float getAx() const { return ax; }
    float getAy() const { return ay; }
    float getAz() const { return az; }
    float getGx() const { return gx; }
    float getGy() const { return gy; }
    float getGz() const { return gz; }

    mpu6050(uint8_t add = 0x68) : address(add){};
};


#endif
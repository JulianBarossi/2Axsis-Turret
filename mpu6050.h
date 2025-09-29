#ifndef _MPU6050_
#define _MPU6050_

#include <stdint.h>

class mpu6050
{
private:
    uint8_t address;
    uint16_t scale_factor = 16384;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float x_offset = 0.0f, y_offset = 0.0f, z_offset = 0.0f;
    float pitch = 0, roll = 0;
    float baseline_pitch = 0.0f, baseline_roll = 0.0f;
    uint16_t raw_x = 0, raw_y = 0, raw_z = 0;


public:
    void init_mpu();
    bool getRaw();
    void raw_to_g();
    void getAngles();
    void zero();
    bool update();
    float getPitch();
    float getRoll();

    mpu6050(uint8_t add = 0x68) : address(add){};
};


#endif
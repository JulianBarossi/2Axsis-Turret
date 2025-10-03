/*
Julian Barossi
SJSU Robotics Trial
09/28/2025
*/

#include "mpu6050.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>


void mpu6050::init_mpu(){
    Wire.begin();
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(address);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission(true);
}

bool mpu6050::getRaw() {
    Wire.beginTransmission(address);
    Wire.write(0x3B);   
    if (Wire.endTransmission(false) != 0) return false; // NACK/ACK check

    Wire.requestFrom(address, 14);
    if (Wire.available() < 14) return false;

    raw_ax = (Wire.read() << 8) | Wire.read();
    raw_ay = (Wire.read() << 8) | Wire.read();
    raw_az = (Wire.read() << 8) | Wire.read();

    (void)Wire.read();  //skip 0x40 and 0x41
    (void)Wire.read();

    raw_gx = (Wire.read() << 8) | Wire.read();
    raw_gy = (Wire.read() << 8) | Wire.read();
    raw_gz = (Wire.read() << 8) | Wire.read();

    return true;
}


void mpu6050::raw_to_g(){


float ax_nofilter = (raw_ax/ accel_scale_factor) - ax_offset;
float ay_nofilter = (raw_ay / accel_scale_factor) - ay_offset;
float az_nofilter = (raw_az / accel_scale_factor) - az_offset;

if (aFirst){
    ax = ax_nofilter;
    ay = ay_nofilter;
    az = az_nofilter;
    aFirst = false;
}else{
    
    ax = (1 - accel_alpha) * ax + (ax_nofilter * accel_alpha);
    ay = (1 - accel_alpha) * ay + (ay_nofilter * accel_alpha);
    az = (1 - accel_alpha) * az + (az_nofilter * accel_alpha);
}

gx = (raw_gx / gyro_scale_factor) - gx_offset;
gy = (raw_gy / gyro_scale_factor) - gy_offset;
gz = (raw_gz / gyro_scale_factor) - gz_offset;

}

void mpu6050::getAcclAngles(){
    pitch = (atan2(-ax, sqrt(ay*ay+az*az)) * 180 / PI )- baseline_pitch;
    roll = (atan2(ay, az) * 180 / PI )- baseline_roll;
}


void mpu6050::calibrate(){

    
    const int intervals = 200;
    float sx=0, sy=0, sz=0;
    for(int i=0; i<intervals; ++i){
        if(!getRaw()) continue;
        raw_to_g();             
        sx += gx; sy += gy; sz += gz;
        delay(5);
    }
    gx_offset = sx / intervals;
    gy_offset = sy / intervals;
    gz_offset = sz / intervals;

    //tare
    baseline_pitch = pitch;
    baseline_roll  = roll;

}

bool mpu6050::update(float delta_t){
    if(!getRaw()) return false;

    raw_to_g();

    float accel_pitch = (atan2(-ax, sqrt(ay*ay+az*az)) * 180 / PI );
    float accel_roll = (atan2(ay, az) * 180 / PI );
    
    //swap
    float temp = accel_roll;
    accel_roll = accel_pitch;
    accel_pitch = temp;

    if(gFrist){
        roll = accel_roll;
        pitch = accel_pitch;
        gFrist = false;
    }

    yaw += gz * delta_t;
    roll = gyro_alpha * (roll + gy * delta_t) + (1.0f - gyro_alpha) * accel_roll;
    pitch = gyro_alpha * (pitch + gx * delta_t) + (1.0f - gyro_alpha) * accel_pitch;

    return true;
}

float mpu6050::getPitch(){
    return pitch - baseline_pitch;
}

float mpu6050::getRoll(){
    return roll - baseline_roll;
}

float mpu6050::getYaw(){
    return yaw - baseline_yaw;
}
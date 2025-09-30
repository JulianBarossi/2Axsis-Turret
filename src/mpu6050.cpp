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

    Wire.requestFrom(address, 6);
    if (Wire.available() < 6) return false;

    raw_x = (Wire.read() << 8) | Wire.read();
    raw_y = (Wire.read() << 8) | Wire.read();
    raw_z = (Wire.read() << 8) | Wire.read();
    return true;
}


void mpu6050::raw_to_g(){
static bool first = true;

float x_nofilter = (raw_x/ scale_factor) - x_offset;
float y_nofilter = (raw_y / scale_factor) - y_offset;
float z_nofilter = (raw_z / scale_factor) - z_offset;

if (first){
    x = x_nofilter;
    y = y_nofilter;
    z = z_nofilter;
    first = false;
}else{
    const float alpha = 0.20f;

    x = (1 - alpha) * x + (x_nofilter * alpha);
    y = (1 - alpha) * y + (y_nofilter * alpha);
    z = (1 - alpha) * z + (z_nofilter * alpha);
}

}

void mpu6050::getAngles(){
    pitch = (atan2(-x, sqrt(y*y+z*z)) * 180 / PI )- baseline_pitch;
    roll = (atan2(y, z) * 180 / PI )- baseline_roll;
}


void mpu6050::zero(){
    baseline_pitch = pitch;
    baseline_roll = roll; 
}

bool mpu6050::update(){
    if(!getRaw()) return false;
    raw_to_g();
    getAngles();
    return true;
}

float mpu6050::getPitch(){
    return pitch;
}

float mpu6050::getRoll(){
    return roll;
}
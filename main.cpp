/*
Julian Barossi
SJSU Robotics Trial
09/28/2025
*/

#include <Arduino.h>
#include "mpu6050.h"
#include "myServo.h"

mpu6050 mpu;
My_servo pitch(9,0,180,90);
My_servo roll(10,0,180,90);

void setup() {
  Serial.begin(115200);

  mpu.init_mpu();
  pitch.init_servo();
  roll.init_servo();

  if(mpu.update()){
    mpu.zero();
    Serial.println("TARED MPU");
  }
}

void loop() {
  if(mpu.update()){
    Serial.print("Pitch: ");
    Serial.print(mpu.getPitch());
    Serial.print("    Roll:");
    Serial.print(mpu.getRoll());

    int roll_angle = map((int)mpu.getRoll(), -90, 90, 0, 180);
    int pitch_angle = map((int)mpu.getPitch(), -90, 90, 0, 180);

    pitch.write(pitch_angle);
    roll.write(roll_angle);
  }
  delay(30);
}


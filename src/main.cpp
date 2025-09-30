/*
Julian Barossi
SJSU Robotics Trial
09/28/2025
*/

#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"
#include "myServo.h"
#define MAIN

#ifdef MAIN
mpu6050 mpu;
My_servo pitch(9,0,180,90);
// My_servo roll(10,0,180,90); 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.init_mpu();
  pitch.init_servo();
  //roll.init_servo();

  if(mpu.update()){
    mpu.zero();
    Serial.println("TARED MPU");
  }
}

void loop() {

  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if(mpu.update()){
    

    //int roll_angle = map((int)mpu.getRoll(), -90, 90, 0, 180);
    int pitch_angle = map((int)mpu.getPitch(), -90, 90, 0, 180);

    pitch.write(pitch_angle);
    //roll.write(roll_angle);

    if (now - lastPrint >= 1000){
      lastPrint = now;
      Serial.print("Pitch: ");
      Serial.print(mpu.getPitch());
      Serial.print("    Roll:");
      Serial.println(mpu.getRoll());
    }
  }
  delay(50);
}

#endif

#ifdef TEST1

void setup(){
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("MPU Test");
}

void loop(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Serial.print("ax="); Serial.print(ax);
  Serial.print(" | ay="); Serial.print(ay);
  Serial.print(" | az="); Serial.println(az);

  delay(500);
}

#endif

#ifdef Read

uint8_t addr = 0x68; 

uint8_t i2c_read8(uint8_t a, uint8_t reg) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF; // error
  if (Wire.requestFrom(a, (uint8_t)1, (uint8_t)true) != 1) return 0xFE; // short read
  return Wire.read();
}

bool i2c_write8(uint8_t a, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();              // IMPORTANT
  delay(200);

  Serial.println("MPU Test");

  uint8_t addresses[] = {0x68, 0x69};
for (uint8_t i = 0; i < 2; i++) {
    uint8_t tryAddr = addresses[i];
    Wire.beginTransmission(tryAddr);
    int e = Wire.endTransmission();
    Serial.print("Probe 0x"); Serial.print(tryAddr, HEX);
    Serial.print(" -> "); Serial.println(e == 0 ? "ACK" : "NACK");
    if (e == 0) addr = tryAddr;
}


  // WHO_AM_I should be 0x68
  uint8_t who = i2c_read8(addr, 0x75);
  Serial.print("WHO_AM_I @0x"); Serial.print(addr, HEX);
  Serial.print(" = 0x"); Serial.println(who, HEX);

  // Wake device (PWR_MGMT_1 = 0)
  if (!i2c_write8(addr, 0x6B, 0x00)) {
    Serial.println("Failed to write PWR_MGMT_1");
  } else {
    Serial.println("Woke MPU6050");
  }
}

void loop() {
  // Request 6 accel bytes with repeated start
  Wire.beginTransmission(addr);
  Wire.write(0x3B);                         // ACCEL_XOUT_H
  int e = Wire.endTransmission(false);      // repeated start
  if (e != 0) {
    Serial.print("endTx error: "); Serial.println(e);
    delay(500);
    return;
  }

  int n = Wire.requestFrom(addr, (uint8_t)6, (uint8_t)true);
  if (n != 6) {
    Serial.print("short read: "); Serial.println(n);
    delay(500);
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Serial.print("ax="); Serial.print(ax);
  Serial.print(" | ay="); Serial.print(ay);
  Serial.print(" | az="); Serial.println(az);

  delay(300);
}

#endif

#ifdef i2c
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();                 // IMPORTANT
  Wire.setClock(100000);        // 100kHz

  Serial.println("I2C scanner starting...");
}

void loop() {
  byte count = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
      count++;
    } else if (err == 4) {
      Serial.print("Unknown error at 0x");
      Serial.println(addr, HEX);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found. Check wiring/power/GND.");
  } else {
    Serial.print("Done. Devices found: ");
    Serial.println(count);
  }

  Serial.println("---- rescanning in 2s ----");
  delay(2000);
}

#endif






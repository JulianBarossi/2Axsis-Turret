/*
Julian Barossi
SJSU Robotics Trial
09/28/2025
*/

#include "myServo.h"
#include <Arduino.h>
#include <Servo.h>


void My_servo::init_servo(){
    driver.attach(pin);
    driver.write(start_degrees);
    current_degrees = start_degrees;
}

bool My_servo::write(int degree){
    if(degree < minLimit) degree = minLimit;
    if(degree > maxLimit) degree = maxLimit;

    current_degrees = degree;
    driver.write(current_degrees);

    if((current_degrees == minLimit) || (current_degrees) == maxLimit){
        return false;
    }
    return true;
}
#ifndef _MY_SERVO_
#define _MY_SERVO_

#include <stdint.h>
#include <Servo.h>


class My_servo{

    private:
        uint8_t pin;
        int start_degrees;
        int current_degrees;
        int minLimit;
        int maxLimit;
        Servo driver;

    public:
    
    My_servo(int pin, uint8_t min = 0, uint8_t max = 180, uint8_t start = 90) : pin(pin), minLimit(min), maxLimit(max), start_degrees(start), current_degrees(start){};
    void init_servo();
    bool write(int degree);
};



#endif
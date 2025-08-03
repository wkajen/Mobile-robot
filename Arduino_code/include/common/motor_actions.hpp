#ifndef MOTOR_ACTIONS_HPP
#define MOTOR_ACTIONS_HPP

#include <Arduino.h>
#include "config.hpp"

class Motor
{
    public:
        // konstruktor klasy
        Motor(int enable_pin, int forward_pin, int backward_pin, int encoder_pin)
                : en_pin(enable_pin), for_pin(forward_pin), back_pin(backward_pin), encoder_pin(encoder_pin) {};

        void spinForward(int pwm_speed);
        void spinBackward(int pwm_speed);
        void stop();

    private:
        int en_pin;
        int for_pin;
        int back_pin;
        int encoder_pin;
};

void motorControl(Motor& motor, double pid_output);

#endif // MOTOR_ACTIONS_HPP

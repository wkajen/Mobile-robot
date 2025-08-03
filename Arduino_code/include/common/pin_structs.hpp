#ifndef PIN_STRUCTS_HPP
#define PIN_STRUCTS_HPP

#include <Arduino.h>

// struktura dla pinów silnika
struct DCmotorPin
{
    const int PWM;
    const int forward;
    const int backward;

    // Konstruktor
    DCmotorPin(int pwm, int forward, int backward)
        : PWM(pwm), forward(forward), backward(backward) {}
};
extern DCmotorPin motor_left_rear_pins;
extern DCmotorPin motor_right_rear_pins;
extern DCmotorPin motor_left_front_pins; 
extern DCmotorPin motor_right_front_pins;

// struktura dla pinów czujnika odległości
struct DistanceSensor
{
    const int trig_pin;
    const int echo_pin;

    // Konstruktor
    DistanceSensor(int trig, int echo) : trig_pin(trig), echo_pin(echo) {}
};
extern DistanceSensor dist_sensor;

// piny enkoderów
extern const int encoder_left_rear_pin;
extern const int encoder_right_rear_pin; 
extern const int encoder_left_front_pin;
extern const int encoder_right_front_pin; 

#endif // PIN_STRUCTS_HPP

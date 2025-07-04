#include <Arduino.h>
#include <functions.hpp>
#include <pin_structs.hpp>
#include <config.hpp>

void pinOutSetting()
{
    // wejścia ----------------------------------
    // sekcja wejść z enkoderów
    pinMode(encoder_left_rear_pin, INPUT_PULLUP);
    pinMode(encoder_right_rear_pin, INPUT_PULLUP);
    pinMode(encoder_left_front_pin, INPUT_PULLUP);
    pinMode(encoder_right_front_pin, INPUT_PULLUP);

    // sekcja wejść z czujnika odległości
    pinMode(dist_sensor.trig_pin, OUTPUT);
    pinMode(dist_sensor.echo_pin, INPUT);
    // koniec wejść -----------------------------

    // wyjścia ----------------------------------
    // sekcja wyjść na silniki 
    pinMode(motor_left_rear_pins.PWM, OUTPUT);
    pinMode(motor_left_rear_pins.forward, OUTPUT);
    pinMode(motor_left_rear_pins.backward, OUTPUT);

    pinMode(motor_right_rear_pins.PWM, OUTPUT);
    pinMode(motor_right_rear_pins.forward, OUTPUT);
    pinMode(motor_right_rear_pins.backward, OUTPUT);

    pinMode(motor_left_front_pins.PWM, OUTPUT);
    pinMode(motor_left_front_pins.forward, OUTPUT);
    pinMode(motor_left_front_pins.backward, OUTPUT);

    pinMode(motor_right_front_pins.PWM, OUTPUT);
    pinMode(motor_right_front_pins.forward, OUTPUT);
    pinMode(motor_right_front_pins.backward, OUTPUT);
    // koniec wyjść -------------------------------
    
    // ustawienie domyślnych wartości na LOW
    digitalWrite(motor_left_rear_pins.forward, LOW);
    digitalWrite(motor_left_rear_pins.backward, LOW);

    digitalWrite(motor_right_rear_pins.forward, LOW);
    digitalWrite(motor_right_rear_pins.backward, LOW);

    digitalWrite(motor_left_front_pins.forward, LOW);
    digitalWrite(motor_left_front_pins.backward, LOW);

    digitalWrite(motor_right_front_pins.forward, LOW);
    digitalWrite(motor_right_front_pins.backward, LOW);
}

int getDistance() 
{
    long time; 
    
    digitalWrite(dist_sensor.trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(dist_sensor.trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(dist_sensor.trig_pin, LOW);

    time = pulseIn(dist_sensor.echo_pin, HIGH);
    return (int) (time / 58);  // działanie, aby otrzymać wynik w [cm]  
}

void countPulses(int encoder_pin, unsigned int &pulses_num, bool &last_state) 
{
	bool current_state = digitalRead(encoder_pin);

    // Zliczanie tylko zbocza narastającego (LOW -> HIGH)
    if (current_state == HIGH && last_state == LOW) 
    {
        pulses_num++;
    }
    last_state = current_state;
}

void send_data(double left_front_speed, double right_front_speed, double left_rear_speed, double right_rear_speed, double distance) 
{
    Serial.print("S:");
    Serial.print(left_front_speed);
    Serial.print(",");
    Serial.print(right_front_speed);
    Serial.print(",");
    Serial.print(left_rear_speed);
    Serial.print(",");
    Serial.print(right_rear_speed);
    Serial.print(",");
    Serial.print(distance);
    Serial.println();
}

// void send_data(double distance)
// {
//     Serial.print("D:");
//     Serial.print(distance);
//     Serial.println();
// }
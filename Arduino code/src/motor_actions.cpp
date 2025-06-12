#include <motor_actions.hpp>

void Motor::spinForward(int pwm_speed)
{
	digitalWrite(Motor::for_pin, HIGH);
	digitalWrite(Motor::back_pin, LOW);

	// wartości PWM mogą być wyłącznie z zakresu 0 do 255
	if (pwm_speed < 0)  
	{
		analogWrite(Motor::en_pin, 0);
	}
	else if (pwm_speed > 255)
	{
		analogWrite(Motor::en_pin, 255);
	}
	else
	{
		analogWrite(Motor::en_pin, pwm_speed);
	}
}

void Motor::spinBackward(int pwm_speed)
{
	digitalWrite(Motor::for_pin, LOW);
	digitalWrite(Motor::back_pin, HIGH);

	// wartości PWM mogą wyłącznie z zakresu 0 do 255
	if (pwm_speed < 0)  
	{
		analogWrite(Motor::en_pin, 0);
	}
	else if (pwm_speed > 255)
	{
		analogWrite(Motor::en_pin, 255);
	}
	else
	{
		analogWrite(Motor::en_pin, pwm_speed);
	}
}

void Motor::stop()
{
	digitalWrite(for_pin, LOW);
	digitalWrite(back_pin, LOW);
	analogWrite(en_pin, 0);
}

void motorControl(Motor& motor, double pid_output) 
{
    int pwm_val = (int)abs(pid_output);
    int pwm_val_scaled = constrain(pwm_val * speed_scaling, 0, 255);
    if (pwm_val_scaled > 5 && pwm_val_scaled < 45)
	{
        pwm_val_scaled = 45;
	}
    else if (pwm_val_scaled <= 5)
	{
        pwm_val_scaled = 0;
	}

    if (pid_output > 0)
        motor.spinForward(pwm_val_scaled);
	else if (pid_output < 0)
		motor.spinBackward(pwm_val_scaled);
    else
        motor.stop();
}
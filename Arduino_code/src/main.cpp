#include <Arduino.h>
#include <PID_v1.h>
#include <MQ2.h>
#include "common/pin_structs.hpp"
#include "config.hpp"
#include "common/motor_actions.hpp"
#include "common/functions.hpp"

// struktury do przechowywania wartości pinów dla każdego silnika,
// jako argumenty przyjmują w kolejności: pin PWM, pin jazdy do przodu, pin dla jazdy do tyłu
DCmotorPin motor_left_rear_pins(9, 11, 8);
DCmotorPin motor_right_rear_pins(10, 12, 13);
DCmotorPin motor_left_front_pins(6, 5, 7);
DCmotorPin motor_right_front_pins(3, 2, 4);

// piny czujnika odległości
DistanceSensor dist_sensor(A5, A4);

// piny enkoderów
const int encoder_left_rear_pin = A0;
const int encoder_right_rear_pin = A1;
const int encoder_left_front_pin = A2;
const int encoder_right_front_pin = A3;

// pin czujnika dymu i gazów
const int mq2_pin = A7;

// zmienne wyjściowe czujnika
MQ2 mq2(mq2_pin);
float lpg{0.0};
float co{0.0}; 
float smoke{0.0};

// utworzenie obiektu klasy Motor dla każdego z czterech silników
Motor motor_left_rear(motor_left_rear_pins.PWM, motor_left_rear_pins.forward, motor_left_rear_pins.backward, encoder_left_rear_pin);
Motor motor_right_rear(motor_right_rear_pins.PWM, motor_right_rear_pins.forward, motor_right_rear_pins.backward, encoder_right_rear_pin);
Motor motor_left_front(motor_left_front_pins.PWM, motor_left_front_pins.forward, motor_left_front_pins.backward, encoder_left_front_pin);
Motor motor_right_front(motor_right_front_pins.PWM, motor_right_front_pins.forward, motor_right_front_pins.backward, encoder_right_front_pin);

// utworzenie struktur dla przechowywania wartości różnych prędkości dla każdego silnika
struct every_motor
{
	double left_rear{0.0};
	double right_rear{0.0};
	double left_front{0.0};
	double right_front{0.0};
};

struct motor_speed_s
{
	every_motor raw;
	every_motor out;
	every_motor set;
	every_motor smooth;
};

struct encoder_state_s
{
	bool left_rear{LOW};
	bool right_rear{LOW};
	bool left_front{LOW};
	bool right_front{LOW};
};

struct counter_s
{
	unsigned int left_rear{0};
	unsigned int right_rear{0};
	unsigned int left_front{0};
	unsigned int right_front{0};
};

const int avg_window = 5; 
struct buffer_s
{
	double left_rear[avg_window] = {0.0};
	double right_rear[avg_window] = {0.0};
	double left_front[avg_window] = {0.0};
	double right_front[avg_window] = {0.0};
};

motor_speed_s motor_speed;
encoder_state_s encoder_state;
counter_s counter;
buffer_s buffer_speed;

// nastawy PID
const double KP = 0.85, KI = 0.2, KD = 0.005;
// instancje PID, jako argumenety po kolei: wejście, wyjście, wartość zadana, Kp, Ki, Kd
PID leftRearSpeedPID(&motor_speed.smooth.left_rear, &motor_speed.out.left_rear, &motor_speed.set.left_rear, 0.65, 0.15, 0.01, DIRECT);
PID rightRearSpeedPID(&motor_speed.smooth.right_rear, &motor_speed.out.right_rear, &motor_speed.set.right_rear, KP, KI, KD, DIRECT);
PID leftFrontSpeedPID(&motor_speed.smooth.left_front, &motor_speed.out.left_front, &motor_speed.set.left_front, KP, KI, KD, DIRECT);
PID rightFrontSpeedPID(&motor_speed.smooth.right_front, &motor_speed.out.right_front, &motor_speed.set.right_front, KP, KI, KD, DIRECT);

// deklaracje lokalnych funkcji 
void apply_velocity(float v_x, float v_y, float w); 
void process_line(String line);
void receive_data();
// void send_data(double left_front_speed, double right_front_speed, double left_rear_speed, double right_rear_speed, int distance);

// zmienne globalne (ale widziane tylko w main)
unsigned int buff_idx = 0;
int distance{0};
unsigned long last_send_time{0};
unsigned long last_count_time{0};
unsigned long last_mq_read_time_ms{0};
unsigned long current_time{0};
String input_buffer = "";

void setup() 
{
	if (is_serial_on)
	{
		Serial.begin(115200); 
		Serial.println("READY");
	}
	// ustawienie typów (wejście, wyjście) dla I/O
	pinOutSetting(); 

	// zapisanie ustawień regulatora PID
	leftRearSpeedPID.SetSampleTime(pid_interval);
	rightRearSpeedPID.SetSampleTime(pid_interval);
	leftFrontSpeedPID.SetSampleTime(pid_interval);
	rightFrontSpeedPID.SetSampleTime(pid_interval);
  	leftRearSpeedPID.SetMode(AUTOMATIC);
	rightRearSpeedPID.SetMode(AUTOMATIC);
	leftFrontSpeedPID.SetMode(AUTOMATIC);
	rightFrontSpeedPID.SetMode(AUTOMATIC);

	// zmiana zakresu wartości na wyjściu PID
	leftRearSpeedPID.SetOutputLimits(-255, 255);
	rightRearSpeedPID.SetOutputLimits(-255, 255);
	leftFrontSpeedPID.SetOutputLimits(-255, 255);
	rightFrontSpeedPID.SetOutputLimits(-255, 255);

	mq2.begin();

	delay(250);
	last_send_time = last_count_time = millis();  // inicjalizacyjny pomiar czasu
	distance = getDistance();			// inicjalizacyjny pomiar odległości
	// Serial.println("ARDUINO_READY");	// sygnał gotowości dla RPi
}

void loop()
{
	current_time = millis();
	// double delta_time = (double)current_time - last_send_time;

	// zliczanie liczby imuplsów na każdym enkoderze
	countPulses(encoder_left_rear_pin, counter.left_rear, encoder_state.left_rear);
	countPulses(encoder_right_rear_pin, counter.right_rear, encoder_state.right_rear);
	countPulses(encoder_left_front_pin, counter.left_front, encoder_state.left_front);
	countPulses(encoder_right_front_pin, counter.right_front, encoder_state.right_front);

	// Odbiór danych
	while (Serial.available()) 
	{
		receive_data();
	}
	
	// wykonanie pomiarów z czujników w określonych odstępach czasu
	if(current_time - last_count_time >= count_interval_ms)
	{
		// przeliczenie ilości pulsów enkodera na prędkość liniową [cm/s]
		if (motor_speed.set.left_rear < -1.0)
			motor_speed.raw.left_rear = -(double)counter.left_rear * tick_mul * (1000.0/count_interval_ms);
		else if (motor_speed.set.left_rear > 1.0)
			motor_speed.raw.left_rear = (double)counter.left_rear * tick_mul * (1000.0/count_interval_ms);
		else
			motor_speed.raw.left_rear = 0.0;

		if (motor_speed.set.right_rear < -1.0)
			motor_speed.raw.right_rear = -(double)counter.right_rear * tick_mul * (1000.0/count_interval_ms);
		else if (motor_speed.set.right_rear > 1.0)
			motor_speed.raw.right_rear = (double)counter.right_rear * tick_mul * (1000.0/count_interval_ms); 
		else
			motor_speed.raw.right_rear = 0.0;

		if (motor_speed.set.left_front < -1.0)
			motor_speed.raw.left_front = -(double)counter.left_front * tick_mul * (1000.0/count_interval_ms); 
		else if (motor_speed.set.left_front > 1.0)
			motor_speed.raw.left_front = (double)counter.left_front * tick_mul * (1000.0/count_interval_ms); 
		else
			motor_speed.raw.left_front = 0.0;

		if (motor_speed.set.right_front < -1.0)
			motor_speed.raw.right_front = -(double)counter.right_front * tick_mul * (1000.0/count_interval_ms);
		else if (motor_speed.set.right_front > 1.0)
			motor_speed.raw.right_front = (double)counter.right_front * tick_mul * (1000.0/count_interval_ms); 
		else
			motor_speed.raw.right_front = 0.0;

		// wpisanie do bufora surowej prędkości
		buffer_speed.left_rear[buff_idx] = motor_speed.raw.left_rear;
		buffer_speed.right_rear[buff_idx] = motor_speed.raw.right_rear;
		buffer_speed.left_front[buff_idx] = motor_speed.raw.left_front;
		buffer_speed.right_front[buff_idx] = motor_speed.raw.right_front;

		motor_speed.smooth.left_rear = 0.0;
		motor_speed.smooth.right_rear = 0.0;
		motor_speed.smooth.left_front = 0.0;
		motor_speed.smooth.right_front = 0.0;
		// Obliczenie średniej prędkości z ostatnich 5 próbek
		for (int i = 0; i < avg_window; ++i)
		{
			motor_speed.smooth.left_rear += buffer_speed.left_rear[i];
			motor_speed.smooth.right_rear += buffer_speed.right_rear[i];
			motor_speed.smooth.left_front += buffer_speed.left_front[i];
			motor_speed.smooth.right_front += buffer_speed.right_front[i];
		}

		motor_speed.smooth.left_rear /= avg_window;
		motor_speed.smooth.right_rear /= avg_window;
		motor_speed.smooth.left_front /= avg_window;
		motor_speed.smooth.right_front /= avg_window;

		// inkrementacja indeksu bufora
		buff_idx = (buff_idx + 1) % avg_window;

		// wyzerowanie liczników pulsów
		counter.left_rear = counter.right_rear = counter.left_front = counter.right_front = 0;

		// pomiar odległości
		distance = getDistance();

		last_count_time = current_time;
	}

	// pomiary czujnika gazów w określonych odstępach czasowych
	if (current_time - last_mq_read_time_ms >= mq_interval_ms)
	{
		lpg = mq2.readLPG();
		co = mq2.readCO();
		smoke = mq2.readSmoke();

		last_mq_read_time_ms = current_time;
	}

	// wysyłanie danych do RPi w ustalonym interwale czasowym
	if(current_time - last_send_time >= send_interval_ms)
	{
		// wysłanie danych do RPi
		sendData(motor_speed.smooth.left_front, motor_speed.smooth.right_front, motor_speed.smooth.left_rear, motor_speed.smooth.right_rear, distance, lpg, co, smoke);
		last_send_time = current_time;
	}

	// hamowanie awaryjne przed przeszkodą
	if (distance < dist_thr)
	{
		motor_speed.smooth.left_rear = 0.0;
		motor_speed.smooth.right_rear = 0.0;
		motor_speed.smooth.left_front = 0.0;
		motor_speed.smooth.right_front = 0.0;
	}

	// obliczenie aktualnych poprawek PID
	leftRearSpeedPID.Compute();
	rightRearSpeedPID.Compute();
	leftFrontSpeedPID.Compute();
	rightFrontSpeedPID.Compute();

	// wysterowanie silników
	motorControl(motor_left_rear, motor_speed.out.left_rear);
	motorControl(motor_right_front, motor_speed.out.right_front);
	motorControl(motor_left_front, motor_speed.out.left_front);
	motorControl(motor_right_rear, motor_speed.out.right_rear);

	// zabezpieczenie przed opóźnieniami lub "pływaniem" PID:
	// zatrzymanie silników, gdy prędkość rzeczywista i zadana są równe zero
	if (motor_speed.set.left_rear == 0.0 && motor_speed.raw.left_rear == 0.0) 
		motor_speed.out.left_rear = 0.0;
	if (motor_speed.set.right_rear == 0.0 && motor_speed.raw.right_rear == 0.0) 
		motor_speed.out.right_rear = 0.0;
	if (motor_speed.set.left_front == 0.0 && motor_speed.raw.left_front == 0.0) 
		motor_speed.out.left_front = 0.0;
	if (motor_speed.set.right_front == 0.0 && motor_speed.raw.right_front == 0.0) 
		motor_speed.out.right_front = 0.0;

}

void receive_data()
{
    char c = Serial.read();
    if (c == '\n') 
    {
        process_line(input_buffer);
        input_buffer = "";
    } else 
    {
        input_buffer += c;
    }
} 

void process_line(String line) 
{
    line.trim();

    if (line.startsWith("HELLO")) 
    {
        Serial.println("HELLO_ACK");
        return;
    }

    if (line.startsWith("V:")) 
    {
        // np. "V:0.15,0.0,-0.05"
        float v_x = 0, v_y = 0, w = 0;
        int first_comma = line.indexOf(',', 2);
        int second_comma = line.indexOf(',', first_comma + 1);

        if (first_comma != -1 && second_comma != -1) 
        {
            v_x = line.substring(2, first_comma).toFloat();
            v_y = line.substring(first_comma + 1, second_comma).toFloat();
            w  = line.substring(second_comma + 1).toFloat();
            apply_velocity(v_x, v_y, w);
        }
    }
}

void apply_velocity(float v_x_m, float v_y_m, float w_angular_rad)
{
    double k = r_length + r_width;  // [m]

    float left_front_speed = v_x_m - v_y_m - k * w_angular_rad;   // [m/s]
    float right_front_speed = v_x_m + v_y_m + k * w_angular_rad;  // [m/s]
    float left_rear_speed = v_x_m + v_y_m - k * w_angular_rad;    // [m/s]
    float right_rear_speed = v_x_m - v_y_m + k * w_angular_rad;   // [m/s]

    motor_speed.set.left_front = left_front_speed * 100.0;	      // [cm/s]
    motor_speed.set.right_front = right_front_speed * 100.0;	  // [cm/s]
    motor_speed.set.left_rear = left_rear_speed * 100.0;	      // [cm/s]
    motor_speed.set.right_rear = right_rear_speed * 100.0;	      // [cm/s]
}

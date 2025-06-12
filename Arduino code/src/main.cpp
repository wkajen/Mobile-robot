#include <Arduino.h>
#include <PID_v1.h>
#include <pin_structs.hpp>
#include <config.hpp>
#include <motor_actions.hpp>
#include <functions.hpp>

// struktury do przechowywania wartości pinów dla każdego silnika,
// jako argumenty przyjmują w kolejności: pin PWM, pin jazdy do przodu, pin jazdy do tyłu
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

motor_speed_s motor_speed;
encoder_state_s encoder_state;
counter_s counter;

// zmienne PID
const double KP = 0.85, KI = 0.2, KD = 0.005;  //  KD=0.01
// instancje PID, jako argumenety po kolei: wejście, wyjście, wartość zadana, Kp, Ki, Kd
PID leftRearSpeedPID(&motor_speed.raw.left_rear, &motor_speed.out.left_rear, &motor_speed.set.left_rear, KP, KI, KD, DIRECT);
PID rightRearSpeedPID(&motor_speed.raw.right_rear, &motor_speed.out.right_rear, &motor_speed.set.right_rear, KP, KI, KD, DIRECT);
PID leftFrontSpeedPID(&motor_speed.raw.left_front, &motor_speed.out.left_front, &motor_speed.set.left_front, KP, KI, KD, DIRECT);
PID rightFrontSpeedPID(&motor_speed.raw.right_front, &motor_speed.out.right_front, &motor_speed.set.right_front, KP, KI, KD, DIRECT);

// sekcje zmiennych globalnych (dostępnych tylko w main)
const int avg_window = 5;
unsigned int buff_idx = 0;
double buffer_speed_left_rear[avg_window] = {0.0};
double buffer_speed_right_rear[avg_window] = {0.0};
double buffer_speed_left_front[avg_window] = {0.0};
double buffer_speed_right_front[avg_window] = {0.0};

int distance{0};
unsigned long start_time{0};
unsigned long last_time{0};
unsigned long current_time{0};
long last_time_disp{0};
// koniec sekcji zmiennych globalnych

void setup() 
{
	if (is_serial_on)
	{
		// Serial.begin(115200); 
		Serial.begin(9600);
		Serial.println("Ready!");
	}
	// ustawienie typów (wejście, wyjście) dla I/O
	pinOutSetting(); 

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

	delay(500);
	last_time = start_time = millis();
	distance = getDistance();
}

void loop()
{
	current_time = millis();

	// zliczanie liczby imuplsów na każdym enkoderze
	countPulses(encoder_left_rear_pin, counter.left_rear, encoder_state.left_rear);
	countPulses(encoder_right_rear_pin, counter.right_rear, encoder_state.right_rear);
	countPulses(encoder_left_front_pin, counter.left_front, encoder_state.left_front);
	countPulses(encoder_right_front_pin, counter.right_front, encoder_state.right_front);

	// wyświetlanie prędkości w celu debuggowania
	if (is_serial_on && (current_time - last_time_disp >= 200))
	{
		if (Serial.available() > 0)
		{
			Serial.println(" ");
			Serial.print("Dist: ");
			Serial.print(distance);
			Serial.print("  ; set: ");
			Serial.print(motor_speed.set.left_rear);
			Serial.print("  ; PID out: ");
			Serial.print(motor_speed.out.left_rear);
			Serial.print("  ; encoder v: ");
			Serial.print(motor_speed.raw.left_rear);
			Serial.print("  ; smooth v: ");
			Serial.println(motor_speed.smooth.left_rear);
		}
		last_time_disp = current_time;
	}

	double delta_time = (double)current_time - last_time;
	if(delta_time >= encoder_interval)
	{
		// przeliczenie ilości pulsów enkodera na prędkość liniową
		motor_speed.raw.left_rear = (double)counter.left_rear * tick_mul * (1000.0/encoder_interval);      // [cm/s]
		motor_speed.raw.right_rear = (double)counter.right_rear * tick_mul * (1000.0/encoder_interval);    // [cm/s]
		motor_speed.raw.left_front = (double)counter.left_front * tick_mul * (1000.0/encoder_interval);    // [cm/s]
		motor_speed.raw.right_front = (double)counter.right_front * tick_mul * (1000.0/encoder_interval);  // [cm/s]

		// wpisanie do bufora surowej prędkości
		buffer_speed_left_rear[buff_idx] = motor_speed.raw.left_rear;
		buffer_speed_right_rear[buff_idx] = motor_speed.raw.right_rear;
		buffer_speed_left_front[buff_idx] = motor_speed.raw.left_front;
		buffer_speed_right_front[buff_idx] = motor_speed.raw.right_front;

		// Obliczenie średniej prędkości z ostatnich 5 próbek
		for (int i = 0; i < avg_window; ++i)
		{
			motor_speed.smooth.left_rear += buffer_speed_left_rear[i];
			motor_speed.smooth.right_rear += buffer_speed_right_rear[i];
			motor_speed.smooth.left_front += buffer_speed_left_front[i];
			motor_speed.smooth.right_front += buffer_speed_right_front[i];
		}
		motor_speed.smooth.left_rear /= avg_window;
		motor_speed.smooth.right_rear /= avg_window;
		motor_speed.smooth.left_front /= avg_window;
		motor_speed.smooth.right_front /= avg_window;

		// inkrementacja indeksu bufora
		buff_idx = (buff_idx + 1) % avg_window;

		// wyzerowanie liczników pulsów
		counter.left_rear = counter.right_rear = counter.left_front = counter.right_front = 0;

		last_time = (double)current_time;
		distance = getDistance();
	}

	leftRearSpeedPID.Compute();
	rightRearSpeedPID.Compute();
	leftFrontSpeedPID.Compute();
	rightFrontSpeedPID.Compute();

	if (distance > dist_thr)
	{
		motor_speed.set.left_rear = 30.0;
		motor_speed.set.right_rear = 30.0;
		motor_speed.set.left_front = 30.0;
		motor_speed.set.right_front = 30.0;
	}
	else if (distance > dist_thr/2)
	{
		motor_speed.set.left_rear = 15.0;
		motor_speed.set.right_rear = 15.0;
		motor_speed.set.left_front = 15.0;
		motor_speed.set.right_front = 15.0;
	}
	else
	{
		motor_speed.set.left_rear = 0.0;
		motor_speed.set.right_rear = 0.0;
		motor_speed.set.left_front = 0.0;
		motor_speed.set.right_front = 0.0;
	}

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

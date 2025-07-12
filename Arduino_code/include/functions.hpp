#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

void pinOutSetting();
int getDistance();
void countPulses(int encoder_pin, unsigned int &pulses_num, bool &last_state);
void send_data(double left_front_speed_cm, double right_front_speed_cm, double left_rear_speed_cm, double right_rear_speed_cm, int distance_cm);
// void send_data(double distance);

#endif // FUNCTIONS_HPP
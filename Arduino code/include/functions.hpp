#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

void pinOutSetting();
int getDistance();
void countPulses(int encoder_pin, unsigned int &pulses_num, bool &last_state) ;

#endif // FUNCTIONS_HPP
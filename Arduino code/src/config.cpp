#include "config.hpp"

const unsigned int encoder_res = 20;
const unsigned int encoder_interval = 200;  // [ms] 
const unsigned int pid_interval = 100;  // [ms]  100
const float wheel_radius = 3;   // [cm]
const double max_speed = 65.0;
const double speed_scaling = 255.0 / max_speed;
const double tick_mul = (2*3.14*wheel_radius/encoder_res);  // 9.4

const bool is_serial_on = true;
const int dist_thr = 30;  // [cm]

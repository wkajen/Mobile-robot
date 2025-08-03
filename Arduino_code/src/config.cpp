#include "config.hpp"

const unsigned int encoder_res = 20;
const unsigned int count_interval_ms = 200;  // [ms]
const unsigned int send_interval_ms = 50;    // [ms] 
const unsigned int mq_interval_ms = 30000;    // [ms] 
const unsigned int pid_interval = 100;       // [ms] 
const double wheel_radius = 3.0;             // [cm]
const double max_speed = 75.0;               // [cm/s]
const double speed_scaling = 255.0 / max_speed;
const double tick_mul = (2*3.14*wheel_radius/encoder_res);  // ~=0.94

const bool is_serial_on = true;
const int dist_thr = 5;                     // [cm]

const double r_length = 0.165;              // [m]
const double r_width = 0.145;               // [m]

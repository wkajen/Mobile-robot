#ifndef CONFIG_HPP
#define CONFIG_HPP

extern const unsigned int encoder_res; // rozdzielczość enkodera
extern const unsigned int encoder_interval;  // czas, w którym są zliczane imoulsy enkodera
extern const unsigned int pid_interval;      // czas próbkowania regulatora PID
extern const float wheel_radius; // promień koła [m]
extern const double speed_scaling;  // skalowanie prędkości do odpowiednich zakresów
extern const double max_speed;   // maksymalna prędkość liniowa silników
extern const double tick_mul;    // przelicznik z pulsów na cm

extern const bool is_serial_on;   // czy jest włączone wyświetlanie w terminalu po UART
extern const int dist_thr;  // threshold dla hamowania

#endif // CONFIG_HPP
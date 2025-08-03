#ifndef CONFIG_HPP
#define CONFIG_HPP

extern const unsigned int encoder_res;        // rozdzielczość enkodera
extern const unsigned int count_interval_ms;  // okres czasu, w którym są zliczane impulsy enkodera
extern const unsigned int send_interval_ms;   // okres czasu, w jakim są wysyłane dane do RPi
extern const unsigned int mq_interval_ms;     // okres czasu, w którym są zliczane impulsy enkodera
extern const unsigned int pid_interval;       // czas próbkowania regulatora PID
extern const double wheel_radius;             // promień koła [m]
extern const double speed_scaling;            // skalowanie prędkości do odpowiednich zakresów
extern const double max_speed;                // maksymalna prędkość liniowa silników
extern const double tick_mul;                 // przelicznik z pulsów na cm

extern const bool is_serial_on;               // czy jest włączone wyświetlanie w terminalu po UART
extern const int dist_thr;                    // threshold dla hamowania

extern const double r_length;                 // długość robota
extern const double r_width;                  // szerokość robota

#endif // CONFIG_HPP

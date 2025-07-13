#include "aht10_node.hpp"

#include <chrono>
#include <fstream>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

// ---------------- GPIO Class ----------------
GPIO::GPIO(int gpio_chip, int pin_number)
    : pin_(pin_number), blinking_(false), stop_thread_(false)
{
    // Export pin
    std::ofstream export_file("/sys/class/gpio/export");
    if (export_file) {
        export_file << pin_number;
    }

    // Ustawienie kierunku outputu
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/direction";
    std::ofstream direction_file(direction_path);
    if (direction_file) {
        direction_file << "out";
    }

    value_path_ = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/value";
    control_thread_ = std::thread(&GPIO::blinkLoop, this);
}

GPIO::~GPIO() {
    stop_thread_ = true;
    if (control_thread_.joinable()) 
    {
        control_thread_.join();
    }

    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (unexport_file) 
    {
        unexport_file << pin_;
    }
}

void GPIO::set(bool value) 
{
    blinking_ = false;  // zatrzymanie mrugania diodą
    writeValue(value);
}

void GPIO::startBlinking() 
{
    blinking_ = true;
}

void GPIO::writeValue(bool value) 
{
    std::ofstream value_file(value_path_);
    if (value_file) 
    {
        value_file << (value ? "1" : "0");
    }
}

void GPIO::blinkLoop() 
{
    bool state = false;
    while (!stop_thread_) 
    {
        if (blinking_) 
        {
            state = !state;
            writeValue(state);
            std::this_thread::sleep_for(500ms);  // mruganie 1Hz (zmiana co 0.5s)
        } 
        else 
        {
            if (state) 
            {
                writeValue(false);
                state = false;
            }
            std::this_thread::sleep_for(100ms);
        }
    }
}

// ------------- AHT10Node Class --------------

AHT10Node::AHT10Node()
  : Node("aht10_sensor_node"),
    sensor_("/dev/i2c-1", 0x38),
    led_gpio_(4, 22)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    timer_ = this->create_wall_timer(2s, std::bind(&AHT10Node::readAndPublish, this));
}

void AHT10Node::readAndPublish() 
{
    auto [temp, hum] = sensor_.getData();

    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = this->get_clock()->now();
    msg.temperature = temp;
    msg.variance = 0.0;

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Temperature: %.2f °C, Humidity: %.2f %%", temp, hum);

    if (temp > 30.0) 
    {
        led_gpio_.startBlinking();
    } 
    else 
    {
        led_gpio_.set(false);  // Turn off if below threshold
    }
}

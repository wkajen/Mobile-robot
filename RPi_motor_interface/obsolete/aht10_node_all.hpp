#ifndef AHT10_HPP_NEW_
#define AHT10_HPP_NEW_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>

class GPIO {
public:
    GPIO(int gpio_chip, int pin_number);
    ~GPIO();
    void set(bool value);           // Set on/off
    void startBlinking();           // Start blinking at 1Hz

private:
    int pin_;
    std::string value_path_;
    std::atomic<bool> blinking_;
    std::atomic<bool> stop_thread_;
    std::thread control_thread_;

    void writeValue(bool value);
    void blinkLoop();
};

class AHT10Sensor {
public:
    explicit AHT10Sensor(const std::string& i2c_device = "/dev/i2c-1", uint8_t addr = 0x38);
    ~AHT10Sensor();
    bool readSensor(float& temperature, float& humidity);

private:
    int fd_;
    uint8_t i2c_addr_;
};

class AHT10Node : public rclcpp::Node {
public:
    AHT10Node();

private:
    void readAndPublish();

    AHT10Sensor sensor_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr hum_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float hum_offset{30.0};
    float temp_offset{50.0};
};

#endif // AHT10_HPP

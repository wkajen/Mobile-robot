#ifndef AHT10_HPP
#define AHT10_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>

class AHT10Sensor 
{
public:
    explicit AHT10Sensor(const std::string& i2c_device = "/dev/i2c-1", uint8_t addr = 0x38);
    ~AHT10Sensor();
    bool readSensor(float& temperature, float& humidity);

private:
    int fd_;
    uint8_t i2c_addr_;
    float temp_offset{50.0};
    // float hum_offset{30.0};
};

class AHT10Node : public rclcpp::Node 
{
public:
    AHT10Node();

private:
    void readAndPublish();

    AHT10Sensor sensor_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr hum_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool led_on{false};
};

#endif // AHT10_HPP

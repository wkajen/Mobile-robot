#include "motor_interface/aht10_node.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

// ------------- AHT10Sensor Class --------------

AHT10Sensor::AHT10Sensor(const std::string& i2c_device, uint8_t addr)
    : i2c_addr_(addr)
{
    fd_ = open(i2c_device.c_str(), O_RDWR);
    if (fd_ < 0) 
    {
        perror("Failed to open I2C bus");
        throw std::runtime_error("I2C open failed");
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) 
    {
        perror("Failed to set I2C address");
        throw std::runtime_error("I2C ioctl failed");
    }

    usleep(200000); // 200 ms
}

AHT10Sensor::~AHT10Sensor() 
{
    if (fd_ >= 0) 
    {
        close(fd_);
        std::system("pkill -f led_blink.py");
        std::system("python3 ~/ros/motor_interface/scripts/led_turn_off.py &");
        usleep(200000); // 200 ms
        std::system("pkill -f led_turn_off.py");
    }
}

bool AHT10Sensor::readSensor(float& temperature, float& humidity) 
{
    uint8_t measure_cmd[3] = { 0xAC, 0x33, 0x00 };

    if (write(fd_, measure_cmd, 3) != 3) {
        perror("Failed to write measure command");
        return false;
    }

    usleep(500000); // 500 ms

    uint8_t data[6] = {0};
    if (read(fd_, data, 6) != 6) {
        perror("Failed to read sensor data");
        return false;
    }

    uint32_t raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    temperature = ((raw_temp * 200.0) / 1048576.0) - AHT10Sensor::temp_offset;

    uint32_t raw_hum = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    humidity = (raw_hum / 1048576.0) * 100.0;

    // if (humidity > AHT10Sensor::hum_offset)
    //     humidity -= AHT10Sensor::hum_offset;

    return true;
}

/* AHT10Node implementation */

AHT10Node::AHT10Node()
    : Node("aht10_sensor_node"), sensor_("/dev/i2c-1", 0x38)
{
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    hum_pub_ = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("humidity", 10);

    timer_ = this->create_wall_timer(5s, std::bind(&AHT10Node::readAndPublish, this));
    RCLCPP_INFO(this->get_logger(), "AHT10 sensor node started.");
}

void AHT10Node::readAndPublish() 
{
    float temp = 0.0, hum = 0.0;

    if (sensor_.readSensor(temp, hum)) 
    {
        auto now = this->get_clock()->now();

        sensor_msgs::msg::Temperature temp_msg;
        temp_msg.header.stamp = now;
        temp_msg.temperature = temp;
        temp_msg.variance = 0.0;
        temp_pub_->publish(temp_msg);

        sensor_msgs::msg::RelativeHumidity hum_msg;
        hum_msg.header.stamp = now;
        hum_msg.relative_humidity = hum;
        hum_msg.variance = 0.0;
        hum_pub_->publish(hum_msg);

        // uncomment to get readings in terminal
        // RCLCPP_INFO(this->get_logger(), "Temp: %.2f C, Humidity: %.1f %%", temp, hum);

        if (temp > 30.0 || hum > 75.0) 
        {
            if (!AHT10Node::led_on)
            {
                std::system("pkill -f led_blink.py");
                std::system("python3 ~/ros/motor_interface/scripts/led_blink.py &");
                AHT10Node::led_on = true;
            }
        } 
        else 
        {
            if (AHT10Node::led_on)
            {
                std::system("pkill -f led_blink.py");
                std::system("python3 ~/ros/motor_interface/scripts/led_turn_off.py &");
                AHT10Node::led_on = false;
                usleep(100000); // 100 ms
                std::system("pkill -f led_turn_off.py");
            }
        }
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Sensor read failed.");
    }
}

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AHT10Node>());
    rclcpp::shutdown();
    return 0;
}

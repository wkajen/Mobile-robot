#ifndef MOTOR_INTERFACE_NODE_HPP_
#define MOTOR_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <string>
#include <memory>

class MotorInterfaceNode : public rclcpp::Node 
{
public:
    MotorInterfaceNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void readSerialData();
    void handshake();

    // subskrypcje
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // publikacje
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_speed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distance_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;

    bool is_arduino_ready_;
    std::string port_;
    int baudrate_;
};

#endif  // MOTOR_INTERFACE_NODE_HPP_

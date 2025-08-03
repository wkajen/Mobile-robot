#include "motor_interface/motor_interface_node.hpp"

using namespace std::chrono_literals;
using boost::asio::serial_port_base;

MotorInterfaceNode::MotorInterfaceNode()
    : Node("motor_interface_node"), io_(), serial_(io_), is_arduino_ready_(false)
{
    // Deklaracja parametrów
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 115200);
    this->get_parameter("port", port_);
    this->get_parameter("baudrate", baudrate_);

    // wykonanie handshake
    handshake();

    // ROS interfaces
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotorInterfaceNode::cmdVelCallback, this, std::placeholders::_1));

    wheel_speed_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/wheel_speeds", 10);
    distance_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/distance_sensor", 10);
    lpg_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gas_sensor/lpg", 10);
    co_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gas_sensor/co", 10);
    smoke_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gas_sensor/smoke", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&MotorInterfaceNode::readSerialData, this));
}

void MotorInterfaceNode::handshake()
{
    try 
    {
        serial_.open(port_);
        serial_.set_option(serial_port_base::baud_rate(baudrate_));
        serial_.set_option(serial_port_base::character_size(8));
        serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        boost::asio::write(serial_, boost::asio::buffer("HELLO\n"));
        RCLCPP_INFO(this->get_logger(), "Serial port '%s' opened and handshake sent.", port_.c_str());
    } 
    catch (const boost::system::system_error &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port '%s': %s", port_.c_str(), e.what());
        rclcpp::shutdown();
    }
}

void MotorInterfaceNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::ostringstream oss;
    oss << "V:" << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z << "\n"; 
    try 
    {
        boost::asio::write(serial_, boost::asio::buffer(oss.str()));
    } catch (const boost::system::system_error &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
    }
}

void MotorInterfaceNode::readSerialData()
{
    boost::asio::streambuf buf;
    try 
    {
        boost::asio::read_until(serial_, buf, "\n");
        std::istream is(&buf);
        std::string line;
        std::getline(is, line);

        // Obsługa stringa z Arduino
        if (line.rfind("Data:", 0) == 0) 
        {
            std::string data = line.substr(5);  // usuń "Data:"
            std::stringstream ss(data);
            std::string item;
            std::vector<std::string> parts;

            while (std::getline(ss, item, ',')) 
            {
                parts.push_back(item);
            }

            if (parts.size() >= 4) 
            {
                double s1 = std::stod(parts[0]);
                double s2 = std::stod(parts[1]);
                double s3 = std::stod(parts[2]);
                double s4 = std::stod(parts[3]);

                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = this->get_clock()->now();
                msg.name = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};
                msg.velocity = {s1, s2, s3, s4};
                wheel_speed_pub_->publish(msg);
            }

            if (parts.size() >= 5) 
            {
                double distance = std::stod(parts[4]);

                auto msg = sensor_msgs::msg::Range();
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "distance_sensor";
                msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
                msg.min_range = 0.02;   // [m]
                msg.max_range = 2;      // [m]
                msg.range = distance;
                distance_pub_->publish(msg);
            }

            if (parts.size() >= 8) 
            {
                float lpg = std::stod(parts[5]);
                float co = std::stod(parts[6]);
                float smoke = std::stod(parts[7]);

                std_msgs::msg::Float32 lpg_msg;
                lpg_msg.data = lpg;
                lpg_pub_->publish(lpg_msg);

                std_msgs::msg::Float32 co_msg;
                co_msg.data = co;
                co_pub_->publish(co_msg);

                std_msgs::msg::Float32 smoke_msg;
                smoke_msg.data = smoke;
                smoke_pub_->publish(smoke_msg);
            }
        }

    } 
    catch (const std::exception &e) 
    {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

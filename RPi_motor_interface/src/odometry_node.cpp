#include "motor_interface/odometry_node.hpp"

OdometryNode::OdometryNode()
    : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0)
{
    // this->declare_parameter("wheel_radius", 0.03);  // [m]
    this->declare_parameter("wheel_base", 0.17);     // odległość osi [m]
    // double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_base = this->get_parameter("wheel_base").as_double();

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>
    (
        "/wheel_speeds", 10, 
        // [this, wheel_radius, wheel_base](const sensor_msgs::msg::JointState::SharedPtr msg) 
        [this, wheel_base](const sensor_msgs::msg::JointState::SharedPtr msg) 
        {
            rclcpp::Time current_time = this->get_clock()->now();
            if (last_time_.nanoseconds() == 0) 
            {
                last_time_ = current_time;
                return;
            }

            double dt = (current_time - last_time_).seconds();

            // w wiadomości są kolejno koła: wheel_FL, wheel_FR, wheel_RL, wheel_RR 
            double v_l = (msg->velocity[0] + msg->velocity[2]) / 2.0;
            double v_r = (msg->velocity[1] + msg->velocity[3]) / 2.0;

            // Prędkość liniowa i kątowa robota -> uśrednienie i przeliczenie z cm/s na m/s
            // (bo mam już prędkości liniowe a nie obrotowe, wtedy byłoby * radius?)
            double v = (v_r + v_l) / (2.0 * 100.0);
            double omega = (v_r - v_l) / (wheel_base * 100.0);

            // Integracja
            x_ += v * std::cos(theta_) * dt;
            y_ += v * std::sin(theta_) * dt;
            theta_ += omega * dt;

            // Publikacja Odometry
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";  // base_link
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.orientation.x = 0.0;
            odom.pose.pose.orientation.y = 0.0;
            odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
            odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
            odom.twist.twist.linear.x = v;
            odom.twist.twist.angular.z = omega;
            odom_pub_->publish(odom);

            // Publikacja transformacji TF
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = current_time;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_footprint";  // base_link
            tf_msg.transform.translation.x = x_;
            tf_msg.transform.translation.y = y_;
            tf_msg.transform.rotation.x = 0.0;
            tf_msg.transform.rotation.y = 0.0;
            tf_msg.transform.rotation.z = std::sin(theta_ / 2.0);
            tf_msg.transform.rotation.w = std::cos(theta_ / 2.0);
            tf_broadcaster_->sendTransform(tf_msg);

            last_time_ = current_time;
        });

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_time_ = this->get_clock()->now();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

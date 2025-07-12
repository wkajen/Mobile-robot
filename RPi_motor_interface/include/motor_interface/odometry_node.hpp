#ifndef ODOMETRY_NODE_HPP
#define ODOMETRY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Pozycja i orientacja
    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

#endif // ODOMETRY_NODE_HPP

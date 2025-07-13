#ifndef POSE_PUBLISHER_NODE_HPP_
#define POSE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PosePublisherNode : public rclcpp::Node 
{
public:
  PosePublisherNode();

private:
  void timerCallback();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // POSE_PUBLISHER_NODE_HPP_

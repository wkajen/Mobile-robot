#include "motor_interface/initial_pose_publisher.hpp"

InitialPosePublisher::InitialPosePublisher()
: Node("initial_pose_publisher"), published_(false)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&InitialPosePublisher::publish_initial_pose, this)
  );
}

void InitialPosePublisher::publish_initial_pose()
{
  if (published_) 
  {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";

  msg.pose.pose.position.x = 0.0;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.orientation.w = 1.0;

  msg.pose.covariance = {
    0.25, 0,    0, 0, 0, 0,
    0,    0.25, 0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0.0685
  };

  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Initial pose published.");
  published_ = true;
}

// main
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}


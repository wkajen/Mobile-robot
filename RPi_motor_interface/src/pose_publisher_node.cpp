#include "motor_interface/pose_publisher_node.hpp"

PosePublisherNode::PosePublisherNode()
: Node("pose_publisher_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PosePublisherNode::timerCallback, this)
  );
}

void PosePublisherNode::timerCallback() 
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try 
  {
    transformStamped = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transformStamped.header;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation = transformStamped.transform.rotation;

    pose_pub_->publish(pose);
  } 
  catch (tf2::TransformException &ex) 
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Could not transform 'map' to 'base_footprint': %s", ex.what());
  }
}

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

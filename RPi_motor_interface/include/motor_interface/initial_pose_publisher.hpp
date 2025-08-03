#ifndef INITIAL_POSE_PUBLISHER_HPP_
#define INITIAL_POSE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
	InitialPosePublisher();

private:
	void publish_initial_pose();

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool published_;
};

#endif  // INITIAL_POSE_PUBLISHER_HPP_

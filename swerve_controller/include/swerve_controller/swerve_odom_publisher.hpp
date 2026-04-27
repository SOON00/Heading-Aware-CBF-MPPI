#ifndef SWERVE_CONTROLLER__SWERVE_ODOM_PUBLISHER_HPP_
#define SWERVE_CONTROLLER__SWERVE_ODOM_PUBLISHER_HPP_

#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"

class SwerveOdomPublisher : public rclcpp::Node
{
public:
  SwerveOdomPublisher();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  int findJointIndex(const std::vector<std::string> & names, const std::string & target) const;

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_;
  double y_;
  double theta_;

  bool has_last_time_;
  rclcpp::Time last_time_;

  std::array<double, 4> lx_;
  std::array<double, 4> ly_;

  std::array<double, 4> vx_modules_;
  std::array<double, 4> vy_modules_;

  std::array<std::string, 4> steer_joints_;
  std::array<std::string, 4> wheel_joints_;

  double wheel_radius_;
};

#endif  // SWERVE_CONTROLLER__SWERVE_ODOM_PUBLISHER_HPP_
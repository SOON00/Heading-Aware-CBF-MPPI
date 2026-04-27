#ifndef SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

struct WheelCommands
{
  std::array<double, 4> angles;
  std::array<double, 4> speeds;
};

class SwerveDriveController : public rclcpp::Node
{
public:
  SwerveDriveController();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  WheelCommands calcKinematics(
    double v_bx,
    double v_by,
    double w_bz);

  void publishCommands(const WheelCommands & wheel_commands);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

  double wheel_radius_;
  double wheel_base_;
  double track_width_;

  // 순서: FL, FR, RL, RR
  std::array<std::pair<double, double>, 4> wheel_positions_;
};

#endif  // SWERVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#include "swerve_controller/swerve_drive_controller.hpp"

#include <cmath>
#include <memory>

SwerveDriveController::SwerveDriveController()
: Node("swerve_drive_controller"),
  wheel_radius_(0.065),
  wheel_base_(0.70),
  track_width_(0.30)
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    std::bind(&SwerveDriveController::cmdVelCallback, this, std::placeholders::_1)
  );

  steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/swerve_steering_controller/commands",
    10
  );

  velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/swerve_velocity_controller/commands",
    10
  );

  wheel_positions_ = {{
    { wheel_base_ / 2.0,  track_width_ / 2.0},  // front_left
    { wheel_base_ / 2.0, -track_width_ / 2.0},  // front_right
    {-wheel_base_ / 2.0,  track_width_ / 2.0},  // rear_left
    {-wheel_base_ / 2.0, -track_width_ / 2.0}   // rear_right
  }};

  RCLCPP_INFO(this->get_logger(), "Swerve drive controller started.");
}

void SwerveDriveController::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double linear_x = msg->linear.x;
  const double linear_y = msg->linear.y;
  const double angular_z = msg->angular.z;

  WheelCommands wheel_commands = calcKinematics(
    linear_x,
    linear_y,
    angular_z
  );

  publishCommands(wheel_commands);
}

WheelCommands SwerveDriveController::calcKinematics(
  double v_bx,
  double v_by,
  double w_bz)
{
  WheelCommands commands;

  for (size_t i = 0; i < wheel_positions_.size(); ++i) {
    const double l_ix = wheel_positions_[i].first;
    const double l_iy = wheel_positions_[i].second;

    const double v_ix = v_bx - w_bz * l_iy;
    const double v_iy = v_by + w_bz * l_ix;

    const double speed = std::sqrt(v_ix * v_ix + v_iy * v_iy) / wheel_radius_;
    const double angle = std::atan2(v_iy, v_ix);

    commands.speeds[i] = speed;
    commands.angles[i] = angle;
  }

  return commands;
}

void SwerveDriveController::publishCommands(
  const WheelCommands & wheel_commands)
{
  std_msgs::msg::Float64MultiArray steering_msg;
  steering_msg.data.resize(4);

  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data.resize(4);

  for (size_t i = 0; i < 4; ++i) {
    steering_msg.data[i] = wheel_commands.angles[i];
    velocity_msg.data[i] = wheel_commands.speeds[i];
  }

  steering_pub_->publish(steering_msg);
  velocity_pub_->publish(velocity_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SwerveDriveController>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
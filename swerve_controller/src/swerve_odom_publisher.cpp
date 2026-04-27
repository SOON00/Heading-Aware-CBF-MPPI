#include "swerve_controller/swerve_odom_publisher.hpp"

#include <cmath>
#include <memory>

SwerveOdomPublisher::SwerveOdomPublisher()
: Node("odom_publisher"),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  has_last_time_(false),
  lx_{0.35, 0.35, -0.35, -0.35},
  ly_{0.15, -0.15, 0.15, -0.15},
  vx_modules_{0.0, 0.0, 0.0, 0.0},
  vy_modules_{0.0, 0.0, 0.0, 0.0},
  steer_joints_{
    "front_left_steer_joint",
    "front_right_steer_joint",
    "rear_left_steer_joint",
    "rear_right_steer_joint"
  },
  wheel_joints_{
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint"
  },
  wheel_radius_(0.065)
{
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&SwerveOdomPublisher::jointStateCallback, this, std::placeholders::_1)
  );

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_plugin/out",
    10,
    std::bind(&SwerveOdomPublisher::imuCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Odom publisher started.");
}

void SwerveOdomPublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & q = msg->orientation;

  theta_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  );
}

int SwerveOdomPublisher::findJointIndex(
  const std::vector<std::string> & names,
  const std::string & target) const
{
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == target) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void SwerveOdomPublisher::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  rclcpp::Time current_time(msg->header.stamp);

  if (!has_last_time_) {
    last_time_ = current_time;
    has_last_time_ = true;
    return;
  }

  const double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  if (dt <= 0.0) {
    return;
  }

  std::array<double, 4> steer_angles{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> wheel_speeds{0.0, 0.0, 0.0, 0.0};

  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t j = 0; j < steer_joints_.size(); ++j) {
      if (msg->name[i] == steer_joints_[j]) {
        if (i < msg->position.size()) {
          steer_angles[j] = msg->position[i];
        }
      }
    }

    for (size_t j = 0; j < wheel_joints_.size(); ++j) {
      if (msg->name[i] == wheel_joints_[j]) {
        if (i < msg->velocity.size()) {
          wheel_speeds[j] = msg->velocity[i];
        }
      }
    }
  }

  for (size_t i = 0; i < 4; ++i) {
    const double v = wheel_speeds[i] * wheel_radius_;

    vx_modules_[i] = v * std::cos(steer_angles[i]);
    vy_modules_[i] = v * std::sin(steer_angles[i]);
  }

  double vx = 0.0;
  double vy = 0.0;

  for (size_t i = 0; i < 4; ++i) {
    vx += vx_modules_[i];
    vy += vy_modules_[i];
  }

  vx /= 4.0;
  vy /= 4.0;

  double numerator = 0.0;
  double denominator = 0.0;

  for (size_t i = 0; i < 4; ++i) {
    numerator += lx_[i] * vy_modules_[i] - ly_[i] * vx_modules_[i];
    denominator += lx_[i] * lx_[i] + ly_[i] * ly_[i];
  }

  double omega = 0.0;
  if (std::abs(denominator) > 1e-9) {
    omega = numerator / denominator;
  }

  const double delta_x = (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
  const double delta_y = (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;

  x_ += delta_x;
  y_ += delta_y;

  const double qz = std::sin(theta_ / 2.0);
  const double qw = std::cos(theta_ / 2.0);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  odom_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(t);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SwerveOdomPublisher>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
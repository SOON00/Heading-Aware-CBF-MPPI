#include "swerve_navigation/waypoint_goal_sender_case2.hpp"

#include <chrono>
#include <cmath>
#include <future>

using namespace std::chrono_literals;

WaypointGoalSenderCase2::WaypointGoalSenderCase2()
: Node("waypoint_goal_sender_case2")
{
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose");
}

void WaypointGoalSenderCase2::yawToQuaternion(
  double yaw,
  double & qz,
  double & qw)
{
  qz = std::sin(yaw * 0.5);
  qw = std::cos(yaw * 0.5);
}

geometry_msgs::msg::PoseStamped WaypointGoalSenderCase2::makePose(
  double x,
  double y,
  double yaw,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;

  pose.header.frame_id = frame_id;
  pose.header.stamp = this->now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  double qz = 0.0;
  double qw = 1.0;
  yawToQuaternion(yaw, qz, qw);

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;

  return pose;
}

bool WaypointGoalSenderCase2::runSingleGoal(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & goal_name)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Start %s: x=%.4f, y=%.4f",
    goal_name.c_str(),
    pose.pose.position.x,
    pose.pose.position.y);

  if (!nav_to_pose_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "NavigateToPose action server is not available.");
    return false;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;

  double last_distance_remaining = -1.0;
  int last_recoveries = -1;

  send_goal_options.feedback_callback =
    [this, goal_name, &last_distance_remaining, &last_recoveries](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }

      const double current_x = feedback->current_pose.pose.position.x;
      const double current_y = feedback->current_pose.pose.position.y;
      const double distance_remaining = feedback->distance_remaining;
      const int recoveries = feedback->number_of_recoveries;

      const double rounded_distance =
        std::round(distance_remaining * 100.0) / 100.0;

      if (rounded_distance != last_distance_remaining ||
          recoveries != last_recoveries)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s Feedback] current=(%.3f, %.3f), distance_remaining=%.3f, recoveries=%d",
          goal_name.c_str(),
          current_x,
          current_y,
          distance_remaining,
          recoveries);

        last_distance_remaining = rounded_distance;
        last_recoveries = recoveries;
      }
    };

  auto goal_handle_future = nav_to_pose_client_->async_send_goal(
    goal_msg,
    send_goal_options);

  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to send %s.",
      goal_name.c_str());
    return false;
  }

  auto goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s was rejected by server.",
      goal_name.c_str());
    return false;
  }

  auto result_future = nav_to_pose_client_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get result for %s.",
      goal_name.c_str());
    return false;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(),
        "%s reached successfully.",
        goal_name.c_str());
      return true;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(
        this->get_logger(),
        "%s was canceled.",
        goal_name.c_str());
      return false;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        this->get_logger(),
        "%s failed.",
        goal_name.c_str());
      return false;

    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "%s returned unknown result.",
        goal_name.c_str());
      return false;
  }
}

void WaypointGoalSenderCase2::runMission()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Waiting for Nav2 NavigateToPose action server...");

  if (!nav_to_pose_client_->wait_for_action_server(30s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "NavigateToPose action server is not available after waiting.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Nav2 is active.");

  const std::string frame_id = "map";

  auto goal_1 = makePose(
    0.4940,
    2.6368,
    1.57,
    frame_id);

  auto goal_2 = makePose(
    -0.4946,
    4.3621,
    3.14,
    frame_id);

  bool ok = runSingleGoal(goal_1, "Goal 1");

  if (!ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Stopping mission because Goal 1 failed.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Goal 1 complete. Starting Goal 2...");

  ok = runSingleGoal(goal_2, "Goal 2");

  if (!ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Goal 2 failed.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "All goals completed successfully.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointGoalSenderCase2>();
  node->runMission();

  rclcpp::shutdown();
  return 0;
}
#include "swerve_navigation/waypoint_goal_sender_case1.hpp"

#include <chrono>
#include <cmath>
#include <future>

using namespace std::chrono_literals;

WaypointGoalSenderCase1::WaypointGoalSenderCase1()
: Node("waypoint_goal_sender_case1")
{
  nav_through_poses_client_ =
    rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");
}

void WaypointGoalSenderCase1::yawToQuaternion(
  double yaw,
  double & qz,
  double & qw)
{
  qz = std::sin(yaw * 0.5);
  qw = std::cos(yaw * 0.5);
}

geometry_msgs::msg::PoseStamped WaypointGoalSenderCase1::makePose(
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

bool WaypointGoalSenderCase1::runWaypointGoals(
  const std::vector<geometry_msgs::msg::PoseStamped> & goal_poses)
{
  if (!nav_through_poses_client_->wait_for_action_server(30s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "NavigateThroughPoses action server is not available.");
    return false;
  }

  NavigateThroughPoses::Goal goal_msg;
  goal_msg.poses = goal_poses;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending %zu waypoint goals.",
    goal_poses.size());

  rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions send_goal_options;

  int last_poses_remaining = -1;
  int last_recoveries = -1;

  send_goal_options.feedback_callback =
    [this, &last_poses_remaining, &last_recoveries](
      GoalHandleNavigateThroughPoses::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }

      const double current_x = feedback->current_pose.pose.position.x;
      const double current_y = feedback->current_pose.pose.position.y;
      const double distance_remaining = feedback->distance_remaining;
      const int poses_remaining = feedback->number_of_poses_remaining;
      const int recoveries = feedback->number_of_recoveries;

      if (poses_remaining != last_poses_remaining ||
          recoveries != last_recoveries)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[Feedback] current=(%.3f, %.3f), distance_remaining=%.3f, "
          "poses_remaining=%d, recoveries=%d",
          current_x,
          current_y,
          distance_remaining,
          poses_remaining,
          recoveries);

        last_poses_remaining = poses_remaining;
        last_recoveries = recoveries;
      }
    };

  auto goal_handle_future =
    nav_through_poses_client_->async_send_goal(
      goal_msg,
      send_goal_options);

  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to send waypoint goals.");
    return false;
  }

  auto goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Waypoint goals were rejected by server.");
    return false;
  }

  auto result_future =
    nav_through_poses_client_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get waypoint navigation result.");
    return false;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(),
        "Waypoint navigation succeeded.");
      return true;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(
        this->get_logger(),
        "Waypoint navigation was canceled.");
      return false;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        this->get_logger(),
        "Waypoint navigation failed.");
      return false;

    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "Waypoint navigation returned unknown result.");
      return false;
  }
}

void WaypointGoalSenderCase1::runMission()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Waiting for Nav2 NavigateThroughPoses action server...");

  if (!nav_through_poses_client_->wait_for_action_server(30s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "NavigateThroughPoses action server is not available after waiting.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Nav2 is active.");

  const std::string frame_id = "map";

  auto goal_1 = makePose(
    2.2254,
    3.1371,
    3.14,
    frame_id);

  auto goal_2 = makePose(
    4.1596,
    6.5161,
    0.0,
    frame_id);

  std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
  goal_poses.push_back(goal_1);
  goal_poses.push_back(goal_2);

  RCLCPP_INFO(
    this->get_logger(),
    "Sending 2 waypoint goals in frame: %s",
    frame_id.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "Goal 1: x=2.2254, y=3.1371, yaw=3.14");

  RCLCPP_INFO(
    this->get_logger(),
    "Goal 2: x=4.1596, y=6.5161, yaw=0.00");

  const bool ok = runWaypointGoals(goal_poses);

  if (!ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Waypoint mission failed.");
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "All waypoint goals completed successfully.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointGoalSenderCase1>();
  node->runMission();

  rclcpp::shutdown();
  return 0;
}
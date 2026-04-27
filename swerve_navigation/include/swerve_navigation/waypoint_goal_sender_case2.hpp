#ifndef SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE2_HPP_
#define SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE2_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class WaypointGoalSenderCase2 : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointGoalSenderCase2();

  void runMission();

private:
  geometry_msgs::msg::PoseStamped makePose(
    double x,
    double y,
    double yaw,
    const std::string & frame_id);

  bool runSingleGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & goal_name);

  void yawToQuaternion(
    double yaw,
    double & qz,
    double & qw);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
};

#endif  // SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE2_HPP_
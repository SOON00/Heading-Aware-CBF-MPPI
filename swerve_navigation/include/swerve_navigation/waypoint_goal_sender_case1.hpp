#ifndef SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE1_HPP_
#define SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE1_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

class WaypointGoalSenderCase1 : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  WaypointGoalSenderCase1();

  void runMission();

private:
  geometry_msgs::msg::PoseStamped makePose(
    double x,
    double y,
    double yaw,
    const std::string & frame_id);

  bool runWaypointGoals(
    const std::vector<geometry_msgs::msg::PoseStamped> & goal_poses);

  void yawToQuaternion(
    double yaw,
    double & qz,
    double & qw);

private:
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
};

#endif  // SWERVE_NAVIGATION__WAYPOINT_GOAL_SENDER_CASE1_HPP_
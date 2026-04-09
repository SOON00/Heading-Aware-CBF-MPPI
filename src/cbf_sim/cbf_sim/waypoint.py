import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, FollowPath

import math


class ThroughPosesClient(Node):
    def __init__(self):
        super().__init__('through_poses_client')

        self.compute_client = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses')

        self.follow_client = ActionClient(
            self, FollowPath, 'follow_path')

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_request(self):
        # 👉 waypoint + goal (순서 중요)
        poses = []
        poses.append(self.create_pose(0.09634, -1.5514, -1.57))  # 경유지
        poses.append(self.create_pose(0.223, -1.521, 3.14))    # 최종 goal

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = poses

        self.compute_client.wait_for_server()
        self.get_logger().info("Computing path...")

        future = self.compute_client.send_goal_async(goal_msg)
        future.add_done_callback(self.compute_done_callback)

    def compute_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Path request rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)

    def path_result_callback(self, future):
        result = future.result().result
        path = result.path

        self.get_logger().info("Path received, following...")

        follow_goal = FollowPath.Goal()
        follow_goal.path = path

        self.follow_client.wait_for_server()
        self.follow_client.send_goal_async(follow_goal)


def main():
    rclpy.init()
    node = ThroughPosesClient()
    node.send_request()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

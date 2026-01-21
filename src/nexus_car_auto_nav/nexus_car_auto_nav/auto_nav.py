#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class AutoNav(Node):
    def __init__(self):
        super().__init__("auto_nav")
        # ===== NAV2 CLIENT =====
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_sent = False
        self.get_logger().info("Waiting for Nav2...")
        self.client.wait_for_server()
        self.create_timer(1.0, self.send_goal_once)

    def send_goal_once(self):
        if self.goal_sent:
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        # ===== TARGET POSITION =====
        goal.pose.pose.position.x = 3.7
        goal.pose.pose.position.y = 0.0
        # ===== TARGET ORIENTATION =====
        q = quaternion_from_euler(0, 0, 0)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        self.get_logger().info("Sending Nav2 goal...")
        self.goal_sent = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav goal rejected")
            self.goal_sent = False
            return
        self.get_logger().info("Nav goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_done_cb)

    def nav_done_cb(self, future):
        status = future.result().status
        if status != 4:  # SUCCEEDED
            self.get_logger().error(f"Navigation failed (status {status})")
            return
        self.get_logger().info("Navigation succeeded ✅")

def main():
    rclpy.init()
    node = AutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

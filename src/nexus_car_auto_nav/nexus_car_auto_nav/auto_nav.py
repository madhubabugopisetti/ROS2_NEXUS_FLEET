#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class AutoNav(Node):
    def __init__(self):
        super().__init__("auto_nav")

        # ===== NAV2 ACTION =====
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ===== CMD_VEL (FROM YOUR TELEOP STYLE) =====
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ===== ODOM =====
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

        self.yaw = 0.0
        self.target_yaw = None

        self.phase = 0  # 0 = nav, 1 = rotate
        self.goal_active = False

        self.angular_speed = 0.8
        self.creep = 0.02

        self.get_logger().info("Waiting for Nav2...")
        self.client.wait_for_server()

        self.create_timer(1.0, self.send_goal_once)
        self.create_timer(0.05, self.rotate_step)

    # ================= ODOM =================
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ================= NAV GOAL =================
    def send_goal_once(self):
        if self.goal_active or self.phase != 0:
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = 3.5
        goal.pose.pose.position.y = 0.0

        q = quaternion_from_euler(0, 0, 0)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.get_logger().info("Sending Nav2 goal")
        self.goal_active = True

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav goal rejected")
            return

        self.get_logger().info("Nav goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_done_cb)

    def nav_done_cb(self, future):
        self.get_logger().info("Navigation complete, switching to rotation")
        self.goal_active = False
        self.phase = 1

        # Rotate 90 degrees right RELATIVE to current yaw
        self.target_yaw = self.normalize(self.yaw + math.radians(130))

    # ================= ROTATION (CMD_VEL) =================
    def rotate_step(self):
        if self.phase != 1 or self.target_yaw is None:
            return

        err = self.normalize(self.target_yaw - self.yaw)

        if abs(err) < 0.03:  # ~2 degrees
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Rotation complete")
            self.phase = 2
            return

        ang = self.angular_speed if err > 0 else -self.angular_speed
        self.publish_cmd(self.creep, ang)

    def publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    # ================= UTILS =================
    @staticmethod
    def normalize(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = AutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

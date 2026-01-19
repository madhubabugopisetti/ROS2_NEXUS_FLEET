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

        # ===== NAV2 =====
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ===== CMD_VEL =====
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ===== ODOM =====
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # ===== STATE =====
        self.yaw = 0.0
        self.target_yaw = None

        self.phase = "NAV"   # NAV → STOP → ROTATE → DONE
        self.goal_active = False

        # ===== ROTATION PARAMS =====
        self.max_ang = 0.8
        self.min_ang = 0.15
        self.creep = 0.02
        self.yaw_tol = 0.03  # radians

        self.get_logger().info("Waiting for Nav2...")
        self.client.wait_for_server()

        self.create_timer(1.0, self.send_goal_once)
        self.create_timer(0.05, self.control_loop)

    # ================= ODOM =================
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ================= NAV GOAL =================
    def send_goal_once(self):
        if self.phase != "NAV" or self.goal_active:
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

        self.goal_active = True
        self.get_logger().info("Sending Nav2 goal")

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Nav goal rejected")
            self.goal_active = False
            return

        self.get_logger().info("Nav goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_done_cb)

    def nav_done_cb(self, future):
        status = future.result().status

        if status != 4:  # SUCCEEDED
            self.get_logger().error(f"Navigation failed (status {status})")
            self.goal_active = False
            return

        self.get_logger().info("Navigation succeeded → stopping")
        self.goal_active = False
        self.phase = "STOP"

        # Hard stop before rotation
        self.publish_cmd(0.0, 0.0)

        # Compute RELATIVE left turn (130°)
        self.target_yaw = self.normalize(self.yaw + math.radians(130))

    # ================= CONTROL LOOP =================
    def control_loop(self):
        if self.phase == "STOP":
            self.phase = "ROTATE"
            return

        if self.phase != "ROTATE":
            return

        err = self.normalize(self.target_yaw - self.yaw)

        if abs(err) < self.yaw_tol:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Rotation complete")
            self.phase = "DONE"
            return

        # Proportional angular velocity
        ang = 1.2 * err
        ang = max(-self.max_ang, min(self.max_ang, ang))

        # Minimum angular speed to overcome friction
        if abs(ang) < self.min_ang:
            ang = math.copysign(self.min_ang, ang)

        self.publish_cmd(self.creep, ang)

    # ================= CMD =================
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

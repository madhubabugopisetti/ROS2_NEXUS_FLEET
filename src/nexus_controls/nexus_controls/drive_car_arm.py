#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_base_and_arm')

        # Publishers
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Base params (same as yours)
        self.linear_speed = 0.4
        self.angular_speed = 0.8
        self.creep = 0.02  # IMPORTANT for skid-steer stability

        # Arm params
        self.step = 0.05
        self.gripper_step = 0.005

        self.joint_names = [
            "shoulder_joint",
            "elbow_joint",
            "forearm_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "left_finger_joint",
            "right_finger_joint"
        ]

        self.joint_pos = [0.0] * 7

        self.get_logger().info("""
            ================ TELEOP =================

            BASE:
            i : forward
            k : backward
            u : turn left
            o : turn right
            j : stop
            q : quit

            ARM:
            s / S : shoulder + / -
            e / E : elbow + / -
            f / F : forearm + / -
            p / P : wrist pitch + / -
            r / R : wrist roll + / -
            O / C : gripper open / close

            ========================================
            """)

    def publish_base(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.base_pub.publish(msg)

    def publish_arm(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = self.joint_pos
        p.time_from_start.sec = 1

        traj.points.append(p)
        self.arm_pub.publish(traj)

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)

                # ========== BASE ==========
                if key == 'i':
                    self.publish_base(self.linear_speed, 0.0)
                elif key == 'k':
                    self.publish_base(-self.linear_speed, 0.0)
                elif key == 'u':
                    self.publish_base(self.creep, self.angular_speed)
                elif key == 'o':
                    self.publish_base(self.creep, -self.angular_speed)
                elif key == 'j':
                    self.publish_base(0.0, 0.0)

                # ========== ARM ==========
                elif key == 's':   # shoulder +
                    self.joint_pos[0] += self.step
                elif key == 'S':   # shoulder -
                    self.joint_pos[0] -= self.step

                elif key == 'e':   # elbow +
                    self.joint_pos[1] += self.step
                elif key == 'E':   # elbow -
                    self.joint_pos[1] -= self.step

                elif key == 'f':   # forearm +
                    self.joint_pos[2] += self.step
                elif key == 'F':   # forearm -
                    self.joint_pos[2] -= self.step

                elif key == 'p':   # wrist pitch +
                    self.joint_pos[3] += self.step
                elif key == 'P':   # wrist pitch -
                    self.joint_pos[3] -= self.step

                elif key == 'r':   # wrist roll +
                    self.joint_pos[4] += self.step
                elif key == 'R':   # wrist roll -
                    self.joint_pos[4] -= self.step

                elif key == 'O':   # open gripper
                    self.joint_pos[5] += self.gripper_step
                    self.joint_pos[6] += self.gripper_step
                elif key == 'C':   # close gripper
                    self.joint_pos[5] -= self.gripper_step
                    self.joint_pos[6] -= self.gripper_step

                elif key == 'q':
                    self.publish_base(0.0, 0.0)
                    break

                else:
                    continue

                # Clamp gripper
                self.joint_pos[5] = self.clamp(self.joint_pos[5], 0.0, 0.025)
                self.joint_pos[6] = self.clamp(self.joint_pos[6], 0.0, 0.025)

                self.publish_arm()

                self.get_logger().info(
                    "Joint pos: " + str(["%.2f" % p for p in self.joint_pos])
                )

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

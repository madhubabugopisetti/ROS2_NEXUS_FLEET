#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KeyboardArmTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_arm_teleop')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

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
================ ARM TELEOP =================

s / x : shoulder + / -
e / d : elbow + / -
f / v : forearm + / -
r / t : wrist pitch + / -
y / h : wrist roll + / -
o / p : gripper open / close

q : quit

============================================
""")

    def publish(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = self.joint_pos
        p.time_from_start.sec = 1

        traj.points.append(p)
        self.pub.publish(traj)

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)

                changed = True

                if key == 's':
                    self.joint_pos[0] += self.step
                elif key == 'x':
                    self.joint_pos[0] -= self.step

                elif key == 'e':
                    self.joint_pos[1] += self.step
                elif key == 'd':
                    self.joint_pos[1] -= self.step

                elif key == 'f':
                    self.joint_pos[2] += self.step
                elif key == 'v':
                    self.joint_pos[2] -= self.step

                elif key == 'r':
                    self.joint_pos[3] += self.step
                elif key == 't':
                    self.joint_pos[3] -= self.step

                elif key == 'y':
                    self.joint_pos[4] += self.step
                elif key == 'h':
                    self.joint_pos[4] -= self.step

                elif key == 'o':
                    self.joint_pos[5] += self.gripper_step
                    self.joint_pos[6] += self.gripper_step
                elif key == 'p':
                    self.joint_pos[5] -= self.gripper_step
                    self.joint_pos[6] -= self.gripper_step
                elif key == '1':
                    # Preset HOME pose
                    self.joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    traj = JointTrajectory()
                    traj.joint_names = self.joint_names

                    p = JointTrajectoryPoint()
                    p.positions = self.joint_pos
                    p.time_from_start.sec = 2

                    traj.points.append(p)
                    self.pub.publish(traj)

                    self.get_logger().info("Sent HOME pose")
                    continue

                elif key == 'q':
                    break
                else:
                    changed = False

                # Clamp gripper
                self.joint_pos[5] = self.clamp(self.joint_pos[5], 0.0, 0.025)
                self.joint_pos[6] = self.clamp(self.joint_pos[6], 0.0, 0.025)

                if changed:
                    self.publish()
                    self.get_logger().info(
                        "Joint pos: " + str(["%.2f" % p for p in self.joint_pos])
                    )

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = KeyboardArmTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

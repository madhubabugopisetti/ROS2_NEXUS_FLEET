#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.4
        self.angular_speed = 0.8
        self.creep = 0.02  # IMPORTANT for skid-steer stability

        self.get_logger().info(
            "Controls:\n"
            "  w : forward\n"
            "  s : backward\n"
            "  a : turn left\n"
            "  d : turn right\n"
            "  x : stop\n"
            "  q : quit\n"
        )

    def publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)

                if key == 'w':
                    self.publish(self.linear_speed, 0.0)
                elif key == 's':
                    self.publish(-self.linear_speed, 0.0)
                elif key == 'a':
                    self.publish(self.creep, self.angular_speed)
                elif key == 'd':
                    self.publish(self.creep, -self.angular_speed)
                elif key == 'x':
                    self.publish(0.0, 0.0)
                elif key == 'q':
                    self.publish(0.0, 0.0)
                    break
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

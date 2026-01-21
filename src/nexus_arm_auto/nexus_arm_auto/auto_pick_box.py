#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import time

JOINTS = [
    "shoulder_joint",
    "elbow_joint",
    "forearm_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "left_finger_joint",
    "right_finger_joint",
]

class AutoPickBox(Node):
    def __init__(self):
        super().__init__("auto_pick_box")
        self.get_logger().info("Auto Pick node started.")

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            "/arm_camera/image",
            self.image_cb,
            10
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_cb,
            10
        )

        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.step = 0
        self.current_pose = [0.0] * 7

        self.timer = self.create_timer(0.1, self.callFunctions)

    def joint_cb(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("arm_camera", frame)
        cv2.waitKey(1)
    
    def send_pose(self, pose, t):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = pose
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        p.time_from_start.sec = sec
        p.time_from_start.nanosec = nanosec
        traj.points = [p]
        self.joint_publisher.publish(traj)

    def callFunctions(self):
        match self.step:
            case 0:
                self.default_pose()
            case _:
                pass

    def default_pose(self):
        self.get_logger().info("Set Default Pose")
        self.send_pose(self.current_pose, 5)
        time.sleep(6)
        self.step = 1

def main():
    rclpy.init()
    node = AutoPickBox()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

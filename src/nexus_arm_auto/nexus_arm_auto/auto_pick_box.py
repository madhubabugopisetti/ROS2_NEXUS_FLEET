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

        self.box_entered = False
        self.gap_between = 15

        # shoulder scan variables
        self.shoulder_step = 0.01
        self.shoulder_min = -3.14
        self.shoulder_max = 3.14
        self.shoulder_dir = 1
        #

        self.timer = self.create_timer(0.1, self.callFunctions)

    def joint_cb(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        P = self.gap_between
        self.align_box_in_window(frame, P, h, w)
        cv2.imshow("arm_camera", frame)
        cv2.waitKey(1)

    def align_box_in_window(self, frame, P, h, w):
        cv2.rectangle(frame, (P, P), (w - P, h - P), (0, 255, 0), 2)
        cv2.putText(frame, "-X", (5, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "+X", (w - 40, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "-Y", (w // 2 - 20, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, "+Y", (w // 2 - 20, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = (
            cv2.inRange(hsv, (0,120,70), (10,255,255)) |
            cv2.inRange(hsv, (170,120,70), (180,255,255))
        )
        top_strip = red_mask[:P, :]
        bottom_strip = red_mask[h-P:, :]
        red_top = cv2.countNonZero(top_strip) > 0
        red_bottom = cv2.countNonZero(bottom_strip) > 0
        red_on_y = red_top or red_bottom
        if self.step == 2:
            if red_on_y:
                if not self.box_entered:
                    self.get_logger().info("[STEP 3] Red ENTERED Y padding")
                self.box_entered = True
            elif self.box_entered and not red_on_y:
                self.get_logger().info("[STEP 3] Red EXITED Y padding → STEP 4")
                self.step = 3

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
            case 1:
                self.search_pose()
            case 2:
                self.find_box()
            case _:
                pass

    def default_pose(self):
        self.get_logger().info("Set Default Pose")
        self.send_pose(self.current_pose, 5)
        time.sleep(6)
        self.step = 1

    def search_pose(self):
        self.get_logger().info("set search pose")
        pose = self.current_pose.copy()
        pose[1:7] = [1.0, 1.0, 1.0, 0.1, 0.0, 0.0]
        self.send_pose(pose, 5)
        self.current_pose = pose
        time.sleep(6)
        self.step = 2

    def find_box(self):
        pose = self.current_pose.copy()
        if pose[0] >= self.shoulder_max or pose[0] <= self.shoulder_min:
            self.shoulder_dir *= -1
        pose[0] += self.shoulder_dir * self.shoulder_step
        self.send_pose(pose, 0.1)
        self.current_pose = pose

def main():
    rclpy.init()
    node = AutoPickBox()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

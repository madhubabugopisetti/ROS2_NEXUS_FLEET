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

        # wrist roll scan variables
        self.wrist_step = 0.01
        self.wrist_min = -3.14
        self.wrist_max = 3.14
        self.wrist_dir = 1
        #

        self.err_r = 0
        self.err_x = 0
        self.err_y = 0

        self.arm_fixed = False

        # elbow roll scan variables
        self.elbow_step = 0.001
        self.elbow_min = -1.57
        self.elbow_max = 1.57
        self.elbow_dir = 1
        self.red_in_padding = False
        self.red_is_missing = False
        #

        self.timer = self.create_timer(0.1, self.callFunctions)

    def joint_cb(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape
        P = self.gap_between
        self.align_box_in_window(frame, P, h, w)
        if self.step !=1 or self.step != 1 or self.step !=2:
            self.show_lines_in_CV(frame, P, h, w)
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
        inner = red_mask[P:h-P, P:w-P]
        red_inside_window = cv2.countNonZero(inner) > 0
        self.red_is_missing = not red_inside_window
        top_strip = red_mask[:P, :]
        bottom_strip = red_mask[h-P:, :]
        red_top = cv2.countNonZero(top_strip) > 0
        red_bottom = cv2.countNonZero(bottom_strip) > 0
        red_on_y = red_top or red_bottom
        if red_on_y:
            self.red_in_padding = True
        else:
            self.red_in_padding = False
        if self.step == 2:
            if red_on_y:
                if not self.box_entered:
                    self.get_logger().info("[STEP 3] Red ENTERED Y padding")
                self.box_entered = True
            elif self.box_entered and not red_on_y:
                self.get_logger().info("[STEP 3] Red EXITED Y padding → STEP 4")
                self.step = 3

    def show_lines_in_CV(self, frame, P, h, w):
        vis = frame.copy()
        hsv = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)
        red_mask = (
            cv2.inRange(hsv, (0,120,70), (10,255,255)) |
            cv2.inRange(hsv, (170,120,70), (180,255,255))
        )
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 0, 255), 2)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 500:
                rect = cv2.minAreaRect(largest)
                angle = rect[2]
                box = cv2.boxPoints(rect)
                box = box.astype(int)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                cx = int(rect[0][0])
                cy = int(rect[0][1])
                self.err_x = cy - (h // 2)
                self.err_y = cx - (w // 2)
                self.err_r = angle
                cv2.putText(frame, f"err_x: {self.err_x} err_y: {self.err_y}  err_r(deg): {self.err_r:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.line(frame, (cx, cy), (w // 2, cy), (255, 0, 0), 2)
                cv2.line(frame, (cx, cy), (cx, h // 2), (255, 0, 0), 2)

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

    def handleGrippers(self, action):
        pose = self.current_pose.copy()
        pose[5] = 0.03 if action == "open" else 0.0
        pose[6] = 0.03 if action == "open" else 0.0
        self.send_pose(pose, 0.001)
        self.current_pose = pose
        self.get_logger().info(f"{action} gripper={pose}")

    def should_stop_elbow(self):
        if self.red_in_padding:
            self.get_logger().warn("Red entered padding → STOPPING ELBOW")
            return True
        return False

    def callFunctions(self):
        match self.step:
            case 0:
                self.default_pose()
            case 1:
                self.search_pose()
            case 2:
                self.find_box()
            case 3:
                self.align_wrist_gripper()
            case 4:
                self.align_shoulder_yaxis()
            case 5:
                self.align_forearm_xaxis()
            case 6:
                self.align_elbox_zaxis()
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

    def align_wrist_gripper(self):
        if abs(self.err_r) >= 90.0:
            self.get_logger().info("wrist aligned → now shoulder")
            self.handleGrippers("open")
            self.step = 4
            return
        pose = self.current_pose.copy()
        if pose[4] >= self.wrist_max or pose[4] <= self.wrist_min:
            self.wrist_dir *= -1
        pose[4] += self.wrist_dir * self.wrist_step
        self.send_pose(pose, 0.01)
        self.current_pose = pose

    def align_shoulder_yaxis(self):
        if abs(self.err_y) <= 2.0:
            self.get_logger().info("shoulder aligned → now forearm")
            self.step = 5
            return
        Kp = 0.0001
        delta = Kp * self.err_y
        self.get_logger().info(f"err_y={self.err_y} delta={delta}")
        pose = self.current_pose.copy()
        pose[0] += delta
        self.send_pose(pose, 0.0001)
        self.current_pose = pose

    def align_forearm_xaxis(self):
        if abs(self.err_x) <= 2.0:
            self.get_logger().info("forearm aligned → now elbow")
            self.step = 6
            return
        Kp = 0.0001
        delta = Kp * self.err_x
        self.get_logger().info(f"err_x={self.err_x} delta={delta}")
        pose = self.current_pose.copy()
        pose[2] -= delta
        self.send_pose(pose, 0.0001)
        self.current_pose = pose

    def align_elbox_zaxis(self):
        pose = self.current_pose.copy()
        if abs(pose[1]) >= 1.2:
            self.get_logger().info(f"Elbow reached target limit: {pose[1]:.3f} → STOP")
            self.arm_fixed = True
            time.sleep(2)
            self.step = 4
            return
        if self.should_stop_elbow():
            self.step = 4
            return
        if self.red_is_missing:
            self.get_logger().warn("Red missing inside green window → STOP ELBOW")
            self.arm_fixed = True
            time.sleep(1)
            self.step = 7
            return
        if abs(self.err_x) > 2.0:
            Kp_x = 0.0001
            delta_x = Kp_x * self.err_x
            pose[2] -= delta_x
            self.get_logger().info(f"[ELBOW PHASE] forearm corr: {delta_x:.6f}")
        if pose[1] >= self.elbow_max or pose[1] <= self.elbow_min:
            self.elbow_dir *= -1
        pose[1] += self.elbow_dir * self.elbow_step
        self.get_logger().info(f"[ELBOW PHASE] elbow={pose[1]:.4f}")
        self.send_pose(pose, 0.02)
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

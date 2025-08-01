#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class DualCameraRecorder(Node):
    def __init__(self, record_r1=True, record_r2=True):
        super().__init__('dual_camera_recorder')

        self.bridge = CvBridge()
        self.width = 1280
        self.height = 720
        self.output_dir = 'temp'  # your existing folder

        # Flags for recording
        self.record_r1 = record_r1
        self.record_r2 = record_r2

        # Timestamp string for filenames
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Initialize writers as None
        self.writer_r1_cam1 = None
        self.writer_r1_cam2 = None
        self.writer_r2_cam1 = None
        self.writer_r2_cam2 = None

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        if self.record_r1:
            r1_cam1_filename = f"{timestamp}_R1_camera1.mp4"
            r1_cam2_filename = f"{timestamp}_R1_camera2.mp4"
            r1_cam1_path = os.path.join(self.output_dir, r1_cam1_filename)
            r1_cam2_path = os.path.join(self.output_dir, r1_cam2_filename)
            self.writer_r1_cam1 = cv2.VideoWriter(r1_cam1_path, fourcc, 30.0, (self.width, self.height))
            self.writer_r1_cam2 = cv2.VideoWriter(r1_cam2_path, fourcc, 30.0, (self.width, self.height))
            self.create_subscription(Image, '/R1_camera1', self.r1_camera1_callback, 10)
            self.create_subscription(Image, '/R1_camera2', self.r1_camera2_callback, 10)
            self.get_logger().info(f"Recording R1 cameras to:\n  {r1_cam1_path}\n  {r1_cam2_path}")

        if self.record_r2:
            r2_cam1_filename = f"{timestamp}_R2_camera1.mp4"
            r2_cam2_filename = f"{timestamp}_R2_camera2.mp4"
            r2_cam1_path = os.path.join(self.output_dir, r2_cam1_filename)
            r2_cam2_path = os.path.join(self.output_dir, r2_cam2_filename)
            self.writer_r2_cam1 = cv2.VideoWriter(r2_cam1_path, fourcc, 30.0, (self.width, self.height))
            self.writer_r2_cam2 = cv2.VideoWriter(r2_cam2_path, fourcc, 30.0, (self.width, self.height))
            self.create_subscription(Image, '/R2_camera1', self.r2_camera1_callback, 10)
            self.create_subscription(Image, '/R2_camera2', self.r2_camera2_callback, 10)
            self.get_logger().info(f"Recording R2 cameras to:\n  {r2_cam1_path}\n  {r2_cam2_path}")

    # Callbacks for R1
    def r1_camera1_callback(self, msg):
        if self.writer_r1_cam1 is not None:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_resized = cv2.resize(frame, (self.width, self.height))
            self.writer_r1_cam1.write(frame_resized)

    def r1_camera2_callback(self, msg):
        if self.writer_r1_cam2 is not None:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_resized = cv2.resize(frame, (self.width, self.height))
            self.writer_r1_cam2.write(frame_resized)

    # Callbacks for R2
    def r2_camera1_callback(self, msg):
        if self.writer_r2_cam1 is not None:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_resized = cv2.resize(frame, (self.width, self.height))
            self.writer_r2_cam1.write(frame_resized)

    def r2_camera2_callback(self, msg):
        if self.writer_r2_cam2 is not None:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_resized = cv2.resize(frame, (self.width, self.height))
            self.writer_r2_cam2.write(frame_resized)

    def destroy_node(self):
        if self.writer_r1_cam1:
            self.writer_r1_cam1.release()
        if self.writer_r1_cam2:
            self.writer_r1_cam2.release()
        if self.writer_r2_cam1:
            self.writer_r2_cam1.release()
        if self.writer_r2_cam2:
            self.writer_r2_cam2.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Change these flags to control recording
    record_r1 = False
    record_r2 = True

    node = DualCameraRecorder(record_r1=record_r1, record_r2=record_r2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Video files saved.")

if __name__ == '__main__':
    main()

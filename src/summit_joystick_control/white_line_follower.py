#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class WhiteLineFollower(Node):
    def __init__(self):
        super().__init__('white_line_follower')

        self.bridge = CvBridge()

        # Subscribe to RGB image topic
        self.image_sub = self.create_subscription(
            Image,
            '/front_rgb',  # Your camera topic
            self.image_callback,
            10)

        # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety timer: checks if line is still visible
        self.timer = self.create_timer(0.1, self.stop_if_no_line)

        self.line_last_seen = self.get_clock().now()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # Focus on a horizontal strip across the center
        crop_height = 50  # pixels
        crop_y = height // 2 - crop_height // 2
        crop_img = cv_image[crop_y:crop_y + crop_height, :]

        # Convert to grayscale and threshold for white
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            self.line_last_seen = self.get_clock().now()

            # Find largest contour (assumed to be white line)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])  # Centroid X

                # Error from image center
                error = cx - width // 2

                # Proportional turning
                k_p = 0.005
                angular_z = -error * k_p

                # PWM-style speed control
                max_speed = 2.0
                min_speed = 0.5
                linear_x = max(min_speed, max_speed - abs(error) * 0.002)

                twist.linear.x = linear_x
                twist.angular.z = angular_z

        # Publish even if twist is zero (in case of no contour)
        self.cmd_vel_pub.publish(twist)

    def stop_if_no_line(self):
        # Stop robot if no white line detected in the last 0.5 seconds
        time_since_seen = self.get_clock().now() - self.line_last_seen
        if time_since_seen.nanoseconds > 0.5 * 1e9:
            twist = Twist()  # Zero velocity
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteLineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

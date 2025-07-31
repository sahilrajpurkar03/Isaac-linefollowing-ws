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

        # Subscriptions
        self.create_subscription(Image, '/R2_front_rgb', self.front_callback, 10)
        self.create_subscription(Image, '/R2_left_rgb', self.left_callback, 10)
        self.create_subscription(Image, '/R2_right_rgb', self.right_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/R2_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.check_line_lost)
        self.last_seen_time = self.get_clock().now()

        # PID parameters
        self.k_p = 0.008
        self.k_i = 0.00002
        self.k_d = 0.004

        self.error_sum = 0.0
        self.last_error = 0.0

        # Image placeholders
        self.front_image = None
        self.left_image = None
        self.right_image = None

        self.left_detect = None
        self.right_detect = None

        # Line visibility flags
        self.front_line_visible = False
        self.left_line_visible = False
        self.right_line_visible = False

        # Turning mode
        self.turning = None  # 'left', 'right', or None

    def front_callback(self, msg):
        self.front_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_front_image()

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left_detect, self.left_line_visible = self.process_side_image(self.left_image)

        if self.left_line_visible and self.turning is None:
            self.turning = 'left'
            self.get_logger().info('Left line detected, starting turn left.')

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right_detect, self.right_line_visible = self.process_side_image(self.right_image)

        if self.right_line_visible and self.turning is None:
            self.turning = 'right'
            self.get_logger().info('Right line detected, starting turn right.')

    def process_side_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
            return thresh_color, True
        return thresh_color, False

    def process_front_image(self):
        cv_image = self.front_image.copy()
        height, width, _ = cv_image.shape

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.front_line_visible = True
            self.last_seen_time = self.get_clock().now()

            # If front sees the line again, stop turning
            if self.turning is not None:
                self.get_logger().info(f'Front line detected. Stopping turn: {self.turning}')
            self.turning = None
        else:
            self.front_line_visible = False

        # Decide behavior
        if self.turning is not None:
            self.execute_turn()
        elif self.front_line_visible:
            self.execute_line_follow(thresh_color)
        else:
            self.cmd_vel_pub.publish(Twist())  # Stop

        self.show_combined_view(thresh_color)

    def execute_turn(self):
        twist = Twist()
        if self.turning == 'left':
            twist.angular.z = -4.0
        elif self.turning == 'right':
            twist.angular.z = 4.0
        self.cmd_vel_pub.publish(twist)

    def execute_line_follow(self, display_img):
        height, _ = display_img.shape[:2]

        gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                error = cy - height // 2
                self.error_sum += error
                delta_error = error - self.last_error
                self.error_sum = max(min(self.error_sum, 10000), -10000)
                angular_z = (self.k_p * error) + (self.k_i * self.error_sum) + (self.k_d * delta_error)
                self.last_error = error

                base_speed = 2.0
                max_deviation = height / 2
                twist.linear.x = base_speed * (1 - min(abs(error) / max_deviation, 0.8))
                twist.angular.z = angular_z

                # Visual feedback
                cv2.drawContours(display_img, [largest], -1, (0, 255, 0), 2)
                cv2.circle(display_img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(display_img, (0, height // 2), (display_img.shape[1], height // 2), (255, 0, 0), 2)
                cv2.putText(display_img, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)

        self.cmd_vel_pub.publish(twist)

    def show_combined_view(self, front_display):
        def resize_and_label(img, label, size):
            if img is None:
                img = np.zeros(size + (3,), dtype=np.uint8)
            img_resized = cv2.resize(img, size)
            cv2.rectangle(img_resized, (0, 0), (img_resized.shape[1], 30), (50, 50, 50), -1)
            cv2.putText(img_resized, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2)
            return img_resized

        scale = 0.3
        target_height = int(front_display.shape[0] * scale)
        target_width = int(front_display.shape[1] * scale)
        size = (target_width, target_height)

        left = resize_and_label(self.left_detect, "Left Detection", size)
        front = resize_and_label(front_display, "Front (PID)", size)
        right = resize_and_label(self.right_detect, "Right Detection", size)

        combined = np.hstack((left, front, right))
        cv2.imshow("White Line Follower Debug", combined)
        cv2.waitKey(1)

    def check_line_lost(self):
        # Optional safety timeout
        time_since_seen = self.get_clock().now() - self.last_seen_time
        if time_since_seen.nanoseconds > 1.0 * 1e9 and self.front_line_visible is False:
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WhiteLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

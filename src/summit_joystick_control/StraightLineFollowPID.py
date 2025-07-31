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

        self.image_sub = self.create_subscription(
            Image,
            '/R2_front_rgb',
            self.image_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/R2_cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.check_line_lost)
        self.last_seen_time = self.get_clock().now()

        # PID parameters
        self.k_p = 0.008
        self.k_i = 0.00001
        self.k_d = 0.004

        self.error_sum = 0.0
        self.last_error = 0.0

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)

        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            self.last_seen_time = self.get_clock().now()

            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                error = cy - height // 2

                # --- PID Controller ---
                self.error_sum += error
                delta_error = error - self.last_error

                # Optional: Integral windup protection
                self.error_sum = max(min(self.error_sum, 10000), -10000)

                angular_z = (self.k_p * error) + (self.k_i * self.error_sum) + (self.k_d * delta_error)

                self.last_error = error

                # Set velocities
                base_speed = 2.0
                max_deviation = height / 2
                twist.linear.x = base_speed * (1 - min(abs(error) / max_deviation, 0.8))
                twist.angular.z = angular_z

                # Debug drawing
                cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
                cv2.circle(thresh_color, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(thresh_color, (0, height // 2), (width, height // 2), (255, 0, 0), 2)
                cv2.putText(thresh_color, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            # No line detected
            self.error_sum = 0.0  # Reset integral term if no line is visible
            self.last_error = 0.0

        self.cmd_vel_pub.publish(twist)

        # Display
        scale = 0.5
        thresh_resized = cv2.resize(thresh_color, (0, 0), fx=scale, fy=scale)

        def add_label(img, label):
            labeled = img.copy()
            cv2.rectangle(labeled, (0, 0), (labeled.shape[1], 30), (50, 50, 50), -1)
            cv2.putText(labeled, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2)
            return labeled

        labeled_thresh = add_label(thresh_resized, "Threshold + PID Control")
        cv2.imshow("White Line Follower Debug", labeled_thresh)
        cv2.waitKey(1)

    def check_line_lost(self):
        time_since_seen = self.get_clock().now() - self.last_seen_time
        if time_since_seen.nanoseconds > 0.5 * 1e9:
            self.cmd_vel_pub.publish(Twist())  # Stop if no line seen

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

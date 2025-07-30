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
            '/front_rgb',
            self.image_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.check_line_lost)
        self.last_seen_time = self.get_clock().now()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # Convert to grayscale and threshold
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        # Work only with thresholded image
        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            self.last_seen_time = self.get_clock().now()

            # Use the largest white contour
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Compute vertical error from image center
                error = cy - height // 2

                # Control logic for horizontal line following
                k_p = 0.004
                twist.angular.z = k_p * error  # Turning depends on vertical misalignment

                base_speed = 1.2
                max_deviation = height / 2
                twist.linear.x = base_speed * (1 - min(abs(error) / max_deviation, 0.8))

                # Draw on thresholded image
                cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
                cv2.circle(thresh_color, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(thresh_color, (0, height // 2), (width, height // 2), (255, 0, 0), 2)  # Horizontal reference line
                cv2.putText(thresh_color, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            # No white line detected
            pass

        # Publish twist
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

        labeled_thresh = add_label(thresh_resized, "Threshold + Horizontal Line Detection")
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

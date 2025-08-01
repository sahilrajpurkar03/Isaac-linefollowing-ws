#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
np.float = float  # temporary fix for transforms3d compatibility
from tf_transformations import euler_from_quaternion


class WhiteLineFollower(Node):
    def __init__(self):
        super().__init__('white_line_follower')

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/R2_front_rgb', self.front_callback, 10)
        self.create_subscription(Image, '/R2_left_rgb', self.left_callback, 10)
        self.create_subscription(Image, '/R2_right_rgb', self.right_callback, 10)
        self.create_subscription(Imu, '/R2_imu', self.imu_callback, 10)

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

        # Turning state
        self.turning = None  # 'left' or 'right'
        self.turning_in_progress = False
        self.target_yaw = None
        self.current_yaw = None

        self.state = 'FOLLOW_LINE'
        self.junction_detected = False
        self.junction_crossing_timer = None
        self.junction_cross_count = 0


    # ========== CALLBACKS ==========
    def front_callback(self, msg):
        self.front_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_front_image()

    def left_callback(self, msg):
        if self.state == 'TURNING'or self.junction_detected:
            return  # Ignore while turning

        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left_detect, self.left_line_visible = self.process_side_image(self.left_image)

        if self.left_line_visible and self.state == 'FOLLOW_LINE':
            self.get_logger().info('Left line detected. Preparing to turn left 90 degrees.')
            self.state = 'TURNING'
            self.turning = 'left'
            self.start_turn('left')


    def right_callback(self, msg):
        if self.state == 'TURNING'or self.junction_detected:
            return  # Ignore while turning

        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right_detect, self.right_line_visible = self.process_side_image(self.right_image)

        if self.right_line_visible and self.state == 'FOLLOW_LINE':
            self.get_logger().info('Right line detected. Preparing to turn right 90 degrees.')
            self.state = 'TURNING'
            self.turning = 'right'
            self.start_turn('right')

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw

    # ========== IMAGE PROCESSING ==========

    def process_side_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 50000  # ⬅️ Increase this if still too sensitive

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > min_area:
                cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
                return thresh_color, True

        return thresh_color, False
    
    def detect_junction(self, thresh_img):
        height, width = thresh_img.shape[:2]
        roi = thresh_img[int(height*0.6):height, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        junction_contour_count = 0
        min_junction_area = 10000

        for cnt in contours:
            if cv2.contourArea(cnt) > min_junction_area:
                junction_contour_count += 1

        if junction_contour_count >= 2 and not self.junction_detected:
            self.junction_detected = True
            self.get_logger().info("🚦 Junction detected! Pausing side detections.")

            # Start a timer to resume side detections after some time (e.g., 3 seconds)
            if self.junction_crossing_timer is None:
                self.junction_crossing_timer = self.create_timer(5.0, self.end_junction_crossing)

            return True
        return False
    
    def end_junction_crossing(self):
        self.junction_detected = False
        self.junction_crossing_timer.cancel()
        self.junction_crossing_timer = None
        self.junction_cross_count += 1
        self.get_logger().info(f"✅ Junction crossed count: {self.junction_cross_count}")

        # if self.junction_cross_count >= 3:
        #     self.get_logger().info("⛔ Passed 2 junctions, stopping robot.")
        #     self.cmd_vel_pub.publish(Twist())  # Stop robot


    def process_front_image(self):
        if self.front_image is None:
            return
        
        # if self.junction_cross_count >= 3:
        #     self.get_logger().info("⛔ Junction limit reached. Stopping line following.")
        #     self.cmd_vel_pub.publish(Twist())  # Stop robot
        #     return        

        if self.state == 'TURNING':
            # During turning, ignore front line following, just do turn logic
            self.execute_turn_to_target()
            # Optionally, show blank or turning visualization
            blank_img = np.zeros_like(self.front_image)
            self.show_combined_view(blank_img)
            return

        cv_image = self.front_image.copy()
        height, width, _ = cv_image.shape

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.detect_junction(thresh)

        if contours:
            self.front_line_visible = True
            self.last_seen_time = self.get_clock().now()

            if self.turning is not None:
                self.get_logger().info(f'Front line detected again. Cancelling turn: {self.turning}')
            self.turning = None
        else:
            self.front_line_visible = False

        if self.turning_in_progress:
            self.execute_turn_to_target()
        elif self.front_line_visible:
            self.execute_line_follow(thresh_color)
        else:
            self.cmd_vel_pub.publish(Twist())  # Stop

        self.show_combined_view(thresh_color)


    # ========== TURNING LOGIC ==========

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def start_turn(self, direction):
        if self.current_yaw is None:
            self.get_logger().warn("IMU data not available yet!")
            return

        self.turning_in_progress = True
        if direction == 'right':
            angle_offset = -math.radians(80)  # Right = clockwise = negative
        else:
            angle_offset = math.radians(80)   # Left = counter-clockwise = positive

        self.target_yaw = self.normalize_angle(self.current_yaw + angle_offset)

        self.get_logger().info(
            f'Starting 90-degree turn to {direction}. '
            f'Current yaw: {math.degrees(self.current_yaw):.2f}°, '
            f'Target yaw: {math.degrees(self.target_yaw):.2f}°'
        )

    def execute_turn_to_target(self):
        if self.current_yaw is None or self.target_yaw is None:
            return

        error = self.normalize_angle(self.target_yaw - self.current_yaw)
        twist = Twist()

        error_deg = math.degrees(error)
        yaw_deg = math.degrees(self.current_yaw)
        target_deg = math.degrees(self.target_yaw)

        if abs(error) > math.radians(2):  # ~2 degree threshold
            twist.angular.z = -3.0 if error > 0 else 3.0
            self.get_logger().info(
                f"Turning... Current yaw: {yaw_deg:.2f}°, "
                f"Target: {target_deg:.2f}°, "
                f"Error: {error_deg:.2f}°"
            )
            self.cmd_vel_pub.publish(twist)
        else:
            self.get_logger().info(
                f"Completed 90-degree turn to {self.turning}. "
                f"Final yaw: {yaw_deg:.2f}°"
            )
            self.turning_in_progress = False
            self.turning = None
            self.target_yaw = None
            self.state = 'FOLLOW_LINE'  # back to line following
            self.cmd_vel_pub.publish(Twist())  # Stop turning

    # ========== LINE FOLLOWING ==========

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

                self.get_logger().info(f'Line follow: error={error}, angular_z={angular_z:.2f}, linear_x={twist.linear.x:.2f}')

        self.cmd_vel_pub.publish(twist)

    # ========== VISUAL DEBUG ==========

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

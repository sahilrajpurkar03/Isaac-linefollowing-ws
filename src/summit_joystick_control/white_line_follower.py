# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class WhiteLineFollower(Node):
#     def __init__(self):
#         super().__init__('white_line_follower')

#         self.bridge = CvBridge()

#         self.image_sub = self.create_subscription(
#             Image,
#             '/front_rgb',
#             self.image_callback,
#             10)

#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.timer = self.create_timer(0.1, self.check_line_lost)
#         self.last_seen_time = self.get_clock().now()

#         self.turning = False
#         self.turn_direction = None

#     def image_callback(self, msg):
#         # Convert ROS Image to OpenCV
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         height, width, _ = cv_image.shape

#         # Convert to grayscale and threshold
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

#         # Crop upper and lower thirds of the thresholded image
#         height, width = thresh.shape
#         third = height // 3
#         upper_left_turn = thresh[0:third, :]
#         lower_right_turn = thresh[-third:, :]

#         # Function to get center box coords (red box)
#         def get_center_box(img):
#             h, w = img.shape
#             box_width = w // 5  
#             box_height = h
#             x_start = (w - box_width) // 2
#             y_start = 0
#             return (x_start, y_start, box_width, box_height)

#         upper_box = get_center_box(upper_left_turn)
#         lower_box = get_center_box(lower_right_turn)

#         def point_in_box(x, y, box):
#             x_start, y_start, box_w, box_h = box
#             return (x_start <= x <= x_start + box_w) and (y_start <= y <= y_start + box_h)

#         def line_in_right_half(x1, x2, img_width):
#             right_half_start = img_width // 2
#             return (x1 >= right_half_start and x2 >= right_half_start)

#         def detect_vertical_lines(region, region_name, box=None):
#             lines = cv2.HoughLinesP(region, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=10)
#             vertical_detected_right_half = False
#             vertical_detected_in_box = False
#             if lines is not None:
#                 h, w = region.shape
#                 for line in lines:
#                     x1, y1, x2, y2 = line[0]
#                     angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
#                     angle = abs(angle)
#                     if angle > 90:
#                         angle = 180 - angle
#                     if angle > 80:
#                         if line_in_right_half(x1, x2, w):
#                             vertical_detected_right_half = True
#                         if box and point_in_box(x1, y1, box) and point_in_box(x2, y2, box):
#                             vertical_detected_in_box = True
#                 return vertical_detected_right_half, vertical_detected_in_box
#             return False, False

#         left_right_half, left_in_box = detect_vertical_lines(upper_left_turn, "left turn", upper_box)
#         right_right_half, right_in_box = detect_vertical_lines(lower_right_turn, "right turn", lower_box)

#         twist = Twist()
#         speed_reduction_factor = 1.0

#         if left_right_half and right_right_half:
#             print("junction ahead")
#         else:
#             if left_right_half:
#                 print("left turn ahead")
#                 speed_reduction_factor = 0.7
#             if right_right_half:
#                 print("right turn ahead")
#                 speed_reduction_factor = 0.7

#         if left_in_box and right_in_box:
#             print("junction detected")
#             self.turning = False
#             self.turn_direction = None
#         else:
#             if left_in_box:
#                 print("left turn")
#                 self.turning = True
#                 self.turn_direction = 'left'
#             if right_in_box:
#                 print("right turn")
#                 self.turning = True
#                 self.turn_direction = 'right'

#         contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         if self.turning:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 3.0 if self.turn_direction == 'right' else -3.0
#             self.cmd_vel_pub.publish(twist)
#             return

#         if contours:
#             self.last_seen_time = self.get_clock().now()

#             if self.turning:
#                 print("Resuming line following")
#                 self.turning = False
#                 self.turn_direction = None

#             largest = max(contours, key=cv2.contourArea)
#             M = cv2.moments(largest)

#             if M['m00'] != 0:
#                 cx = int(M['m10'] / M['m00'])
#                 cy = int(M['m01'] / M['m00'])

#                 error = cy - height // 2

#                 k_p = 0.004
#                 twist.angular.z = k_p * error

#                 base_speed = 2.0 * speed_reduction_factor
#                 max_deviation = height / 2
#                 twist.linear.x = base_speed * (1 - min(abs(error) / max_deviation, 0.8))

#                 thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
#                 cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
#                 cv2.circle(thresh_color, (cx, cy), 5, (0, 0, 255), -1)
#                 cv2.line(thresh_color, (0, height // 2), (width, height // 2), (255, 0, 0), 2)
#                 cv2.putText(thresh_color, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
#         else:
#             twist = Twist()
#             thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

#         upper_color = cv2.cvtColor(upper_left_turn, cv2.COLOR_GRAY2BGR)
#         lower_color = cv2.cvtColor(lower_right_turn, cv2.COLOR_GRAY2BGR)

#         cv2.rectangle(upper_color,
#                       (upper_box[0], upper_box[1]),
#                       (upper_box[0] + upper_box[2], upper_box[1] + upper_box[3]),
#                       (0, 0, 255), 2)

#         cv2.rectangle(lower_color,
#                       (lower_box[0], lower_box[1]),
#                       (lower_box[0] + lower_box[2], lower_box[1] + lower_box[3]),
#                       (0, 0, 255), 2)

#         scale = 0.25
#         upper_resized = cv2.resize(upper_color, (int(width * scale), int(third * scale)))
#         lower_resized = cv2.resize(lower_color, (int(width * scale), int(third * scale)))
#         center_resized = cv2.resize(thresh_color, (0, 0), fx=scale, fy=scale)

#         def add_label(img, label):
#             labeled = img.copy()
#             cv2.rectangle(labeled, (0, 0), (labeled.shape[1], 30), (50, 50, 50), -1)
#             cv2.putText(labeled, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
#                         0.7, (255, 255, 255), 2)
#             return labeled

#         upper_labeled = add_label(upper_resized, "Upper Left Turn Region")
#         center_labeled = add_label(center_resized, "Center Straight Line Region")
#         lower_labeled = add_label(lower_resized, "Lower Right Turn Region")

#         stacked = cv2.vconcat([upper_labeled, center_labeled, lower_labeled])
#         cv2.imshow("White Line Follower Debug", stacked)
#         cv2.waitKey(1)

#         self.cmd_vel_pub.publish(twist)

#     def check_line_lost(self):
#         time_since_seen = self.get_clock().now() - self.last_seen_time
#         if time_since_seen.nanoseconds > 0.5 * 1e9:
#             self.cmd_vel_pub.publish(Twist())

# def main(args=None):
#     rclpy.init(args=args)
#     node = WhiteLineFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

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

        self.turning = False
        self.turn_direction = None
        self.turn_start_time = None
        self.turn_duration = 1.0  # seconds for 90 degree turn

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        height, width = thresh.shape
        third = height // 3
        upper_left_turn = thresh[0:third, :]
        lower_right_turn = thresh[-third:, :]

        def get_center_box(img):
            h, w = img.shape
            box_width = w // 5
            box_height = h
            x_start = (w - box_width) // 2
            y_start = 0
            return (x_start, y_start, box_width, box_height)

        upper_box = get_center_box(upper_left_turn)
        lower_box = get_center_box(lower_right_turn)

        def point_in_box(x, y, box):
            x_start, y_start, box_w, box_h = box
            return (x_start <= x <= x_start + box_w) and (y_start <= y <= y_start + box_h)

        def line_in_right_half(x1, x2, img_width):
            right_half_start = img_width // 2
            return (x1 >= right_half_start and x2 >= right_half_start)

        def detect_vertical_lines(region, region_name, box=None):
            lines = cv2.HoughLinesP(region, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=10)
            vertical_detected_right_half = False
            vertical_detected_in_box = False
            if lines is not None:
                h, w = region.shape
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                    angle = abs(angle)
                    if angle > 90:
                        angle = 180 - angle
                    if angle > 80:
                        if line_in_right_half(x1, x2, w):
                            vertical_detected_right_half = True
                        if box and point_in_box(x1, y1, box) and point_in_box(x2, y2, box):
                            vertical_detected_in_box = True
                return vertical_detected_right_half, vertical_detected_in_box
            return False, False

        left_right_half, left_in_box = detect_vertical_lines(upper_left_turn, "left turn", upper_box)
        right_right_half, right_in_box = detect_vertical_lines(lower_right_turn, "right turn", lower_box)

        twist = Twist()
        speed_reduction_factor = 1.0

        if left_right_half and right_right_half:
            print("junction ahead")
        else:
            if left_right_half:
                print("left turn ahead")
                speed_reduction_factor = 0.7
            if right_right_half:
                print("right turn ahead")
                speed_reduction_factor = 0.7

        if self.turning:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 3.0 if self.turn_direction == 'right' else -3.0
            if time.time() - self.turn_start_time < self.turn_duration:
                self.cmd_vel_pub.publish(twist)
                return
            else:
                print("Turn complete, resuming line following")
                self.turning = False
                self.turn_direction = None

        if left_in_box and right_in_box:
            print("junction detected")
            self.turning = False
            self.turn_direction = None
        else:
            if left_in_box:
                print("left turn")
                self.turning = True
                self.turn_direction = 'left'
                self.turn_start_time = time.time()
                return
            if right_in_box:
                print("right turn")
                self.turning = True
                self.turn_direction = 'right'
                self.turn_start_time = time.time()
                return

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.last_seen_time = self.get_clock().now()

            if self.turning:
                print("Resuming line following")
                self.turning = False
                self.turn_direction = None

            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                error = cy - height // 2

                k_p = 0.005
                twist.angular.z = k_p * error

                base_speed = 2.5 * speed_reduction_factor
                max_deviation = height / 2
                twist.linear.x = base_speed * (1 - min(abs(error) / max_deviation, 0.8))

                thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(thresh_color, [largest], -1, (0, 255, 0), 2)
                cv2.circle(thresh_color, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(thresh_color, (0, height // 2), (width, height // 2), (255, 0, 0), 2)
                cv2.putText(thresh_color, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            twist = Twist()
            thresh_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        upper_color = cv2.cvtColor(upper_left_turn, cv2.COLOR_GRAY2BGR)
        lower_color = cv2.cvtColor(lower_right_turn, cv2.COLOR_GRAY2BGR)

        cv2.rectangle(upper_color,
                      (upper_box[0], upper_box[1]),
                      (upper_box[0] + upper_box[2], upper_box[1] + upper_box[3]),
                      (0, 0, 255), 2)

        cv2.rectangle(lower_color,
                      (lower_box[0], lower_box[1]),
                      (lower_box[0] + lower_box[2], lower_box[1] + lower_box[3]),
                      (0, 0, 255), 2)

        scale = 0.25
        upper_resized = cv2.resize(upper_color, (int(width * scale), int(third * scale)))
        lower_resized = cv2.resize(lower_color, (int(width * scale), int(third * scale)))
        center_resized = cv2.resize(thresh_color, (0, 0), fx=scale, fy=scale)

        def add_label(img, label):
            labeled = img.copy()
            cv2.rectangle(labeled, (0, 0), (labeled.shape[1], 30), (50, 50, 50), -1)
            cv2.putText(labeled, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2)
            return labeled

        upper_labeled = add_label(upper_resized, "Upper Left Turn Region")
        center_labeled = add_label(center_resized, "Center Straight Line Region")
        lower_labeled = add_label(lower_resized, "Lower Right Turn Region")

        stacked = cv2.vconcat([upper_labeled, center_labeled, lower_labeled])
        cv2.imshow("White Line Follower Debug", stacked)
        cv2.waitKey(1)

        self.cmd_vel_pub.publish(twist)

    def check_line_lost(self):
        time_since_seen = self.get_clock().now() - self.last_seen_time
        if time_since_seen.nanoseconds > 0.5 * 1e9:
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

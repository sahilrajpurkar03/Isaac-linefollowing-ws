#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Toggle with A button (button 0)
        self.enabled = False
        self.last_button_state = 0

        # Desired scaling from joystick [-1,1] → [-3,3]
        self.linear_scale = 2.0
        self.angular_scale = 3.0

    def joy_callback(self, msg):
        button_a = msg.buttons[0]
        axis_linear = msg.axes[1]    # Left stick vertical
        axis_angular = msg.axes[3]   # Right stick horizontal

        self.get_logger().info(f"Raw joystick axes: linear={axis_linear:.2f}, angular={axis_angular:.2f}")

        # Toggle enable state with A button
        if button_a == 1 and self.last_button_state == 0:
            self.enabled = not self.enabled
            self.get_logger().info(f"Controller {'ENABLED' if self.enabled else 'DISABLED'}")
        self.last_button_state = button_a

        twist = Twist()

        if self.enabled:

            twist.linear.x = self.linear_scale * axis_linear 
            twist.angular.z = self.angular_scale * axis_angular 

            # Debug output
            self.get_logger().info(f"[ENABLED] Joy: lin={axis_linear:.2f} ang={axis_angular:.2f} → cmd_vel: x={twist.linear.x:.2f}, z={twist.angular.z:.2f}")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("[DISABLED] Sending zero velocity.")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

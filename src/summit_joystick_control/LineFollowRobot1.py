#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import gym
from gym import spaces
from stable_baselines3 import PPO
import os

class LineFollowEnv(gym.Env):
    def __init__(self):
        super(LineFollowEnv, self).__init__()
        self.observation_space = spaces.Box(low=0, high=255, shape=(84, 84, 3), dtype=np.uint8)
        self.action_space = spaces.Discrete(3)  # [left, forward, right]

    def reset(self):
        # Reset simulation if needed
        obs = np.zeros((84, 84, 3), dtype=np.uint8)
        return obs

    def step(self, action):
        # Apply action to the robot here if sim
        obs = np.zeros((84, 84, 3), dtype=np.uint8)
        reward = 1.0  # Dummy reward
        done = False
        info = {}
        return obs, reward, done, info

def train_rl_model():
    env = LineFollowEnv()
    model = PPO('CnnPolicy', env, verbose=1)
    model.learn(total_timesteps=10000)
    model.save("ppo_line_follower")

class RLLineFollower(Node):
    def __init__(self):
        super().__init__('rl_line_follower')

        self.bridge = CvBridge()
        self.width = 1280
        self.height = 720
        self.latest_ang_vel = (0.0, 0.0, 0.0)
        self.latest_lin_acc = (0.0, 0.0, 0.0)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/front_rgb', self.image_callback, 10)
        self.create_subscription(Imu, '/R1_imu', self.imu_callback, 10)

        # RL Model
        self.model = PPO.load(os.path.expanduser('~/models/ppo_line_follower'))

        # RL Visualization
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.rl_writer = cv2.VideoWriter('robot1_rl_window.mp4', fourcc, 30.0, (self.width, self.height))

    def imu_callback(self, msg):
        self.latest_ang_vel = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        )
        self.latest_lin_acc = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        resized_frame = cv2.resize(frame, (84, 84))
        obs = np.array(resized_frame, dtype=np.uint8)
        action, _ = self.model.predict(obs, deterministic=True)

        twist = Twist()
        if action == 0:
            twist.angular.z = 0.5  # Turn Left
        elif action == 1:
            twist.linear.x = 0.5  # Forward
        elif action == 2:
            twist.angular.z = -0.5  # Turn Right

        self.cmd_vel_pub.publish(twist)
        self.rl_visualization(frame, action)

    def rl_visualization(self, frame, action):
        frame_resized = cv2.resize(frame, (self.width, self.height))

        # Action label
        action_str = ["Turn Left", "Forward", "Turn Right"][action]

        # Draw info box and text
        cv2.rectangle(frame_resized, (10, 10), (400, 130), (0, 0, 0), -1)
        cv2.putText(frame_resized, "RL Prediction", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame_resized, f"Action: {action_str}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        ax, ay, az = self.latest_lin_acc
        gx, gy, gz = self.latest_ang_vel
        cv2.putText(frame_resized, f"Lin Acc: [{ax:.2f}, {ay:.2f}, {az:.2f}]", (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 200, 255), 1)
        cv2.putText(frame_resized, f"Ang Vel: [{gx:.2f}, {gy:.2f}, {gz:.2f}]", (20, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 200, 255), 1)

        # Draw action arrow
        center = (self.width // 2, self.height // 2)
        if action_str == "Turn Left":
            arrow_end = (center[0] - 100, center[1])
        elif action_str == "Turn Right":
            arrow_end = (center[0] + 100, center[1])
        else:
            arrow_end = (center[0], center[1] - 100)
        cv2.arrowedLine(frame_resized, center, arrow_end, (255, 0, 0), 4, tipLength=0.3)

        # Show scaled down window
        small_frame = cv2.resize(frame_resized, (0, 0), fx=0.25, fy=0.25)
        cv2.imshow("RL Visualization", small_frame)
        cv2.waitKey(1)

        self.rl_writer.write(frame_resized)

def main(args=None):
    rclpy.init(args=args)
    train = False  # Set True to train model
    if train:
        train_rl_model()
    else:
        node = RLLineFollower()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            node.rl_writer.release()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
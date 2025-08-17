# Robocon-Lidar-ws

![Image](doc/competition.gif)

**Robocon-Lidar-ws** is a virtual robotics racing simulation built using **Isaac Sim** and **ROS2**, inspired by the Robocon competition. Two **Robotnik Summit** robots race by following a white line path while avoiding obstacles.

- **Robot 1**: Uses **PID control** for line following.
- **Robot 2**: Uses **Reinforcement Learning (RL)**, improving through trial and error.

It’s a friendly race between classic control theory and intelligent learning. Who finishes first?

> **Tip**: You can run your own control code on the robots! All sensors and ROS2 topics are ready—just change the logic and experiment.

## 📦 Requirements

- [Isaac Sim](https://developer.nvidia.com/isaac-sim)
- ROS2 (Foxy or newer)
- Python 3.8+
- NVIDIA GPU (for Isaac Sim + RL training)
- `ros2_control`, `joy`, and other ROS2 packages

## 🤖 Robot Control Commands

### PID Line Following
```bash
cd src/summit_joystick_control
python3 LineFollowRobot2.py
```

### Reinforcement Learning Line Following(directory for RL training and model is not updated yet)

### Direct Velocity Control
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Joystick Control
Make sure your joystick is connected:
```bash
ros2 launch summit_joystick_control joystick_launch.py
```

## 🎥 Recording

### Record Camera Feed to MP4
```bash
cd src/summit_joystick_control
python3 recordingpy
```
## 🛠️ Debugging

### Check Joystick Input
```bash
ros2 topic echo /joy
```

### Check Velocity Commands
```bash
ros2 topic echo /cmd_vel
```

## 🤖 Robot Model Credit

Robotnik Summit robot model used in simulation:  
🔗 https://robotnik.eu/products/mobile-robots/rb-summit/

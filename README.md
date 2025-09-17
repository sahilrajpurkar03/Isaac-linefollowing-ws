# Isaac-Linefollowing-ws

![Image](competition.gif)

**Robocon-Lidar-ws** is a virtual robotics racing simulation built using **Isaac Sim** and **ROS2**, inspired by the Robocon competition. Two **Robotnik Summit** robots race by following a white line path while avoiding obstacles.

> **Tip**: You can run your own control code on the robots! All sensors and ROS2 topics are ready—just change the logic and experiment.

## 📦 Requirements

- [Isaac Sim](https://developer.nvidia.com/isaac-sim)
- ROS2 (Foxy or newer)
- Python 3.8+
- `ros2_control`, `joy`, and other ROS2 packages

## 🤖 Robot Control Commands

### PID Line Following
```bash
cd src/summit_joystick_control
python3 LineFollowRobot2.py
```

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

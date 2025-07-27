# Robocon-Lidar-ws

direct command for movement 

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

lanuch project 
```bash
ros2 launch summit_joystick_control joystick_launch.py
```

debug
```bash
ros2 topic echo /joy

ros2 topic echo /cmd_val
```
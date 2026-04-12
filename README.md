# ece-484-f1tenth

F1Tenth autonomous racing simulation with ROS2 and OpenAI Gym.

## Prerequisites

- ROS2 (Foxy or later)
- Python 3.8+
- colcon build tools

Install ROS2 dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs \
                 ros-${ROS_DISTRO}-nav2-map-server \
                 ros-${ROS_DISTRO}-nav2-lifecycle-manager \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-teleop-twist-keyboard
```

Install the f1tenth_gym Python package:
```bash
cd src/f1tenth_simulator/f1tenth_gym
pip install -e .
```

## Build

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/Documents/ECE484/ece-484-f1tenth
colcon build
source install/setup.bash
```

To build only the autonomy stack packages:
```bash
colcon build --packages-select planning controls
source install/setup.bash
```

## Run

### Terminal 1: Start the simulator
```bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: Start the autonomy stack
```bash
source install/setup.bash
ros2 launch planning planning_launch.py &
ros2 launch controls controls_launch.py
```

## Package Structure

```
src/
  f1tenth_simulator/       # Simulator (third-party, do not modify)
    f1tenth_gym/           # Core gym environment
    f1tenth_gym_ros/       # ROS2 bridge
    gym/                   # OpenAI Gym
  planning/                # Path planning ROS2 package
  controls/                # Vehicle controls ROS2 package
```

## Topic Graph

```
gym_bridge ──/ego_racecar/scan──► planning_node ──/planning/trajectory──► controls_node ──/ego_racecar/drive──► gym_bridge
             /ego_racecar/odom──► planning_node
             /ego_racecar/odom──────────────────────────────────────────► controls_node
```

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/ego_racecar/scan` | `sensor_msgs/LaserScan` | 1080-beam LiDAR scan |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Vehicle pose and velocity |
| `/planning/trajectory` | `nav_msgs/Path` | Planned path (viewable in rviz2) |
| `/ego_racecar/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering + speed command |
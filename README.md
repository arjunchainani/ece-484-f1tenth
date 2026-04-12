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

Install Python dependencies:
```bash
pip install scipy scikit-image Pillow pyyaml numpy
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

## Configuration

The simulator map and spawn pose are configured in `src/f1tenth_simulator/f1tenth_gym_ros/config/sim.yaml`. The planning and controls nodes read their parameters from:

- `src/planning/config/planning_params.yaml` — map selection, velocity planner limits, XTE penalty
- `src/controls/config/controls_params.yaml` — wheelbase, pure pursuit lookahead, steering limits

The `map_path` and spawn pose (`sx`, `sy`, `stheta`) in `planning_params.yaml` should match `sim.yaml`.

Available maps: `levine`, `Spielberg_map`, `square`.

## Package Structure

```
src/
  f1tenth_simulator/           # Simulator (third-party, do not modify)
    f1tenth_gym/               # Core gym environment
    f1tenth_gym_ros/           # ROS2 bridge
    gym/                       # OpenAI Gym
  planning/                    # Path planning ROS2 package
    planning/
      centerline.py            # Map -> centerline extraction (skeletonize + spline)
      min_curvature.py         # Minimum curvature path optimizer (QP via L-BFGS-B)
      planning_node.py         # ROS2 node: raceline generation + 4-pass velocity planner
      utils.py                 # closest_point_on_path, cross-track error
    config/
      planning_params.yaml
    launch/
      planning_launch.py
  controls/                    # Vehicle controls ROS2 package
    controls/
      controls_node.py         # ROS2 node: pure pursuit + velocity following
      utils.py                 # quaternion_to_yaw, normalize_angle
    config/
      controls_params.yaml
    launch/
      controls_launch.py
```

## Architecture

### Path Generation Pipeline

On startup, the planning node automatically generates a racing line from the selected map:

1. **Centerline extraction** — loads the map occupancy grid PNG, skeletonizes the free space, prunes dead-end branches, and orders the remaining cycle into a continuous loop starting near the spawn point. The result is smoothed with a periodic cubic spline and resampled at uniform spacing.

2. **Track width computation** — at each centerline point, rays are cast along the left and right normals to measure distance to the nearest wall.

3. **Minimum curvature optimization** — parameterizes the path as lateral offsets from the centerline, formulates a box-constrained QP (minimize total squared curvature, subject to staying within track boundaries minus half vehicle width), and solves with `scipy.optimize.minimize` (L-BFGS-B).

### Velocity Planning (20 Hz)

At runtime, the planning node runs a 4-pass velocity planner on the raceline:

1. **Curvature limit** — `v = sqrt(a_lat_max / kappa)` at each waypoint
2. **Backward deceleration pass** — enforces braking constraints via friction ellipse coupling
3. **Forward acceleration pass** — enforces acceleration constraints from current velocity
4. **XTE penalty** — reduces speed when the car drifts off the planned line

### Pure Pursuit Controller (50 Hz)

The controls node tracks the planned trajectory using pure pursuit with dynamic lookahead:

- Lookahead distance scales linearly with velocity: `ld = k * v + lfc`
- Lookahead point found via circle-line intersection on the path
- Steering: `delta = atan2(2 * L * sin(alpha), ld)`
- Target velocity read from the trajectory (encoded in `pose.position.z`)

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
| `/planning/trajectory` | `nav_msgs/Path` | Planned path with velocity (z = speed) |
| `/ego_racecar/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering + speed command |

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Wheelbase | 0.3302 m | F1Tenth axle distance |
| Max steering | ±0.4189 rad | ~±24 degrees |
| Max speed | 10.0 m/s | Conservative limit (sim supports up to 20) |
| Min speed | 1.5 m/s | Floor for tight corners |
| Lateral accel limit | 6.0 m/s² | Grip limit for velocity planning |
| Lookahead gain | 0.5 | Pure pursuit ld = 0.5*v + 0.5 |
| Vehicle width | 0.31 m | Track boundary margin = 0.155 m |

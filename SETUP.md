# f1tenth Simulator — Setup & Run Guide

End-to-end instructions to go from a clean Ubuntu 22.04 install to a running
`f1tenth_gym_ros` simulator. The automated `setup_f1tenth.sh` covers most
steps; the manual pieces below fill in the gaps it doesn't handle.

## 1. Prerequisites

- Ubuntu 22.04 (Jammy)
- A normal user account with `sudo` privileges (do **not** run the script as root)
- Internet access (the install pulls ~2 GB of ROS 2 packages)

## 2. Clone the repo

```bash
cd ~/Documents
git clone <repo-url> ece-484-f1tenth
cd ece-484-f1tenth
```

## 3. Run the automated setup

```bash
chmod +x setup_f1tenth.sh
./setup_f1tenth.sh
```

This installs ROS 2 Humble, creates a Python venv at `.venv/`, installs the
local `gym` and `f1tenth_gym` packages editable, and appends source lines to
`~/.bashrc`.

If the script stops partway (e.g. `set -euo pipefail` aborts on an apt
failure), re-run it — it's idempotent. If `~/.bashrc` ends up without the
three source lines at the bottom, the script exited before step 7; finish
the steps below by hand.

## 4. Install workspace ROS deps (rosdep)

`rosdep` resolves packages declared in each `package.xml`. A few may fail
silently inside the script; install them explicitly:

```bash
sudo apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-nav2-lifecycle-manager
sudo pip3 install -U transforms3d
```

Then verify the workspace is clean:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --simulate
```

The simulate flag should produce no output if all deps are satisfied.

## 5. Install the `gym_bridge` Python deps (system Python)

**Important:** `gym_bridge`'s entry-point script has a hard-coded shebang
`#!/usr/bin/python3`, so it runs under **system Python**, not the venv.
All of `f110_gym`'s runtime deps must be visible to `/usr/bin/python3`.
Install them into your user site (no sudo, visible to system Python via
`~/.local`):

```bash
/usr/bin/python3 -m pip install --user \
    "numba>=0.55.2" \
    "pyglet<1.5" \
    pyopengl \
    "coverage>=7.4" \
    "numpy~=1.26.0"
```

Why each one:

- `numba` — required by `f110_gym/envs/base_classes.py`
- `pyglet<1.5` / `pyopengl` — required by `f110_gym` rendering
- `coverage>=7.4` — `numba>=0.65` imports `coverage.types`, which only exists
  in coverage 7+ (Ubuntu ships 6.2 in apt)
- `numpy~=1.26.0` — numba's installer will otherwise pull in numpy 2.x, which
  breaks the system scipy ABI; re-pin to match `f110_gym`'s spec

## 6. Build the workspace

```bash
cd ~/Documents/ece-484-f1tenth
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--symlink-install` lets you edit Python sources in `src/` without rebuilding.

## 7. Verify `~/.bashrc`

The last lines of `~/.bashrc` should be:

```bash
# --- f1tenth_simulator setup ---
source /opt/ros/humble/setup.bash
source /home/<you>/Documents/ece-484-f1tenth/install/setup.bash
source /home/<you>/Documents/ece-484-f1tenth/.venv/bin/activate
```

If missing, append them (adjust the path to your clone).

## 8. Run the simulator

Open a **new** terminal so `~/.bashrc` re-sources, then:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

You should see RViz with the Levine map and the ego vehicle. In another
terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

to drive the car with the keyboard.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `ros2: command not found` | `~/.bashrc` doesn't source ROS. Reopen terminal or `source /opt/ros/humble/setup.bash`. |
| `ModuleNotFoundError: numba` (from `gym_bridge`) | Step 5 wasn't done. |
| `AttributeError: module 'coverage' has no attribute 'types'` | coverage < 7. Run `pip install --user "coverage>=7.4"`. |
| `ImportError: numpy.core.multiarray failed to import` | numpy 2.x shadowing system scipy. `pip install --user "numpy~=1.26.0"`. |
| rosdep fails with `sudo: a terminal is required` | Prime sudo in your shell first (`sudo -v`) before running the command in a context that can't prompt. |
| `colcon build` not finding packages | Re-source `/opt/ros/humble/setup.bash` before building. |

## Layout reference

```
ece-484-f1tenth/
├── setup_f1tenth.sh          # automated setup
├── SETUP.md                  # this guide
├── src/f1tenth_simulator/    # gym, f1tenth_gym, f1tenth_gym_ros
├── build/  install/  log/    # colcon outputs
└── .venv/                    # Python venv (used by the local editable installs)
```

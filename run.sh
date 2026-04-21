#!/usr/bin/env bash
# =============================================================================
# F1Tenth Autonomy Stack — Install, Build, and Run
#
# Usage:
#   ./run.sh install    Install all dependencies
#   ./run.sh build      Build ROS2 workspace
#   ./run.sh sim        Start the simulator only
#   ./run.sh auto       Start the autonomy stack (lane_follower)
#   ./run.sh all        Start simulator + autonomy stack together
#   ./run.sh clean      Remove build artifacts
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- helpers ----------------------------------------------------------

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

check_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f /opt/ros/*/setup.bash ]; then
            # shellcheck disable=SC1090
            source /opt/ros/*/setup.bash
        else
            error "ROS2 not found. Source your ROS2 setup.bash first."
        fi
    fi
    info "Using ROS2 distro: $ROS_DISTRO"
}

# ---------- commands ---------------------------------------------------------

cmd_install() {
    info "Installing dependencies..."

    check_ros

    # ROS2 packages
    info "Installing ROS2 dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-teleop-twist-keyboard

    # Python packages
    info "Installing Python dependencies..."
    pip install scipy scikit-image Pillow pyyaml numpy numba

    # f1tenth_gym
    info "Installing f1tenth_gym..."
    pip install -e src/f1tenth_simulator/f1tenth_gym

    info "Dependencies installed successfully."
}

cmd_build() {
    check_ros

    info "Building workspace..."
    colcon build --symlink-install
    info "Build complete. Run 'source install/setup.bash' or use './run.sh all'."
}

cmd_sim() {
    check_ros
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || error "Workspace not built. Run './run.sh build' first."

    info "Starting simulator..."
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
}

cmd_auto() {
    check_ros
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || error "Workspace not built. Run './run.sh build' first."

    info "Starting autonomy stack (lane_follower)..."
    ros2 launch lane_follower lane_follower_launch.py &
    LF_PID=$!
    info "lane_follower PID: $LF_PID"

    trap "kill $LF_PID 2>/dev/null; exit" INT TERM
    wait
}

cmd_all() {
    check_ros
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || error "Workspace not built. Run './run.sh build' first."

    info "Starting simulator + autonomy stack (lane_follower)..."

    ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
    SIM_PID=$!
    info "Simulator PID: $SIM_PID"

    # Wait for the sim's /scan topic to come up before launching the controller,
    # otherwise the lane_follower starts spinning on an empty subscription.
    info "Waiting for /ego_racecar/scan..."
    for _ in $(seq 1 20); do
        if timeout 1 ros2 topic list 2>/dev/null | grep -q '/ego_racecar/scan'; then
            break
        fi
        sleep 0.5
    done

    ros2 launch lane_follower lane_follower_launch.py &
    LF_PID=$!
    info "lane_follower PID: $LF_PID"

    trap "kill $SIM_PID $LF_PID 2>/dev/null; exit" INT TERM
    wait
}

cmd_clean() {
    info "Cleaning build artifacts..."
    rm -rf build/ install/ log/
    info "Clean complete."
}

# ---------- main -------------------------------------------------------------

case "${1:-}" in
    install) cmd_install ;;
    build)   cmd_build ;;
    sim)     cmd_sim ;;
    auto)    cmd_auto ;;
    all)     cmd_all ;;
    clean)   cmd_clean ;;
    *)
        echo "Usage: $0 {install|build|sim|auto|all|clean}"
        echo ""
        echo "  install  Install all ROS2 and Python dependencies"
        echo "  build    Build the ROS2 workspace with colcon"
        echo "  sim      Start the simulator only (rviz + gym bridge)"
        echo "  auto     Start the autonomy stack only (lane_follower)"
        echo "  all      Start simulator + autonomy stack together"
        echo "  clean    Remove build/, install/, log/ directories"
        exit 1
        ;;
esac

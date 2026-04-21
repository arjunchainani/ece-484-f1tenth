#!/usr/bin/env bash
# =============================================================================
# f1tenth_simulator setup script for Ubuntu 22.04 + ROS 2 Humble
#
# Assumes: you have already cloned the f1tenth_simulator repo (e.g. a private
# fork) somewhere on disk. Run this script FROM INSIDE the repo, or from a
# colcon workspace that contains it under src/.
#
# Usage:
#   cd /path/to/your/cloned/f1tenth_simulator   # or to the ws root
#   chmod +x setup_f1tenth.sh
#   ./setup_f1tenth.sh
#
# Notes:
#   - Run as your normal user, NOT as root. The script will sudo when needed.
#   - Assumes a clean Ubuntu 22.04.x desktop install with internet access.
#   - Idempotent-ish: safe to re-run; steps already done will skip.
# =============================================================================

set -euo pipefail

# ---------- pretty logging ----------
RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YLW=$'\033[0;33m'; BLU=$'\033[0;34m'; RST=$'\033[0m'
log()  { printf "${BLU}[*]${RST} %s\n" "$*"; }
ok()   { printf "${GRN}[+]${RST} %s\n" "$*"; }
warn() { printf "${YLW}[!]${RST} %s\n" "$*"; }
err()  { printf "${RED}[x]${RST} %s\n" "$*" >&2; }

trap 'err "failed at line $LINENO"' ERR

# ---------- sanity checks ----------
if [[ $EUID -eq 0 ]]; then
    err "Do not run as root. Run as your normal user; sudo will be invoked as needed."
    exit 1
fi

if ! grep -q "22.04" /etc/os-release; then
    warn "This script targets Ubuntu 22.04. You appear to be on:"
    grep PRETTY_NAME /etc/os-release
    read -rp "Continue anyway? [y/N] " ans
    [[ "${ans,,}" == "y" ]] || exit 1
fi

# =============================================================================
# 0. Figure out where the repo is and infer the workspace root
#
#    Supported layouts (checked in order):
#      (A) script launched from inside the repo checkout itself
#      (B) script launched from a ws root that contains src/<repo>/
#      (C) $REPO_DIR env var points directly at the repo
# =============================================================================
SCRIPT_CWD="$(pwd -P)"

find_repo() {
    # Caller-provided override wins.
    if [[ -n "${REPO_DIR:-}" ]]; then
        [[ -d "$REPO_DIR" ]] || { err "REPO_DIR=$REPO_DIR does not exist."; exit 1; }
        (cd "$REPO_DIR" && pwd -P); return
    fi

    # (A) cwd itself looks like the repo (has f1tenth_gym/ and gym/)
    if [[ -d "$SCRIPT_CWD/f1tenth_gym" && -d "$SCRIPT_CWD/gym" ]]; then
        echo "$SCRIPT_CWD"; return
    fi

    # (B) cwd is a ws root with src/.../f1tenth_gym underneath
    local hit
    hit="$(find "$SCRIPT_CWD/src" -maxdepth 2 -type d -name f1tenth_gym 2>/dev/null | head -n1 || true)"
    if [[ -n "$hit" ]]; then
        dirname "$hit"; return
    fi

    # Last-ditch: walk up from cwd looking for the repo markers
    local d="$SCRIPT_CWD"
    while [[ "$d" != "/" ]]; do
        if [[ -d "$d/f1tenth_gym" && -d "$d/gym" ]]; then echo "$d"; return; fi
        d="$(dirname "$d")"
    done

    err "Could not locate the cloned f1tenth_simulator repo."
    err "Either cd into the repo, cd into a workspace whose src/ contains it,"
    err "or re-run with REPO_DIR=/absolute/path/to/repo ./setup_f1tenth.sh"
    exit 1
}

REPO_DIR="$(find_repo)"
ok "Found repo at: ${REPO_DIR}"

# Workspace root = parent of src/, if repo sits under a src/ dir; else the repo's parent.
if [[ "$(basename "$(dirname "$REPO_DIR")")" == "src" ]]; then
    WS_DIR="$(dirname "$(dirname "$REPO_DIR")")"
else
    # Repo is not under src/ yet. colcon needs a src/ layout, so we create a ws
    # next to the repo and symlink the repo into it — nothing moves on disk.
    WS_DIR="$(dirname "$REPO_DIR")/f1tenth_ws"
    mkdir -p "$WS_DIR/src"
    LINK="$WS_DIR/src/$(basename "$REPO_DIR")"
    if [[ ! -e "$LINK" ]]; then
        ln -s "$REPO_DIR" "$LINK"
        warn "Repo is not under a src/ directory. Created workspace at:"
        warn "  $WS_DIR"
        warn "with a symlink src/$(basename "$REPO_DIR") -> $REPO_DIR"
    fi
fi
ok "Using workspace: ${WS_DIR}"

ROS_DISTRO="humble"
VENV_DIR="${WS_DIR}/.venv"

# ---------- sudo keep-alive ----------
log "Priming sudo (you'll be prompted once)…"
sudo -v
( while true; do sudo -n true; sleep 60; kill -0 "$$" 2>/dev/null || exit; done ) &
SUDO_KEEPALIVE_PID=$!
trap 'kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true' EXIT

# =============================================================================
# 1. System packages
# =============================================================================
log "Updating apt and installing base packages…"
sudo apt update
sudo apt install -y \
    build-essential git curl wget gnupg lsb-release ca-certificates \
    software-properties-common \
    python3-pip python3-dev python3-venv

sudo add-apt-repository -y universe
ok "Base packages installed."

# =============================================================================
# 2. ROS 2 Humble
# =============================================================================
if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
    ok "ROS 2 ${ROS_DISTRO} already installed — skipping install."
else
    log "Adding ROS 2 apt repository…"
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    log "Installing ROS 2 ${ROS_DISTRO} desktop + simulator deps (~2 GB)…"
    sudo apt update
    sudo apt install -y \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-dev-tools \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool
    ok "ROS 2 ${ROS_DISTRO} installed."
fi

# =============================================================================
# 3. rosdep init + update
# =============================================================================
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep…"
    sudo rosdep init
fi
log "Updating rosdep database (user-level)…"
rosdep update
ok "rosdep ready."

# =============================================================================
# 4. Python venv for the pinned gym / pyglet versions
# =============================================================================
if [[ ! -d "${VENV_DIR}" ]]; then
    log "Creating Python venv at ${VENV_DIR}…"
    python3 -m venv --system-site-packages "${VENV_DIR}"
fi
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip wheel setuptools
ok "venv active: $(which python)"

# =============================================================================
# 5. Install the two local Python packages from the repo (editable)
# =============================================================================
if [[ -d "${REPO_DIR}/gym" ]]; then
    log "Installing local gym (editable)…"
    pip install -e "${REPO_DIR}/gym"
else
    warn "${REPO_DIR}/gym not found — skipping. Check repo layout."
fi

if [[ -d "${REPO_DIR}/f1tenth_gym" ]]; then
    log "Installing local f1tenth_gym (editable)…"
    pip install -e "${REPO_DIR}/f1tenth_gym"
else
    warn "${REPO_DIR}/f1tenth_gym not found — skipping. Check repo layout."
fi

pip install "numpy==1.26.0"
ok "Python deps installed."

# =============================================================================
# 6. rosdep install + colcon build
# =============================================================================
deactivate

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${WS_DIR}"
log "Running rosdep install for workspace deps…"
rosdep install --from-paths src --ignore-src -r -y || \
    warn "rosdep reported unresolved deps — continuing; inspect output above."

log "Building workspace with colcon (--symlink-install)…"
colcon build --symlink-install
ok "Workspace built."

# =============================================================================
# 7. Shell integration
# =============================================================================
BRC="$HOME/.bashrc"
add_line() {
    local line="$1"
    grep -qxF "$line" "$BRC" || echo "$line" >> "$BRC"
}

log "Adding source lines to ~/.bashrc (idempotent)…"
add_line "# --- f1tenth_simulator setup ---"
add_line "source /opt/ros/${ROS_DISTRO}/setup.bash"
add_line "source ${WS_DIR}/install/setup.bash"
add_line "source ${VENV_DIR}/bin/activate"
ok "${HOME}/.bashrc updated."

# =============================================================================
# 8. Dual-boot clock fix
# =============================================================================
if timedatectl show | grep -q 'LocalRTC=no'; then
    log "Setting hardware clock to local time (dual-boot friendly)…"
    sudo timedatectl set-local-rtc 1 --adjust-system-clock
    ok "RTC set to local time."
else
    ok "RTC already local — skipping."
fi

# =============================================================================
# Done
# =============================================================================
cat <<EOF

${GRN}============================================================${RST}
${GRN} Setup complete.${RST}
${GRN}============================================================${RST}

Next steps (open a NEW terminal so ~/.bashrc re-sources):

  # Launch the simulator
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py

  # In another terminal, keyboard control:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Repo:       ${REPO_DIR}
Workspace:  ${WS_DIR}
Venv:       ${VENV_DIR}
ROS distro: ${ROS_DISTRO}

If you have an NVIDIA GPU and haven't installed drivers yet:
  ubuntu-drivers devices
  sudo ubuntu-drivers autoinstall
  sudo reboot

EOF

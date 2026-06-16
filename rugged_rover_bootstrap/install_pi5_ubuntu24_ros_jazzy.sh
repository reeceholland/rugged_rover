#!/usr/bin/env bash
set -euo pipefail

# Bootstrap a Raspberry Pi 5 for Rugged Rover.
# Target: Ubuntu Server 24.04 + ROS 2 Jazzy.
#
# This script is designed to run from inside the rugged_rover repository:
#   cd ~/rugged_rover_ws/src/rugged_rover
#   bash rugged_rover_bootstrap/install_pi5_ubuntu24_ros_jazzy.sh
#
# It also repairs a common fresh-image apt issue where noble-updates is missing,
# which can leave packages such as bzip2/libbz2 with incompatible versions.

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/rugged_rover_ws}"
REPO_DIR="${REPO_DIR:-$WORKSPACE_DIR/src/rugged_rover}"
REPO_URL="${REPO_URL:-git@github.com:reeceholland/rugged_rover.git}"
REPO_BRANCH="${REPO_BRANCH:-main}"
GIT_USER_NAME="${GIT_USER_NAME:-Reece Holland}"
GIT_USER_EMAIL="${GIT_USER_EMAIL:-reece.j.holland@gmail.com}"

log() {
  echo
  echo "==> $*"
}

require_ubuntu_24_04() {
  if [[ ! -r /etc/os-release ]]; then
    echo "Cannot read /etc/os-release; refusing to continue." >&2
    exit 1
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
    echo "This script targets Ubuntu 24.04. Detected: ${PRETTY_NAME:-unknown}" >&2
    exit 1
  fi
}

ensure_ubuntu_updates_repo() {
  log "Checking Ubuntu apt suites"

  local sources_file="/etc/apt/sources.list.d/ubuntu.sources"

  if grep -Rqs "Suites:.*noble-updates" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    echo "noble-updates already configured."
  elif [[ -f "$sources_file" ]]; then
    sudo cp "$sources_file" "${sources_file}.bak.$(date +%Y%m%d%H%M%S)"
    sudo python3 - <<'PY'
from pathlib import Path
path = Path('/etc/apt/sources.list.d/ubuntu.sources')
text = path.read_text()
text = text.replace('Suites: noble\n', 'Suites: noble noble-updates\n', 1)
path.write_text(text)
PY
  else
    sudo tee /etc/apt/sources.list.d/99-ubuntu-noble-updates.sources >/dev/null <<'APT'
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble-updates
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
APT
  fi

  sudo apt update
}

repair_base_apt_state() {
  log "Repairing base apt state"

  sudo dpkg --configure -a
  sudo apt --fix-broken install -y

  # The Pi image previously had libbz2 from noble-updates while noble-updates was
  # missing from apt sources. Install bzip2 explicitly after repairing sources so
  # dpkg-dev and build-essential can resolve cleanly.
  sudo apt install -y bzip2 libbz2-1.0
}

configure_git() {
  log "Configuring git identity"
  git config --global user.name "$GIT_USER_NAME"
  git config --global user.email "$GIT_USER_EMAIL"
}

install_base_tools() {
  log "Installing base utilities"
  sudo apt install -y \
    apt-transport-https \
    bash-completion \
    build-essential \
    ca-certificates \
    chrony \
    cmake \
    curl \
    git \
    gnupg \
    htop \
    iproute2 \
    iw \
    libasio-dev \
    lsb-release \
    nano \
    ninja-build \
    openssh-server \
    python3-argcomplete \
    python3-pip \
    python3-venv \
    software-properties-common \
    tmux \
    udev \
    usbutils

  sudo systemctl enable --now ssh
  sudo systemctl enable --now chrony || true
}

configure_ros_apt_repo() {
  log "Configuring ROS 2 apt repository"
  sudo add-apt-repository universe -y

  sudo install -d -m 0755 /etc/apt/keyrings
  if [[ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
      sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" |
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  sudo apt update
}

install_ros_packages() {
  log "Installing ROS 2 ${ROS_DISTRO} packages"
  sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    "ros-${ROS_DISTRO}-ament-copyright" \
    "ros-${ROS_DISTRO}-ament-uncrustify" \
    "ros-${ROS_DISTRO}-controller-manager" \
    "ros-${ROS_DISTRO}-depthimage-to-laserscan" \
    "ros-${ROS_DISTRO}-diagnostic-updater" \
    "ros-${ROS_DISTRO}-diff-drive-controller" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-joy" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-realsense2-camera" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-teleop-twist-joy" \
    "ros-${ROS_DISTRO}-tf2-tools" \
    "ros-${ROS_DISTRO}-topic-tools" \
    "ros-${ROS_DISTRO}-xacro"
}

configure_rosdep() {
  log "Configuring rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

configure_groups_and_udev() {
  log "Configuring serial/GPIO permissions and device aliases"
  sudo usermod -aG dialout,input,gpio,i2c,spi "$USER" || true

  sudo tee /etc/udev/rules.d/99-rugged-rover.rules >/dev/null <<'UDEV'
# Raspberry Pi primary UART used by the Teensy micro-ROS transport.
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660", SYMLINK+="teensy_uart"

# RPLIDAR S2 USB serial adapter. Most units enumerate as Silicon Labs CP210x.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="rplidar"

# SparkFun 9DoF Razor IMU.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1b4f", ATTRS{idProduct}=="9d0f", GROUP="dialout", MODE="0660", SYMLINK+="razor_imu"
UDEV

  sudo udevadm control --reload-rules
  sudo udevadm trigger || true
}

ensure_repo_present() {
  log "Ensuring rugged_rover repository is present"
  mkdir -p "$WORKSPACE_DIR/src"

  if [[ -d "$REPO_DIR/.git" ]]; then
    git -C "$REPO_DIR" fetch --all --prune || true
    git -C "$REPO_DIR" checkout "$REPO_BRANCH" || true
    git -C "$REPO_DIR" pull --ff-only || true
  else
    git clone --branch "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR"
  fi

  git -C "$REPO_DIR" submodule update --init --recursive
}

install_workspace_dependencies() {
  log "Installing workspace dependencies with rosdep"
  cd "$WORKSPACE_DIR"
  rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
}

build_workspace() {
  log "Building workspace"
  cd "$WORKSPACE_DIR"
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  colcon build --symlink-install
}

configure_shell() {
  log "Configuring shell environment"
  local bashrc="$HOME/.bashrc"

  grep -qxF "source /opt/ros/${ROS_DISTRO}/setup.bash" "$bashrc" ||
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >>"$bashrc"

  grep -qxF "if [ -f \"$WORKSPACE_DIR/install/setup.bash\" ]; then source \"$WORKSPACE_DIR/install/setup.bash\"; fi" "$bashrc" ||
    echo "if [ -f \"$WORKSPACE_DIR/install/setup.bash\" ]; then source \"$WORKSPACE_DIR/install/setup.bash\"; fi" >>"$bashrc"

  grep -qxF "export ROS_DOMAIN_ID=0" "$bashrc" ||
    echo "export ROS_DOMAIN_ID=0" >>"$bashrc"

  grep -qxF "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" "$bashrc" ||
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >>"$bashrc"
}

print_next_steps() {
  cat <<EOF

Setup complete.

Recommended next steps:
  1. Reboot so group/udev changes are definitely active:
       sudo reboot

  2. After reconnecting, verify devices:
       ls -l /dev/teensy_uart /dev/rplidar /dev/razor_imu 2>/dev/null || true

  3. Launch the rover baseline:
       ros2 launch rugged_rover_bringup bringup.launch.py use_ekf:=true use_slam:=true use_rplidar:=true

Notes:
  - If bzip2/libbz2 failed before, this script enables noble-updates before installing build tools.
  - If the RPLIDAR symlink does not appear, check lsusb for its VID:PID.
  - Keep RViz on your laptop when possible; the Pi should run the rover stack.
EOF
}

main() {
  require_ubuntu_24_04
  ensure_ubuntu_updates_repo
  repair_base_apt_state
  configure_git
  install_base_tools
  configure_ros_apt_repo
  install_ros_packages
  configure_rosdep
  configure_groups_and_udev
  ensure_repo_present
  install_workspace_dependencies
  build_workspace
  configure_shell
  print_next_steps
}

main "$@"

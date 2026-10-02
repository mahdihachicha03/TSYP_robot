#!/usr/bin/env bash
# Install everything the robot needs on the Raspberry Pi (Ubuntu 24.04 + ROS 2 Jazzy).
# Safe to re-run. Usage: ./scripts/install_deps.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_DIR/ros2_ws"

echo "==> ROS 2 packages (apt)"
sudo apt-get update
sudo apt-get install -y \
  python3-venv python3-serial \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools

echo "==> Serial port access (takes effect after logging out and back in)"
sudo usermod -aG dialout "$USER"

echo "==> udev rules"
sudo cp "$REPO_DIR/udev/99-robot.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "==> YDLidar SDK (installed to ~/.local)"
if [ ! -f "$HOME/.local/lib/libydlidar_sdk.a" ]; then
  [ -d "$HOME/YDLidar-SDK" ] || git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$HOME/YDLidar-SDK"
  cmake -S "$HOME/YDLidar-SDK" -B "$HOME/YDLidar-SDK/build" -DCMAKE_INSTALL_PREFIX="$HOME/.local"
  cmake --build "$HOME/YDLidar-SDK/build" -j2
  cmake --install "$HOME/YDLidar-SDK/build"
fi

echo "==> ydlidar_ros2_driver (humble branch: master uses a pre-Humble rclcpp API)"
[ -d "$WS/src/ydlidar_ros2_driver" ] || \
  git clone --depth 1 -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$WS/src/ydlidar_ros2_driver"

echo "==> PlatformIO (venv in ~/.platformio-venv)"
if [ ! -x "$HOME/.platformio-venv/bin/pio" ]; then
  python3 -m venv "$HOME/.platformio-venv"
  "$HOME/.platformio-venv/bin/pip" install platformio
fi

echo "==> Build the workspace"
source /opt/ros/jazzy/setup.bash
cd "$WS"
CMAKE_PREFIX_PATH="$HOME/.local:${CMAKE_PREFIX_PATH:-}" LIBRARY_PATH="$HOME/.local/lib" \
  colcon build --symlink-install --parallel-workers 2

echo
echo "Done. Add to ~/.bashrc:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS/install/setup.bash"
echo "  alias pio=\$HOME/.platformio-venv/bin/pio"

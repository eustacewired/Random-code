#!/bin/bash
# One-time machine setup for the Auracle workspace (sim + real-robot-laptop roles).
#
# Installs ROS 2 Jazzy, Gazebo (ros_gz), Nav2/SLAM/control stack, Zenoh, builds this
# workspace, and wires up sourcing in ~/.bashrc.
#
# Run it yourself, NOT through an agent, so your sudo password stays in your terminal:
#     bash pc/setup_machine.sh
#
# Safe to re-run: every step checks before it acts.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # repo root = colcon workspace root
ROS_DISTRO=jazzy
# This host reports Ubuntu 26.04 ("resolute"), newer than Jazzy's official target
# (24.04 "noble"). packages.ros.org has no "resolute" build, so we pin the ROS apt
# repo to the noble codename, which is the standard workaround on newer-than-supported
# Ubuntu releases. If apt ever fails to resolve a ros-jazzy-* package below, that's the
# first thing to suspect.
ROS_APT_CODENAME=noble

log()  { echo -e "\n== $* =="; }
ok()   { echo "  [ok] $*"; }

if [ "$(id -u)" -eq 0 ]; then
    echo "Don't run this as root; run it as your normal user, it calls sudo itself." >&2
    exit 1
fi

log "1. Base prerequisites"
sudo apt-get update -qq
sudo apt-get install -y curl ca-certificates gnupg lsb-release git build-essential
ok "prerequisites installed"

log "2. ROS 2 Jazzy apt repo (pinned to $ROS_APT_CODENAME)"
if [ ! -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    sudo mkdir -p /usr/share/keyrings
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $ROS_APT_CODENAME main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update -qq
else
    ok "ROS 2 $ROS_DISTRO already present, skipping repo add"
fi

log "3. Zenoh apt repo (for the Pi-link / twist-mux networking role)"
if [ ! -f /etc/apt/sources.list.d/zenoh.list ]; then
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
        | sudo tee /etc/apt/sources.list.d/zenoh.list >/dev/null
    sudo apt-get update -qq
else
    ok "zenoh apt repo already present"
fi

log "4. Installing packages (this is the big download, ~15-25 min)"
sudo apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-robot-localization \
    ros-jazzy-twist-mux ros-jazzy-twist-stamper \
    ros-jazzy-rplidar-ros \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy \
    ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joy \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
    libserial-dev \
    python3-colcon-common-extensions python3-rosdep \
    zenoh chrony
ok "all packages installed"

log "5. rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

log "6. Serial access (dialout group) -- harmless even with no device attached yet"
if ! groups "$USER" | grep -q '\bdialout\b'; then
    sudo usermod -aG dialout "$USER"
    echo "  Added $USER to dialout. Needs a fresh login/WSL restart to take effect."
else
    ok "$USER already in dialout"
fi

log "7. Build the workspace ($WS)"
source /opt/ros/$ROS_DISTRO/setup.bash
chmod +x "$WS"/src/auracle_bringup/scripts/*.py
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || echo "  (rosdep install reported issues above -- check before relying on it, build will still be attempted)"
rm -rf build install log
colcon build --symlink-install
ok "workspace built"

log "8. Shell sourcing (~/.bashrc)"
BRC="$HOME/.bashrc"
add_line() { grep -qxF "$1" "$BRC" 2>/dev/null || echo "$1" >> "$BRC"; }
add_line "source /opt/ros/$ROS_DISTRO/setup.bash"
add_line "source $WS/install/setup.bash"
ok "bashrc updated (source it or open a new terminal)"

log "Done"
cat <<EOF

Next steps, in a NEW terminal (so .bashrc reloads):

  Simulation (self-contained, no Pi needed):
    ros2 launch auracle_bringup launch_sim.launch.py use_joy_node:=false
    ros2 launch auracle_bringup slam_mapping.launch.py use_sim_time:=true
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy

  Real robot (this machine as the laptop, once a Pi is on the network):
    source $WS/pc/auracle_up.sh            # finds the Pi, sets Zenoh env, syncs clocks
    ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle

Not done by this script (do these only if needed):
  - 99-auracle.rules / udev symlinks: only relevant if the lidar or Arduino is plugged
    directly into THIS machine rather than the Pi. See README.md "Adding udev rules".
  - PlatformIO / arduino-cli firmware flashing: separate from launch/teleop/map, see
    README.md "Flashing the Nano from WSL".
  - neo_localization2 (localization_neo.launch.py): experimental, not in package.xml,
    not installed here.
EOF

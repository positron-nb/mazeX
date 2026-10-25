#!/usr/bin/env bash
# Source ROS 2 and the MazeMind workspace:  source scripts/env.sh
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${MAZEMIND_WS:-$HOME/mazemind_ws}/install/setup.bash"
export MAZEMIND_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# WSLg: after a WSL restart the X socket link can be missing, so GUI apps (Gazebo, RViz)
# cannot open windows. Recreate it when the real socket exists.
if [ -S /mnt/wslg/.X11-unix/X0 ] && [ ! -S /tmp/.X11-unix/X0 ] && [ -w /tmp/.X11-unix ]; then
  ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0
fi

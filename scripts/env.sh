#!/usr/bin/env bash
# Source ROS 2 and the MazeMind workspace:  source scripts/env.sh
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${MAZEMIND_WS:-$HOME/mazemind_ws}/install/setup.bash"
export MAZEMIND_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

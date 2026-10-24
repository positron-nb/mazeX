#!/usr/bin/env bash
# Build MazeMind in a colcon workspace on the Linux filesystem.
#
# The repository may live on a Windows drive (/mnt/c/...); building there is slow
# and paths with spaces confuse some tools, so sources are rsync'd into
# $MAZEMIND_WS/src (default ~/mazemind_ws) and built there.
#
#   bash scripts/build.sh            # sync + build all packages
#   bash scripts/build.sh --test     # ... and run colcon test
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${MAZEMIND_WS:-$HOME/mazemind_ws}"
ROS_DISTRO_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

mkdir -p "$WS/src"
rsync -a --delete --exclude '__pycache__' --exclude '.pytest_cache' "$REPO/src/" "$WS/src/"

# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"
cd "$WS"
colcon build --symlink-install --event-handlers console_direct- 2>&1 | tail -n 25

if [[ "${1:-}" == "--test" ]]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  colcon test --event-handlers console_direct- --pytest-args -q -p no:cacheprovider || true
  colcon test-result --verbose
fi
echo "Built into $WS. Run: source $WS/install/setup.bash"

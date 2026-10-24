#!/usr/bin/env bash
# Run the main strategy-comparison campaigns and analyse them.
#
#   bash scripts/run_campaigns.sh                # easy + medium, seeds 0-9
#   SEEDS="0 1 2" bash scripts/run_campaigns.sh  # fewer seeds
#
# Resumable: finished runs (results/<campaign>/runs/*.json) are skipped.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/scripts/env.sh"
SEEDS=${SEEDS:-"0 1 2 3 4 5 6 7 8 9"}
OUT="$REPO/results"

# shellcheck disable=SC2086
ros2 run mazemind_evaluation run_experiments --campaign easy_main --difficulty easy \
  --seeds $SEEDS --time-limit 420 --out "$OUT"
ros2 run mazemind_evaluation analyze_results "$OUT/easy_main"

# shellcheck disable=SC2086
ros2 run mazemind_evaluation run_experiments --campaign medium_main --difficulty medium \
  --seeds $SEEDS --time-limit 600 --out "$OUT"
ros2 run mazemind_evaluation analyze_results "$OUT/medium_main"

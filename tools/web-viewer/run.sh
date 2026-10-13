#!/usr/bin/env bash
# Start the V2X E2E Simulator web viewer on http://localhost:8080
#
#   tools/web-viewer/run.sh [--port 8080]
#
# Environment (override before running if your setup differs):
#   ROS_SETUP   ROS 2 setup script              (default /opt/ros/humble/setup.bash)
#   MSGS_SETUP  v2x_e2e_msgs workspace setup     (default ../../../v2x_e2e_msgs/install/setup.bash)
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MSGS_SETUP="${MSGS_SETUP:-$HERE/../../../../v2x_e2e_msgs/install/setup.bash}"

# Same middleware settings as the simulator (see docs: Getting started).
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [ -f "$MSGS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$MSGS_SETUP"
else
  echo "warning: $MSGS_SETUP not found; DM / V2X topics cannot be decoded." >&2
  echo "         Build https://github.com/tlab-wide/v2x_e2e_msgs and set MSGS_SETUP." >&2
fi

# Python packages the viewer needs beyond ROS (kept out of the system Python).
VENV="$HERE/.venv"
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv --system-site-packages "$VENV"
  "$VENV/bin/pip" install -q -r "$HERE/requirements.txt"
fi

exec "$VENV/bin/python" "$HERE/server/main.py" "$@"

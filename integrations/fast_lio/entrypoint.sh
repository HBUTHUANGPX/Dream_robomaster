#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/catkin_ws/devel/setup.bash
exec /opt/fast_lio/bin/fast_lio_replay "$@"

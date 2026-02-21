#!/usr/bin/env bash
# despawn_robot.sh — remove a robot model from a running Gazebo simulation.
#
# Usage:
#   scripts/despawn_robot.sh [ROBOT_NAME] [WORLD_NAME]
#
# Defaults:
#   ROBOT_NAME = derpbot_0
#   WORLD_NAME = indoor_office
#
# Examples:
#   scripts/despawn_robot.sh
#   scripts/despawn_robot.sh derpbot_0
#   scripts/despawn_robot.sh derpbot_1 indoor_warehouse
#
# After despawning, press Ctrl-C in the terminal running spawn_robot.launch.py
# to stop the associated bridges and robot_state_publisher.

set -euo pipefail

ROBOT_NAME="${1:-derpbot_0}"
WORLD_NAME="${2:-indoor_office}"

echo "Removing '${ROBOT_NAME}' from world '${WORLD_NAME}' ..."

gz service \
  -s "/world/${WORLD_NAME}/remove" \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "name: \"${ROBOT_NAME}\" type: MODEL"

echo "Done.  Remember to Ctrl-C the spawn_robot.launch.py process for '${ROBOT_NAME}'."

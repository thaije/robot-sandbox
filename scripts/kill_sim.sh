#!/usr/bin/env bash
# Nuclear option: kill every process left over from a scenario run.
# Use after a failed/stuck run before starting a new one.
pkill -KILL -f "gz sim"        2>/dev/null; true
pkill -KILL -f "ros_gz_bridge" 2>/dev/null; true
pkill -KILL -f "scenario_runner" 2>/dev/null; true
pkill -KILL -f "map_publisher"  2>/dev/null; true
fuser -k 7400/tcp               2>/dev/null; true
echo "[kill_sim] done"

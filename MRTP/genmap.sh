#!/usr/bin/env bash
set -e

BIN_FILE="${1:-castle.bin}"

cd /MRTP/MRTP
source install/setup.bash

ros2 run generatemap tcp_demux_node --ros-args \
  -p output_path:="/MRTP/MRTP/clearpath_simulator/clearpath_gz/worlds/generated_orchard.sdf" &
TCP_PID=$!

cleanup() {
  kill $TCP_PID 2>/dev/null || true
}
trap cleanup EXIT

sleep 2

cd /MRTP/MRTP/generatemap
nc localhost 12346 < "$BIN_FILE"

cd /MRTP/MRTP/clearpath_simulator
colcon build
source install/setup.bash
ros2 launch generatemap bringup_simulation.launch.py
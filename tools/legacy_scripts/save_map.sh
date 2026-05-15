#!/usr/bin/env bash
# Save the current RTAB-Map occupancy grid to PGM+YAML for Nav2 map_server.
# Run this while the SLAM container is running.
# Output: data/slam/map.pgm + map.yaml (overwritten each call)
set -euo pipefail

CONTAINER=$(docker ps --filter ancestor=wildbot_slam_fusion:latest --format "{{.Names}}" | head -1)
if [[ -z "$CONTAINER" ]]; then
    echo "ERROR: slam_fusion container is not running." >&2
    exit 1
fi

MAP_NAME="${1:-map}"   # optional first arg overrides filename, e.g. ./save_map.sh kitchen

echo "Saving map as '$MAP_NAME' from container $CONTAINER ..."
docker exec "$CONTAINER" bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 run nav2_map_server map_saver_cli \
       --ros-args -p save_map_timeout:=5.0 \
       -- -f /root/.ros/${MAP_NAME}"

echo "Saved to data/slam/${MAP_NAME}.pgm + ${MAP_NAME}.yaml"

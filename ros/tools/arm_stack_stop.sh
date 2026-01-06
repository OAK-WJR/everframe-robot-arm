#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${1:-ros_noetic_container}"

docker exec "${CONTAINER}" bash -lc '
set -euo pipefail

echo "[stop] killing ROS/MoveIt/bridge processes in container..."

# Collect PIDs precisely to avoid killing this shell.
# NOTE: we also kill rosmaster/rosout; otherwise masters from previous ports accumulate and can cause run_id issues.
PIDS=$(ps -eo pid=,cmd= | egrep "roscore -p|/opt/ros/noetic/bin/rosmaster --core -p|/opt/ros/noetic/lib/rosout/rosout|/opt/ros/noetic/bin/roslaunch|move_group|moveit_servo/servo_server|arm_control/arm_hw_node|arm_nt4_bridge/arm_nt4_bridge_node|servo_traj_to_jtc.py|controller_manager/spawner" | egrep -v "egrep" | awk "{print \$1}" | tr "\n" " ")

echo "[stop] PIDS=[${PIDS}]"
if [ -n "${PIDS}" ]; then
  kill -9 ${PIDS} >/dev/null 2>&1 || true
fi

sleep 1
echo "[stop] remaining (should be empty):"
ps -eo pid=,cmd= | egrep "roscore -p|/opt/ros/noetic/bin/rosmaster --core -p|/opt/ros/noetic/lib/rosout/rosout|/opt/ros/noetic/bin/roslaunch|move_group|moveit_servo/servo_server|arm_control/arm_hw_node|arm_nt4_bridge/arm_nt4_bridge_node|servo_traj_to_jtc.py|controller_manager/spawner" | egrep -v "egrep" || true
'



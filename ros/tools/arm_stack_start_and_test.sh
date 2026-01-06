#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./tools/arm_stack_start_and_test.sh [PORT] [VEL] [DUR_S] [SERVER_IP] [CONTAINER] [NT_RATE] [CMD_SOURCE] [JTC_STATE_PUB_RATE]
#
# Example:
#   ./tools/arm_stack_start_and_test.sh 11380 0.2 2 10.1.14.2 ros_noetic_container

PORT="${1:-11380}"
VEL="${2:-0.2}"
DUR_S="${3:-2}"
SERVER_IP="${4:-10.1.14.2}"
CONTAINER="${5:-ros_noetic_container}"
NT_RATE="${6:-50}"
CMD_SOURCE="${7:-}"
JTC_STATE_PUB_RATE="${8:-}"

OUT_DIR="/tmp/phys_${PORT}"

# Monitor window tuning (seconds)
# - Keep pre short to avoid lots of "standing still" data
# - Keep post long to capture stop/settling/overshoot
PRE_S="${PRE_S:-0.10}"
POST_S="${POST_S:-2.50}"

# Sync controllers.yaml from host workspace into container workspace (common source of "yaml改了但没生效").
# This script runs on the host (docker available), so we can docker cp before launching.
HOST_CONTROLLERS_YAML="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/arm_control/config/controllers.yaml"
if [ -f "${HOST_CONTROLLERS_YAML}" ]; then
  docker cp "${HOST_CONTROLLERS_YAML}" "${CONTAINER}:/root/catkin_ws/src/arm_control/config/controllers.yaml" >/dev/null 2>&1 || true
fi

# Sync frequently edited launch/C++ files too (avoids "我改了但容器里还是旧的").
HOST_JTC_NT4_LAUNCH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/arm_control/launch/jtc_nt4.launch"
if [ -f "${HOST_JTC_NT4_LAUNCH}" ]; then
  docker cp "${HOST_JTC_NT4_LAUNCH}" "${CONTAINER}:/root/catkin_ws/src/arm_control/launch/jtc_nt4.launch" >/dev/null 2>&1 || true
fi

HOST_NT4_BRIDGE_CPP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/arm_nt4_bridge/src/arm_nt4_bridge_node.cpp"
if [ -f "${HOST_NT4_BRIDGE_CPP}" ]; then
  docker cp "${HOST_NT4_BRIDGE_CPP}" "${CONTAINER}:/root/catkin_ws/src/arm_nt4_bridge/src/arm_nt4_bridge_node.cpp" >/dev/null 2>&1 || true
fi

# Sync homing helper script (position target, bypass jog smoothing for this one command)
HOST_JTC_GOTO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tools/jtc_goto_joint_pos.py"
if [ -f "${HOST_JTC_GOTO}" ]; then
  docker cp "${HOST_JTC_GOTO}" "${CONTAINER}:/root/catkin_ws/tools/jtc_goto_joint_pos.py" >/dev/null 2>&1 || true
fi

# Always stop old stacks first
./tools/arm_stack_stop.sh "${CONTAINER}" >/dev/null 2>&1 || true

# ========== HOME TO 0° USING NT4 (non-ROS mode) ==========
# This runs on the HOST (Jetson) before any ROS stack starts.
# Uses pynetworktables to directly write TargetDeg=0 to RoboRIO.
# Much more reliable than ROS-based homing which can be overwritten by Servo.
NT4_GOTO_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nt/nt4_goto.py"
if [ -f "${NT4_GOTO_SCRIPT}" ]; then
  echo "[home] NT4 non-ROS mode: go AngleDeg -> 0.0°"
  python3 "${NT4_GOTO_SCRIPT}" --server "${SERVER_IP}" --joint J2 --target 0.0 --maxvolts 2.0 --timeout 20 --eps 1.0 2>&1 | head -n 20 || true
  sleep 0.5
else
  echo "[home] WARN: ${NT4_GOTO_SCRIPT} not found; skipping NT4 home"
fi

docker exec "${CONTAINER}" bash -lc "
set -eo pipefail

# NOTE: avoid 'set -u' before sourcing ROS setup files; some distros reference ROS_MASTER_URI if unset.
export ROS_MASTER_URI=http://127.0.0.1:${PORT}
export ROS_IP=127.0.0.1

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Best-effort rebuild of arm_nt4_bridge (fast) so changes to the C++ bridge take effect.
cd /root/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo --pkg arm_nt4_bridge >/tmp/build_arm_nt4_bridge.log 2>&1 || (tail -n 200 /tmp/build_arm_nt4_bridge.log && exit 2)

OUT_DIR='${OUT_DIR}'
SERVER_IP='${SERVER_IP}'
VEL='${VEL}'
DUR_S='${DUR_S}'
PORT='${PORT}'
PRE_S='${PRE_S}'
POST_S='${POST_S}'

mkdir -p \"\${OUT_DIR}\"

echo \"[start] roscore \${PORT}\"
nohup roscore -p \${PORT} >\"\${OUT_DIR}/roscore.log\" 2>&1 &
echo \$! >\"\${OUT_DIR}/roscore.pid\" || true
sleep 2

# NOTE: Homing is now done BEFORE docker exec, using NT4 non-ROS mode (nt4_goto.py on host).
# This is much more reliable because it doesn't get overwritten by servo_traj_to_jtc.

echo \"[start] stack moveit_servo_to_jtc_to_nt4\"
EXTRA_ARGS=\"\"
if [ -n \"\${CMD_SOURCE}\" ]; then
  EXTRA_ARGS=\"\${EXTRA_ARGS} cmd_source:=\${CMD_SOURCE}\"
fi
if [ -n \"\${JTC_STATE_PUB_RATE}\" ]; then
  EXTRA_ARGS=\"\${EXTRA_ARGS} jtc_state_publish_rate:=\${JTC_STATE_PUB_RATE}\"
fi
nohup roslaunch arm_nt_bridge moveit_servo_to_jtc_to_nt4.launch server_ip:=\${SERVER_IP} nt_rate:=\${NT_RATE} control_rate:=100 ros_timeout_s:=0.25 \${EXTRA_ARGS} >\"\${OUT_DIR}/stack.log\" 2>&1 &
echo \$! >\"\${OUT_DIR}/stack.pid\" || true

READY=0
for i in \$(seq 1 80); do
  if rostopic list 2>/dev/null | grep -q \"/arm_controller/command\" && rostopic list 2>/dev/null | grep -q \"/arm_hw/nt_state\"; then
    READY=1; break
  fi
  sleep 1
done
if [ \"\${READY}\" -ne 1 ]; then
  echo \"[error] stack not ready\"
  tail -n 200 \"\${OUT_DIR}/stack.log\" || true
  exit 2
fi

echo \"[ok] stack ready -> NT4 watch + jog\"
# NT4 watch window: PRE_S before motion, POST_S after motion
WATCH_DUR_S=\$(awk -v d=\"\${DUR_S}\" -v pre=\"\${PRE_S}\" -v post=\"\${POST_S}\" 'BEGIN { v=pre+d+post; if (v < 2.5) v=2.5; print v }')
# IMPORTANT:
# - Even if motion stops early, we keep dumping until WATCH_DUR_S to capture settling.
# - To prevent rare hangs, wrap the dump with a hard timeout.
WATCH_TIMEOUT_S=\$(awk -v w=\"\${WATCH_DUR_S}\" 'BEGIN { print w + 2.0 }')
(timeout \"\${WATCH_TIMEOUT_S}\" /root/catkin_ws/devel/lib/arm_nt4_bridge/nt4_dump_arm \${SERVER_IP} J2 0.2 \${NT_RATE} \${WATCH_DUR_S} >\"\${OUT_DIR}/nt4_watch.txt\" 2>&1 || true) &
W=\$!

# Rate-check in parallel
RATECHECK_S=\$(awk -v d=\"\${DUR_S}\" 'BEGIN { v=d+1.0; if (v < 2.0) v=2.0; print v }')
RATECHECK_TIMEOUT_S=\$(awk -v r=\"\${RATECHECK_S}\" 'BEGIN { print r + 2.0 }')
(timeout \"\${RATECHECK_TIMEOUT_S}\" /root/catkin_ws/devel/lib/arm_nt4_bridge/nt4_rate_check \${SERVER_IP} J2 \${RATECHECK_S} >\"\${OUT_DIR}/rate_check.txt\" 2>&1 || true) &
RC=\$!

# Wait PRE_S to capture (short) pre-motion state
sleep \${PRE_S}

# Verify key controller params
{
  echo \"--- rosparam verify (arm_controller) ---\";
  rosparam get /arm_controller/stop_trajectory_duration 2>/dev/null || echo \"stop_trajectory_duration: (not set)\";
  rosparam get /arm_controller/allow_partial_joints_goal 2>/dev/null || echo \"allow_partial_joints_goal: (not set)\";
} >\"\${OUT_DIR}/rosparam_verify.txt\" 2>&1 || true

# Send jog command
JOG_TIMEOUT_S=\$(awk -v d=\"\${DUR_S}\" 'BEGIN { print d + 3.0 }')
timeout \"\${JOG_TIMEOUT_S}\" roslaunch arm_nt_bridge joint_jog_pub.launch joint:=joint2 vel:=\${VEL} run_for_s:=\${DUR_S} >\"\${OUT_DIR}/jog.log\" 2>&1 || true

# Wait for NT4 dump to finish (captures post-motion state)
wait \${W} || true
wait \${RC} || true

echo \"=== NT4 key lines (tail) ===\"
egrep \"Arm/Enabled=|Arm/J2/Enabled=|RosValid=|InhibitReason=|ControlMode=|RosPosRad=|RosVelRadPerS=|RosSeq=|AngleDeg=|OutVoltsCmd=\" \"\${OUT_DIR}/nt4_watch.txt\" | tail -n 120 || true
echo
echo \"=== stale warnings (tail) ===\"
grep -n \"Stale command\" \"\${OUT_DIR}/stack.log\" | tail -n 20 || true
echo

echo \"=== rosparam verify (tail) ===\"
tail -n 40 \"\${OUT_DIR}/rosparam_verify.txt\" || true
echo

echo \"=== nt4_rate_check (tail) ===\"
tail -n 80 \"\${OUT_DIR}/rate_check.txt\" || true
echo
echo \"OUT_DIR=\${OUT_DIR}\"
"

# Auto-save data to permanent storage (arm_data folder)
ARM_DATA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/arm_data"
mkdir -p "${ARM_DATA_DIR}"

# Create timestamped filename
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DATA_FILE="${ARM_DATA_DIR}/nt4_watch_${VEL}_${TIMESTAMP}.txt"

# Copy from container to permanent storage
docker cp "${CONTAINER}:/tmp/phys_${PORT}/nt4_watch.txt" "${DATA_FILE}" 2>/dev/null && \
  echo "Data saved: ${DATA_FILE}" || true

# Stop processes after each test to avoid lingering stacks
./tools/arm_stack_stop.sh "${CONTAINER}" >/dev/null 2>&1 || true


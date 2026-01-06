#!/usr/bin/env bash
set -eo pipefail

# Diagnose "卡顿" by aligning a single time window:
# - Servo status
# - Servo output hz
# - /joint_states hz
# - NT4 RosSeq + AngleDeg + RosPosRad
#
# Run inside ROS1 Noetic environment (e.g. container).
#
# Usage:
#   servo_chain_diag.sh [duration_s] [joint] [server_ip] [ros_master_port]
#
# Example:
#   servo_chain_diag.sh 15 joint2 10.1.14.2 11312

DUR="${1:-15}"
ROS_JOINT="${2:-joint2}"
SERVER_IP="${3:-10.1.14.2}"
MASTER_PORT="${4:-11312}"

CATKIN_WS="${CATKIN_WS:-/root/catkin_ws}"
LOG_DIR="${LOG_DIR:-/tmp/servo_chain_diag}"
mkdir -p "${LOG_DIR}"

export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_IP="127.0.0.1"
export ROS_LOG_DIR="${LOG_DIR}/roslog_${MASTER_PORT}"
mkdir -p "${ROS_LOG_DIR}"

export LD_LIBRARY_PATH="${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/ntcore/linux/arm64/shared:${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/wpinet/linux/arm64/shared:${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/wpiutil/linux/arm64/shared:${LD_LIBRARY_PATH:-}"

echo "== servo_chain_diag =="
echo "duration_s=${DUR} ros_joint=${ROS_JOINT} server_ip=${SERVER_IP} master_port=${MASTER_PORT}"
echo "CATKIN_WS=${CATKIN_WS}"
echo "LOG_DIR=${LOG_DIR}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "----"

cleanup() {
  pkill -f "roslaunch arm_nt4_bridge moveit_servo_to_nt4.launch" >/dev/null 2>&1 || true
  pkill -f "roscore -p ${MASTER_PORT}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# Start isolated master
(roscore -p "${MASTER_PORT}" >"${LOG_DIR}/roscore_${MASTER_PORT}.log" 2>&1) &
sleep 2

# Start stack
(roslaunch arm_nt4_bridge moveit_servo_to_nt4.launch loop_hz:=70 publish_joint_states:=true >"${LOG_DIR}/stack.log" 2>&1) &
sleep 12

echo "-- topic sanity --"
timeout 3 rostopic list | egrep "servo_server|joint_states|joint_trajectory_cmd|delta_joint_cmds|status" || true
echo "----"

# Record status message type for decoding (status=0 means different things depending on msg type)
{
  echo "status_topic_type=$(rostopic type /servo_server/status 2>/dev/null || true)"
  T="$(rostopic type /servo_server/status 2>/dev/null || true)"
  if [[ -n "${T}" ]]; then
    echo "---- rosmsg show ${T} ----"
    rosmsg show "${T}" 2>/dev/null || true
  fi
} > "${LOG_DIR}/servo_status_type.txt" 2>&1 || true

# Capture status stream (CSV)
timeout "${DUR}" rostopic echo -p /servo_server/status >"${LOG_DIR}/servo_status.csv" 2>/dev/null || true

# Hz logs (run in background)
(timeout "$((DUR+2))" rostopic hz /servo_server/joint_trajectory_cmd || true) >"${LOG_DIR}/hz_joint_traj.log" 2>&1 &
HZ1=$!
(timeout "$((DUR+2))" rostopic hz /joint_states || true) >"${LOG_DIR}/hz_joint_states.log" 2>&1 &
HZ2=$!

# NT4 monitor (watch mode) + seq stats
( "${CATKIN_WS}/devel/lib/arm_nt4_bridge/nt4_dump_arm" "${SERVER_IP}" J2 0.5 10 "${DUR}" ) >"${LOG_DIR}/nt4_dump_J2.log" 2>&1 &
NTD=$!
( "${CATKIN_WS}/devel/lib/arm_nt4_bridge/nt4_seq_monitor" "${SERVER_IP}" J2 RosSeq "${DUR}" ) >"${LOG_DIR}/nt4_seq_J2.log" 2>&1 &
NTS=$!

echo "-- start joint_jog_pub --"
roslaunch arm_nt_bridge joint_jog_pub.launch joint:="${ROS_JOINT}" vel:=0.2 run_for_s:="${DUR}" >"${LOG_DIR}/joint_jog_pub.log" 2>&1 || true

wait "${HZ1}" || true
wait "${HZ2}" || true
wait "${NTD}" || true
wait "${NTS}" || true

echo "----"
echo "== SUMMARY =="
echo "-- nt4_seq_monitor --"
tail -n 5 "${LOG_DIR}/nt4_seq_J2.log" || true
echo "-- hz /servo_server/joint_trajectory_cmd --"
tail -n 15 "${LOG_DIR}/hz_joint_traj.log" || true
echo "-- hz /joint_states --"
tail -n 15 "${LOG_DIR}/hz_joint_states.log" || true
echo "-- tail /servo_server/status (csv) --"
tail -n 5 "${LOG_DIR}/servo_status.csv" || true
echo "-- logs saved under: ${LOG_DIR} --"
echo "-- status msg type saved: ${LOG_DIR}/servo_status_type.txt --"



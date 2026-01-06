#!/usr/bin/env bash
set -euo pipefail

# One-shot + watch monitor for NT4 Arm table.
#
# Usage:
#   nt4_monitor.sh [server_ip] [joint] [duration_s]
#
# Examples:
#   nt4_monitor.sh 10.1.14.2 J2 5
#   CATKIN_WS=/home/jetson/catkin_ws nt4_monitor.sh 10.1.14.2 J2 20

SERVER_IP="${1:-10.1.14.2}"
JOINT="${2:-J2}"
DURATION_S="${3:-5}"

CATKIN_WS="${CATKIN_WS:-/home/jetson/catkin_ws}"
LIBS="${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/ntcore/linux/arm64/shared:${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/wpinet/linux/arm64/shared:${CATKIN_WS}/build/arm_nt4_bridge/wpilib_ext/wpiutil/linux/arm64/shared"

BIN_DIR="${CATKIN_WS}/devel/lib/arm_nt4_bridge"
DUMP="${BIN_DIR}/nt4_dump_arm"
SEQ="${BIN_DIR}/nt4_seq_monitor"

if [[ ! -x "${DUMP}" || ! -x "${SEQ}" ]]; then
  echo "ERROR: nt4 binaries not found/executable under ${BIN_DIR}"
  exit 2
fi

export LD_LIBRARY_PATH="${LIBS}:${LD_LIBRARY_PATH:-}"

echo "== nt4_monitor =="
echo "server_ip=${SERVER_IP} joint=${JOINT} duration_s=${DURATION_S}"
echo "CATKIN_WS=${CATKIN_WS}"
echo "LD_LIBRARY_PATH=${LIBS}:..."
echo "----"

# Watch key fields a few times for quick sanity.
"${DUMP}" "${SERVER_IP}" "${JOINT}" 0.5 5 3 || true

echo "----"
echo "== seq_monitor (RosSeq) =="
"${SEQ}" "${SERVER_IP}" "${JOINT}" RosSeq "${DURATION_S}" || true






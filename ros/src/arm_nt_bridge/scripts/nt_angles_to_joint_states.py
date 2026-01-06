#!/usr/bin/env python3
import math
import time
from typing import Dict, Optional

import rospy
from sensor_msgs.msg import JointState
from networktables import NetworkTables


def _unwrap_deg(prev: Optional[float], cur: float) -> float:
    """
    简单解包：把 cur 调整到离 prev 最近的等价角（cur + 360*k）。
    仅在你 RoboRIO 只给 0..360/(-180..180) 时用于连续角度。
    """
    if prev is None:
        return cur
    best = cur
    best_err = abs(cur - prev)
    for k in (-2, -1, 0, 1, 2):
        cand = cur + 360.0 * k
        err = abs(cand - prev)
        if err < best_err:
            best, best_err = cand, err
    return best


class NTAnglesToJointStates:
    """
    从 NetworkTables 读取 RoboRIO 的关节角（deg），发布成 ROS JointState（rad）。

    术语（与 RoboRIO 侧约定一致）：
    - AngleDeg：本关节自身的测量角度（单关节编码器），单位 deg（通常更适合作为 ROS 的“关节变量”发布）。

    本节点行为：
    - 从 NT 读取角度（deg），做标定/解包，转换为 rad 发布为 JointState.position
    - 默认发布到 /joint_states（注意：一个系统里应避免多个节点同时发布 /joint_states）
    - 只读取 AngleDeg（保持 JointState 语义明确，避免复合角污染）
    """

    def __init__(self):
        # NT
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.nt_table = rospy.get_param("~table", "Arm")
        # 固定读取的 key（保持语义明确）
        self.nt_key = "AngleDeg"

        # ROS
        self.pub_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.frame_id = rospy.get_param("~frame_id", "")
        # 额外关节（不从 NT 读取，直接按固定值补齐；用于 MoveIt/Servo 需要“完整关节状态”）
        # 例：
        # extra_joints:
        #   - {name: joint4, position: 0.0}
        self.extra_joints = rospy.get_param("~extra_joints", [])
        # 启动初期 NT 连接可能短暂为 False，避免一启动就刷屏报警
        self.warn_grace_period_s = float(rospy.get_param("~warn_grace_period_s", 2.0))
        self._start_t = time.time()

        # mapping: ROS joint name -> NT subtable name (J1/J2/J3/J4)
        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {"joint1": "J1", "joint2": "J2", "joint3": "J3", "joint4": "J4"},
        )

        # 角度标定（与下发 TargetDeg 一致）：out_deg = sign * in_deg + offset_deg
        # 这里做“逆变换”：in_deg = (out_deg - offset_deg) / sign
        self.joint_sign: Dict[str, float] = rospy.get_param("~joint_sign", {})
        self.joint_offset_deg: Dict[str, float] = rospy.get_param("~joint_offset_deg", {})

        # 如果 RoboRIO 给的是 0..360 或 -180..180，但你想连续角（例如 joint1 连续转），可开启解包
        self.unwrap = bool(rospy.get_param("~unwrap", False))
        # 始终发布完整 joint_states（Servo 需要 arm 组所有关节），缺数据时用上一次值/0 补齐
        self.publish_all_joints = bool(rospy.get_param("~publish_all_joints", True))
        # 某些关节你可能“禁用/不回传”（例如 joint4），但仍希望 JointState 里补齐它。
        # 这些关节缺数据时不报警。
        self.optional_joints = set(rospy.get_param("~optional_joints", []))
        self._last_deg: Dict[str, Optional[float]] = {j: 0.0 for j in self.joint_map.keys()}
        self._last_warn_t = 0.0

        NetworkTables.initialize(server=self.server_ip)
        self.nt = NetworkTables.getTable(str(self.nt_table))

        self.pub = rospy.Publisher(self.pub_topic, JointState, queue_size=1)

        rospy.loginfo(
            "nt_angles_to_joint_states ready. server_ip=%s table=%s key=%s -> %s @ %.1fHz",
            self.server_ip,
            self.nt_table,
            self.nt_key,
            self.pub_topic,
            self.rate_hz,
        )
        if self.pub_topic == "/joint_states":
            rospy.logwarn(
                "Publishing to /joint_states. Ensure there is ONLY ONE publisher of /joint_states in the system "
                "(otherwise downstream consumers will see unstable mixed messages)."
        )

    def _read_deg(self, nt_joint: str) -> Optional[float]:
        st = self.nt.getSubTable(nt_joint)
        # pynetworktables: getNumber(key, default)
        val = st.getNumber(self.nt_key, None)
            if val is None:
            return None
            try:
                return float(val)
            except Exception:
        return None

    def spin(self):
        rate = rospy.Rate(max(self.rate_hz, 1.0))
        while not rospy.is_shutdown():
            names = []
            pos = []
            missing = []

            for ros_joint, nt_joint in self.joint_map.items():
                deg_out = self._read_deg(nt_joint)
                if deg_out is None:
                    if ros_joint not in self.optional_joints:
                        missing.append(ros_joint)
                    deg_out = self._last_deg.get(ros_joint, 0.0)
                else:
                    if self.unwrap:
                        deg_out = _unwrap_deg(self._last_deg.get(ros_joint), deg_out)
                    self._last_deg[ros_joint] = deg_out

                sign = float(self.joint_sign.get(ros_joint, 1.0))
                off = float(self.joint_offset_deg.get(ros_joint, 0.0))
                if abs(sign) < 1e-9:
                    sign = 1.0
                deg_in = (deg_out - off) / sign
                rad = math.radians(deg_in)

                names.append(ros_joint)
                pos.append(rad)

            if missing:
                now = time.time()
                if (now - self._start_t) >= self.warn_grace_period_s and (now - self._last_warn_t) > 1.0:
                    self._last_warn_t = now
                    rospy.logwarn("NT feedback missing joints (using last/0): %s", ",".join(missing))

            if names and (self.publish_all_joints or not missing):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                if self.frame_id:
                    msg.header.frame_id = str(self.frame_id)
                msg.name = names
                msg.position = pos
                # Append fixed extra joints
                for ej in self.extra_joints or []:
                    try:
                        jn = str(ej.get("name"))
                        if not jn:
                            continue
                        if jn in msg.name:
                            continue
                        jp = float(ej.get("position", 0.0))
                        msg.name.append(jn)
                        msg.position.append(jp)
                    except Exception:
                        continue
                self.pub.publish(msg)

            rate.sleep()


def main():
    rospy.init_node("nt_angles_to_joint_states", anonymous=False)
    node = NTAnglesToJointStates()
    node.spin()


if __name__ == "__main__":
    main()




#!/usr/bin/env python3
import rospy
import yaml
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class InteractiveXyzMarker:
    """
    RViz 中生成一个可拖拽的 3D 点（Interactive Marker）。
    拖动时会持续收到反馈，并实时发布 PointStamped（默认 /arm_xyz_goal）。
    """

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.topic = rospy.get_param("~topic", "/arm_xyz_goal")
        self.name = rospy.get_param("~name", "arm_xyz_goal")
        self.scale = float(rospy.get_param("~scale", 0.25))
        init = rospy.get_param("~initial_xyz", [0.2, 0.0, 0.2])
        # roslaunch 用 <param value="[...]"/> 时会把它当 string；这里兼容两种写法
        if isinstance(init, str):
            init = yaml.safe_load(init)
        self._last = (float(init[0]), float(init[1]), float(init[2]))

        self.pub = rospy.Publisher(self.topic, PointStamped, queue_size=1)
        self.server = InteractiveMarkerServer("arm_xyz_marker")
        self._make_marker()
        self.server.applyChanges()

        rospy.loginfo("interactive_xyz_marker ready. Drag marker in RViz. topic=%s frame=%s", self.topic, self.frame_id)

    def _make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = self.name
        int_marker.description = "Drag to publish XYZ"
        int_marker.scale = self.scale
        int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z = self._last
        int_marker.pose.orientation.w = 1.0

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = self.scale * 0.25
        sphere.scale.y = self.scale * 0.25
        sphere.scale.z = self.scale * 0.25
        sphere.color.r = 0.1
        sphere.color.g = 0.8
        sphere.color.b = 0.2
        sphere.color.a = 0.9

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(sphere)

        move_ctrl = InteractiveMarkerControl()
        move_ctrl.name = "move_3d"
        move_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_3D

        int_marker.controls.append(vis_ctrl)
        int_marker.controls.append(move_ctrl)

        self.server.insert(int_marker, self._feedback_cb)

    def _publish(self, x: float, y: float, z: float):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.pub.publish(msg)

    def _feedback_cb(self, fb):
        x = float(fb.pose.position.x)
        y = float(fb.pose.position.y)
        z = float(fb.pose.position.z)
        self._last = (x, y, z)
        self._publish(x, y, z)


def main():
    rospy.init_node("interactive_xyz_marker", anonymous=False)
    InteractiveXyzMarker()
    rospy.spin()


if __name__ == "__main__":
    main()




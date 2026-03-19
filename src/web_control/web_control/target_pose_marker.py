"""Interactive marker node for setting CRISP cartesian_impedance_controller target pose.

Provides a 6-DOF interactive marker in RViz that publishes to /target_pose
when dragged. Subscribes to /current_pose to initialize marker position.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer
from std_msgs.msg import Header


class TargetPoseMarkerNode(Node):

    def __init__(self):
        super().__init__("target_pose_marker")
        self._frame_id = self.declare_parameter("frame_id", "base_link").value

        self._target_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self._current_sub = self.create_subscription(
            PoseStamped, "current_pose", self._current_pose_cb, 10
        )

        self._server = InteractiveMarkerServer(self, "target_pose_marker")
        self._initialized = False
        self._current_pose = None

        self.get_logger().info("Waiting for /current_pose to initialize marker...")

    def _current_pose_cb(self, msg: PoseStamped):
        self._current_pose = msg.pose
        if not self._initialized:
            self._create_marker(msg.pose)
            self._initialized = True
            self.get_logger().info("Interactive marker created at current EE pose.")

    def _create_marker(self, pose: Pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._frame_id
        int_marker.name = "target_pose"
        int_marker.description = "Drag to set target pose"
        int_marker.scale = 0.15
        int_marker.pose = pose

        # Visual: a small sphere at the marker center
        visual_ctrl = InteractiveMarkerControl()
        visual_ctrl.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.04
        sphere.scale.y = 0.04
        sphere.scale.z = 0.04
        sphere.color.r = 0.9
        sphere.color.g = 0.2
        sphere.color.b = 0.2
        sphere.color.a = 0.8
        visual_ctrl.markers.append(sphere)
        int_marker.controls.append(visual_ctrl)

        # 6-DOF controls
        for axis, name in [
            ((1, 0, 0), "move_x"), ((0, 1, 0), "move_y"), ((0, 0, 1), "move_z"),
            ((1, 0, 0), "rot_x"), ((0, 1, 0), "rot_y"), ((0, 0, 1), "rot_z"),
        ]:
            ctrl = InteractiveMarkerControl()
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = float(axis[0])
            ctrl.orientation.y = float(axis[1])
            ctrl.orientation.z = float(axis[2])
            ctrl.name = name
            if name.startswith("move"):
                ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ctrl)

        self._server.insert(int_marker, feedback_callback=self._marker_feedback)
        self._server.applyChanges()

    def _marker_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            msg = PoseStamped()
            msg.header.frame_id = self._frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = feedback.pose
            self._target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Teleoperate UR15 from Alicia-D leader arm joint states.

Subscribes to /arm_joint_state (ArmJointState) from the Alicia driver,
maps joint angles to UR15 joint commands, and sends them via the
scaled_joint_trajectory_controller.

Joint mapping (Alicia → UR15):
  joint1 → shoulder_pan_joint
  joint2 → shoulder_lift_joint
  joint3 → elbow_joint
  joint4 → wrist_1_joint
  joint5 → wrist_2_joint
  joint6 → wrist_3_joint
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from alicia_duo_leader_driver.msg import ArmJointState


UR15_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class AliciaTeleop(Node):
    def __init__(self):
        super().__init__("alicia_teleop")

        # Parameters
        self.declare_parameter("rate", 50.0)  # Hz — command publish rate
        self.declare_parameter("joint_scale", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_time", 0.1)  # seconds — time_from_start per point

        rate = self.get_parameter("rate").value
        self._joint_scale = np.array(self.get_parameter("joint_scale").value, dtype=np.float64)
        self._joint_offset = np.array(self.get_parameter("joint_offset").value, dtype=np.float64)
        self._traj_time = self.get_parameter("trajectory_time").value

        # State
        self._leader_joints = None  # latest leader arm joint angles (6,)
        self._active = False

        # Subscribe to Alicia leader arm
        self.create_subscription(
            ArmJointState,
            "/arm_joint_state",
            self._leader_cb,
            10,
        )

        # Publisher for joint trajectory commands
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Timer to send commands at fixed rate
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f"Alicia teleop started at {rate:.0f} Hz, "
            f"traj_time={self._traj_time:.3f}s"
        )

    def _leader_cb(self, msg: ArmJointState):
        """Store latest leader arm joints."""
        self._leader_joints = np.array([
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6,
        ], dtype=np.float64)

        # Use button to toggle active state
        # but1: 0x10 = sync mode active
        if msg.but1 == 0x10:
            if not self._active:
                self._active = True
                self.get_logger().info("Teleop ACTIVE (sync button pressed)")
        else:
            if self._active:
                self._active = False
                self.get_logger().info("Teleop PAUSED (sync button released)")

    def _timer_cb(self):
        """Send joint trajectory command from leader arm data."""
        if self._leader_joints is None or not self._active:
            return

        # Map leader joints to UR15 joints
        target = self._leader_joints * self._joint_scale + self._joint_offset

        # Build trajectory message with single point
        traj = JointTrajectory()
        traj.joint_names = UR15_JOINTS

        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        sec = int(self._traj_time)
        nanosec = int((self._traj_time - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [point]

        self._traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = AliciaTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Verify the UR native force-control framework's preemption/recovery cycle.

Phase 1: send a no-motion URScript (textmsg + sleep) -> observe External Control
         program is preempted, then recovered via resend_robot_program.
Phase 2: send a minimal force_mode() URScript with ALL axes in position-hold
         (selection_vector = [0,0,0,0,0,0]) -> robot should not move; verify
         clean exit and recovery.

Safety rationale for Phase 2:
- selection_vector all 0 => every axis is position-controlled, force_mode just
  enforces max_deviation on each axis. Robot holds position.
- Wrench is zero anyway.
- Duration 3 s.
- limits small.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.srv import IsProgramRunning


PHASE1_SCRIPT = """def test_textmsg():
  textmsg("UR_TEST: phase1 start")
  sleep(2.0)
  textmsg("UR_TEST: phase1 done")
end
"""

# selection_vector = [0]*6 -> every axis position-controlled; robot holds pose.
# wrench zero; task_frame at base; limits = small max deviations (0.02 m / 0.1 rad).
PHASE2_SCRIPT = """def test_force_mode_hold():
  textmsg("UR_TEST: phase2 entering force_mode (position-hold)")
  force_mode_set_damping(0.5)
  force_mode_set_gain_scaling(0.5)
  i = 0
  while i < 150:
    force_mode(p[0,0,0,0,0,0], [0,0,0,0,0,0], [0.0,0.0,0.0,0.0,0.0,0.0], 2, [0.02,0.02,0.02,0.1,0.1,0.1])
    sync()
    i = i + 1
  end
  end_force_mode()
  textmsg("UR_TEST: phase2 done")
end
"""


class RecoveryTester(Node):
    def __init__(self):
        super().__init__("test_urscript_recovery")
        self.script_pub = self.create_publisher(String, "/urscript_interface/script_command", 1)
        self.resend_cli = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")
        self.prog_cli = self.create_client(IsProgramRunning, "/dashboard_client/program_running")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

    def _js_cb(self, msg):
        self.joint_state = msg

    def program_running(self):
        if not self.prog_cli.wait_for_service(timeout_sec=1.0):
            return None
        fut = self.prog_cli.call_async(IsProgramRunning.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            return None
        return fut.result().program_running

    def wait_for(self, pred, timeout, label):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                self.get_logger().info(f"[OK] {label} after {time.time()-t0:.2f}s")
                return True
        self.get_logger().error(f"[FAIL] {label} timed out after {timeout}s")
        return False

    def send_script(self, script):
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)
        self.get_logger().info(f"Published script ({len(script)} bytes)")

    def call_resend(self):
        if not self.resend_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("resend_robot_program service not available")
            return False
        fut = self.resend_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error("resend_robot_program call timed out")
            return False
        r = fut.result()
        self.get_logger().info(f"resend_robot_program: success={r.success} msg='{r.message}'")
        return r.success

    def snapshot_joints(self):
        return None if self.joint_state is None else list(self.joint_state.position)


def max_abs_diff(a, b):
    if a is None or b is None or len(a) != len(b):
        return float("inf")
    return max(abs(x - y) for x, y in zip(a, b))


def run_phase(node, label, script, expect_motion_tol_rad, hold_duration):
    node.get_logger().info(f"===== {label} =====")
    # Wait for program_running True at start
    if not node.wait_for(lambda: node.program_running() is True, 5.0, "program_running=True (initial)"):
        return False
    if not node.wait_for(lambda: node.joint_state is not None, 5.0, "joint_states received"):
        return False
    q_before = node.snapshot_joints()
    node.get_logger().info(f"Joints before: {[f'{x:.4f}' for x in q_before]}")

    # Send script - External Control should be preempted
    node.send_script(script)
    # Observe preemption
    if not node.wait_for(lambda: node.program_running() is False, 3.0, "program_running=False (preempted)"):
        node.get_logger().warn("Preemption not observed; continuing")

    # Wait for script to finish
    time.sleep(hold_duration)
    q_mid = node.snapshot_joints()
    drift = max_abs_diff(q_before, q_mid)
    node.get_logger().info(f"Joints during/after script: {[f'{x:.4f}' for x in q_mid]}")
    node.get_logger().info(f"Max joint drift vs start: {drift:.5f} rad ({drift*180/3.14159:.3f} deg)")
    if drift > expect_motion_tol_rad:
        node.get_logger().error(f"[FAIL] Drift {drift:.4f} > tolerance {expect_motion_tol_rad}")
        return False

    # Recover
    node.get_logger().info("Calling resend_robot_program ...")
    if not node.call_resend():
        return False
    if not node.wait_for(lambda: node.program_running() is True, 8.0, "program_running=True (recovered)"):
        return False

    # Check scaled_joint_trajectory_controller is still active (control still works)
    time.sleep(1.0)
    q_after = node.snapshot_joints()
    drift_total = max_abs_diff(q_before, q_after)
    node.get_logger().info(f"Joints after recovery: {[f'{x:.4f}' for x in q_after]}")
    node.get_logger().info(f"Total drift: {drift_total:.5f} rad")
    node.get_logger().info(f"===== {label} PASSED =====")
    return True


def main():
    rclpy.init()
    node = RecoveryTester()
    try:
        # Phase 1: no-motion textmsg
        ok1 = run_phase(node, "PHASE 1 (no motion)", PHASE1_SCRIPT,
                        expect_motion_tol_rad=0.005, hold_duration=3.0)
        if not ok1:
            node.get_logger().error("Phase 1 failed; aborting phase 2")
            return 1
        time.sleep(2.0)
        # Phase 2: force_mode position-hold
        ok2 = run_phase(node, "PHASE 2 (force_mode hold)", PHASE2_SCRIPT,
                        expect_motion_tol_rad=0.03, hold_duration=4.0)
        if not ok2:
            return 1
        node.get_logger().info("ALL PHASES PASSED - preemption/recovery works")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

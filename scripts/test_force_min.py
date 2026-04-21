#!/usr/bin/env python3
"""
Minimal phase 3: force_mode with constant small Fz for 2 s, watch TCP z.
Does not rely on program_running detection -- just watches TCP pose.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


# Simplest possible: push +Fz with small force for 2s. Purely compliant in Z.
SCRIPT = """def t():
  textmsg("UR_TEST: constant Fz")
  force_mode_set_damping(0.5)
  force_mode_set_gain_scaling(0.5)
  i = 0
  while i < 1000:
    force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], [0.0, 0.0, 15.0, 0.0, 0.0, 0.0], 2, [0.1,0.1,0.05,0.2,0.2,0.2])
    sync()
    i = i + 1
  end
  end_force_mode()
  textmsg("UR_TEST: done")
end
"""


class T(Node):
    def __init__(self):
        super().__init__("t")
        self.pub = self.create_publisher(String, "/urscript_interface/script_command", 1)
        self.resend = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")
        self.tcp = None
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._cb, 10)

    def _cb(self, m):
        p = m.pose.position
        self.tcp = (p.x, p.y, p.z, time.time())


def main():
    rclpy.init()
    n = T()
    # Wait for TCP
    t0 = time.time()
    while n.tcp is None and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.tcp is None:
        print("no tcp pose")
        return 1
    z0 = n.tcp[2]
    x0, y0 = n.tcp[0], n.tcp[1]
    print(f"TCP start: x={x0:.4f} y={y0:.4f} z={z0:.4f}")

    msg = String(); msg.data = SCRIPT
    n.pub.publish(msg)
    print("script sent; watching TCP z for 3s")

    t0 = time.time()
    zmax = z0
    zmin = z0
    samples = 0
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.tcp is not None and n.tcp[3] > t0 - 0.1:
            z = n.tcp[2]
            samples += 1
            if z > zmax: zmax = z
            if z < zmin: zmin = z
    z1 = n.tcp[2]
    print(f"samples={samples}  z_min={zmin:.4f}  z_max={zmax:.4f}  z_end={z1:.4f}")
    print(f"dz_peak_up  = {(zmax-z0)*1000:+.2f} mm")
    print(f"dz_peak_dn  = {(zmin-z0)*1000:+.2f} mm")
    print(f"dz_final    = {(z1-z0)*1000:+.2f} mm")

    # Recover
    n.resend.wait_for_service(timeout_sec=3)
    fut = n.resend.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    print(f"resend: {fut.result().success if fut.result() else 'fail'}")

    # Verify control recovered
    time.sleep(1.5)
    print(f"TCP after recovery: z={n.tcp[2]:.4f}")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

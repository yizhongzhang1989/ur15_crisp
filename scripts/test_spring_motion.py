#!/usr/bin/env python3
"""
Test the PROPOSED control law: on-robot spring + force_mode() at 500 Hz.
URScript computes wrench = K * (target - get_actual_tcp_pose()) each cycle
and calls force_mode() to enact it. Target pose is hardcoded to the robot's
CURRENT pose plus [0, 0, dz] so we can observe the error-driven motion.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


SCRIPT_TEMPLATE = """def spring():
  textmsg("UR_TEST: spring start")
  start = get_actual_tcp_pose()
  tx = start[0]
  ty = start[1]
  tz = start[2] + ({dz:.5f})
  K = {K:.1f}
  mf = {mf:.1f}
  force_mode_set_damping(0.5)
  force_mode_set_gain_scaling(0.5)
  i = 0
  while i < {n}:
    c = get_actual_tcp_pose()
    fx = K * (tx - c[0])
    fy = K * (ty - c[1])
    fz = K * (tz - c[2])
    if fx > mf:
      fx = mf
    end
    if fx < -mf:
      fx = -mf
    end
    if fy > mf:
      fy = mf
    end
    if fy < -mf:
      fy = -mf
    end
    if fz > mf:
      fz = mf
    end
    if fz < -mf:
      fz = -mf
    end
    force_mode(p[0,0,0,0,0,0], [1,1,1,0,0,0], [fx, fy, fz, 0.0, 0.0, 0.0], 2, [0.08,0.08,0.08,0.2,0.2,0.2])
    sync()
    i = i + 1
  end
  end_force_mode()
  textmsg("UR_TEST: spring done")
end
"""


class T(Node):
    def __init__(self):
        super().__init__("ts")
        self.pub = self.create_publisher(String, "/urscript_interface/script_command", 1)
        self.resend = self.create_client(Trigger, "/io_and_status_controller/resend_robot_program")
        self.tcp = None
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._cb, 10)

    def _cb(self, m):
        p = m.pose.position
        self.tcp = (p.x, p.y, p.z, time.time())


def main():
    import sys
    dz = float(sys.argv[1]) if len(sys.argv) > 1 else -0.012
    rclpy.init()
    n = T()
    t0 = time.time()
    while n.tcp is None and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    p0 = n.tcp
    print(f"TCP start: x={p0[0]:.4f} y={p0[1]:.4f} z={p0[2]:.4f}")

    script = SCRIPT_TEMPLATE.format(dz=dz, K=400.0, mf=20.0, n=1500)
    n.pub.publish(String(data=script))
    print(f"spring script sent (target dz={dz*1000:+.1f} mm); watching 3.5s")

    t0 = time.time()
    zs = []
    while time.time() - t0 < 3.5:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.tcp is not None and n.tcp[3] > t0 - 0.1:
            zs.append(n.tcp[2])
    if zs:
        print(f"samples={len(zs)}  z_start={zs[0]:.4f}  z_min={min(zs):.4f}  z_max={max(zs):.4f}  z_end={zs[-1]:.4f}")
        print(f"target_z = {p0[2] + dz:.4f}")
        print(f"achieved dz = {(zs[-1]-p0[2])*1000:+.2f} mm (target {dz*1000:+.1f} mm)")
        print(f"steady-state error = {(zs[-1] - (p0[2]+dz))*1000:+.2f} mm")

    fut = n.resend.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    print(f"resend: {fut.result().success if fut.result() else 'fail'}")
    time.sleep(1.0)
    print(f"TCP after recovery: z={n.tcp[2]:.4f}")

    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

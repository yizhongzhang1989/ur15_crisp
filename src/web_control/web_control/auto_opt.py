"""Auto optimization framework for controller parameters.

Phase 1 — trajectory recording and replay (single trajectory in tmp/param_opt/).
Study management, Optuna integration, and objective loop will be added later.
"""

import os
import threading
import time

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

from common.workspace import get_workspace_root


def _param_opt_dir():
    root = get_workspace_root()
    d = os.path.join(root, "tmp", "param_opt")
    os.makedirs(d, exist_ok=True)
    return d


def _traj_path():
    return os.path.join(_param_opt_dir(), "reference_trajectory.npz")


class OptimizationManager:
    """Manages trajectory recording, replay and (later) parameter optimization.

    Single reference trajectory stored at tmp/param_opt/reference_trajectory.npz.

    Dependencies are injected by the WebControlServer:
      robot: crisp_py Robot
      node: ROS node for subs
      set_cmd_target_fn: callable(joints_list) → publishes /cmd_target_joint
      owner: WebControlServer (optional) — to pause teleop during replay
    """

    def __init__(self, robot, node, set_cmd_target_fn, owner=None):
        self.robot = robot
        self.node = node
        self._set_cmd_target = set_cmd_target_fn
        self._owner = owner
        self._lock = threading.Lock()

        # Recording state (buffers; not written to disk until save())
        self._recording = False
        self._rec_start_wall = 0.0
        self._rec_cmd_joint = []   # list of (t_rel, [6])
        self._rec_cmd_pose = []    # list of (t_rel, {x,y,z,qx,qy,qz,qw})
        self._rec_actual = []      # list of (t_rel, [6])
        self._rec_sub_cmd_joint = None
        self._rec_sub_cmd_pose = None
        self._rec_sub_joint_state = None
        self._rec_duration = 0.0   # final duration after stop
        self._saved_meta_cache = None  # cached saved trajectory metadata

        # Replay state
        self._replaying = False
        self._replay_progress = 0.0
        self._replay_abort = False
        self._replay_thread = None

    # ------------------------------------------------------------------
    # Public status
    # ------------------------------------------------------------------
    def status(self):
        with self._lock:
            saved = os.path.isfile(_traj_path())
            in_memory = len(self._rec_cmd_joint) > 0 and not self._recording
            saved_meta = None
            if saved:
                if self._saved_meta_cache is not None:
                    saved_meta = self._saved_meta_cache
                else:
                    try:
                        with np.load(_traj_path(), allow_pickle=True) as z:
                            saved_meta = {
                                "duration": float(z["duration"]),
                                "n_cmd_joint": int(len(z["cmd_joint_t"])),
                                "n_cmd_pose": int(len(z["cmd_pose_t"])),
                                "n_actual": int(len(z["actual_t"])),
                                "recorded_at": str(z["recorded_at"]) if "recorded_at" in z.files else "",
                            }
                        self._saved_meta_cache = saved_meta
                    except Exception:
                        saved_meta = {"error": "unreadable"}
            else:
                self._saved_meta_cache = None
            return {
                "recording": self._recording,
                "rec_elapsed": round(time.monotonic() - self._rec_start_wall, 2) if self._recording else round(self._rec_duration, 2),
                "rec_frames": {
                    "cmd_joint": len(self._rec_cmd_joint),
                    "cmd_pose": len(self._rec_cmd_pose),
                    "actual": len(self._rec_actual),
                },
                "has_buffer": in_memory,
                "saved": saved,
                "saved_meta": saved_meta,
                "replaying": self._replaying,
                "replay_progress": round(self._replay_progress, 3),
            }

    def traj_path(self) -> str:
        return _traj_path()

    @property
    def is_replaying(self) -> bool:
        return self._replaying

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def start_recording(self):
        with self._lock:
            if self._recording:
                raise RuntimeError("Already recording")
            if self._replaying:
                raise RuntimeError("Cannot record while replaying")

            self._rec_cmd_joint = []
            self._rec_cmd_pose = []
            self._rec_actual = []
            self._rec_duration = 0.0
            self._rec_start_wall = time.monotonic()
            self._recording = True

        cb_group = ReentrantCallbackGroup()
        self._rec_sub_cmd_joint = self.node.create_subscription(
            JointState, "/cmd_target_joint", self._rec_cmd_joint_cb, 50,
            callback_group=cb_group,
        )
        self._rec_sub_cmd_pose = self.node.create_subscription(
            PoseStamped, "/cmd_target_pose", self._rec_cmd_pose_cb, 50,
            callback_group=cb_group,
        )
        self._rec_sub_joint_state = self.node.create_subscription(
            JointState, "/joint_states", self._rec_joint_state_cb, 50,
            callback_group=cb_group,
        )

    def stop_recording(self):
        """Stop recording (buffer kept in memory, not yet saved)."""
        with self._lock:
            if not self._recording:
                return False
            self._recording = False
            self._rec_duration = time.monotonic() - self._rec_start_wall
            for sub in (self._rec_sub_cmd_joint, self._rec_sub_cmd_pose, self._rec_sub_joint_state):
                if sub is not None:
                    try:
                        self.node.destroy_subscription(sub)
                    except Exception:
                        pass
            self._rec_sub_cmd_joint = None
            self._rec_sub_cmd_pose = None
            self._rec_sub_joint_state = None
            print(f"[auto_opt] Recording stopped: {self._rec_duration:.2f}s "
                  f"(cmd_joint={len(self._rec_cmd_joint)}, cmd_pose={len(self._rec_cmd_pose)}, "
                  f"actual={len(self._rec_actual)})")
        return True

    def _rec_cmd_joint_cb(self, msg: JointState):
        if not self._recording:
            return
        t = time.monotonic() - self._rec_start_wall
        names = self.robot.config.joint_names
        q = [0.0] * len(names)
        for jname, p in zip(msg.name, msg.position):
            if jname in names:
                q[names.index(jname)] = float(p)
        self._rec_cmd_joint.append((t, q))

    def _rec_cmd_pose_cb(self, msg: PoseStamped):
        if not self._recording:
            return
        t = time.monotonic() - self._rec_start_wall
        p = msg.pose
        self._rec_cmd_pose.append((t, {
            "x": float(p.position.x), "y": float(p.position.y), "z": float(p.position.z),
            "qx": float(p.orientation.x), "qy": float(p.orientation.y),
            "qz": float(p.orientation.z), "qw": float(p.orientation.w),
        }))

    def _rec_joint_state_cb(self, msg: JointState):
        if not self._recording:
            return
        t = time.monotonic() - self._rec_start_wall
        names = self.robot.config.joint_names
        q = [0.0] * len(names)
        for jname, p in zip(msg.name, msg.position):
            if jname in names:
                q[names.index(jname)] = float(p)
        self._rec_actual.append((t, q))

    # ------------------------------------------------------------------
    # Save / Delete
    # ------------------------------------------------------------------
    def save(self):
        """Write current in-memory recording to disk."""
        with self._lock:
            if self._recording:
                raise RuntimeError("Stop recording before saving")
            if not self._rec_cmd_joint and not self._rec_cmd_pose:
                raise RuntimeError("Nothing to save — record a trajectory first")
            cmd_joint = list(self._rec_cmd_joint)
            cmd_pose = list(self._rec_cmd_pose)
            actual = list(self._rec_actual)
            duration = self._rec_duration

        cmd_joint_t = np.array([t for t, _ in cmd_joint], dtype=np.float64)
        cmd_joint_q = np.array([q for _, q in cmd_joint], dtype=np.float64) if cmd_joint else np.zeros((0, 6))
        cmd_pose_t = np.array([t for t, _ in cmd_pose], dtype=np.float64)
        cmd_pose_v = np.array([
            [p["x"], p["y"], p["z"], p["qx"], p["qy"], p["qz"], p["qw"]]
            for _, p in cmd_pose
        ], dtype=np.float64) if cmd_pose else np.zeros((0, 7))
        actual_t = np.array([t for t, _ in actual], dtype=np.float64)
        actual_q = np.array([q for _, q in actual], dtype=np.float64) if actual else np.zeros((0, 6))

        path = _traj_path()
        np.savez(
            path,
            duration=duration,
            recorded_at=time.strftime("%Y-%m-%d %H:%M:%S"),
            cmd_joint_t=cmd_joint_t, cmd_joint_q=cmd_joint_q,
            cmd_pose_t=cmd_pose_t, cmd_pose_v=cmd_pose_v,
            actual_t=actual_t, actual_q=actual_q,
        )
        print(f"[auto_opt] Saved trajectory: {path}")
        self._saved_meta_cache = None  # invalidate cache
        return path

    def delete(self):
        """Remove the saved trajectory file and clear the in-memory buffer."""
        with self._lock:
            if self._recording:
                raise RuntimeError("Stop recording before deleting")
            if self._replaying:
                raise RuntimeError("Stop replay before deleting")
            self._rec_cmd_joint = []
            self._rec_cmd_pose = []
            self._rec_actual = []
            self._rec_duration = 0.0
        path = _traj_path()
        if os.path.isfile(path):
            os.remove(path)
            print(f"[auto_opt] Deleted trajectory: {path}")
            self._saved_meta_cache = None  # invalidate cache
            return True
        return False

    # ------------------------------------------------------------------
    # Replay (joint mode only — cartesian is computed from FK in the timer)
    # ------------------------------------------------------------------
    def start_replay(self, alignment_time_sec: float = 2.0, settle_time_sec: float = 0.5):
        with self._lock:
            if self._recording:
                raise RuntimeError("Cannot replay while recording")
            if self._replaying:
                raise RuntimeError("Already replaying")

            path = _traj_path()
            if not os.path.isfile(path):
                raise FileNotFoundError("No saved trajectory — record and save first")

            with np.load(path, allow_pickle=True) as z:
                frames_t = z["cmd_joint_t"].copy()
                frames_q = z["cmd_joint_q"].copy()

            if len(frames_t) == 0:
                raise ValueError("Saved trajectory has no cmd_target_joint data")

            self._replaying = True
            self._replay_abort = False
            self._replay_progress = 0.0

        self._replay_thread = threading.Thread(
            target=self._replay_loop,
            args=(frames_t, frames_q, float(alignment_time_sec), float(settle_time_sec)),
            daemon=True,
        )
        self._replay_thread.start()

    def abort_replay(self):
        with self._lock:
            if not self._replaying:
                return
            self._replay_abort = True

    def _replay_loop(self, frames_t, frames_q, alignment_sec, settle_sec):
        print(f"[auto_opt] Replay START ({len(frames_t)} frames, {frames_t[-1]:.2f}s)")
        try:
            first_q = frames_q[0]
            last_q = frames_q[-1]

            # ---- Alignment phase ----
            print(f"[auto_opt] Alignment ({alignment_sec:.1f}s)")
            t0 = time.monotonic()
            while time.monotonic() - t0 < alignment_sec:
                if self._replay_abort:
                    return
                self._set_cmd_target(list(first_q))
                time.sleep(0.02)

            # ---- Evaluation phase ----
            print(f"[auto_opt] Evaluation ({frames_t[-1]:.2f}s)")
            t_start = time.monotonic()
            for i in range(len(frames_t)):
                if self._replay_abort:
                    return
                target_rel = float(frames_t[i] - frames_t[0])
                now_rel = time.monotonic() - t_start
                sleep_for = target_rel - now_rel
                if sleep_for > 0:
                    time.sleep(min(sleep_for, 0.1))
                self._set_cmd_target(list(frames_q[i]))
                self._replay_progress = (i + 1) / len(frames_t)

            # ---- Settle phase ----
            print(f"[auto_opt] Settle ({settle_sec:.1f}s)")
            t1 = time.monotonic()
            while time.monotonic() - t1 < settle_sec:
                if self._replay_abort:
                    return
                self._set_cmd_target(list(last_q))
                time.sleep(0.02)
            print("[auto_opt] Replay complete")
        except Exception as e:
            print(f"[auto_opt] Replay error: {e}")
        finally:
            with self._lock:
                self._replaying = False
                self._replay_progress = 0.0

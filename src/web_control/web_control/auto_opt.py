"""Auto optimization framework for controller parameters.

Records a reference trajectory via Alicia teleop, replays it with different
controller parameters suggested by Optuna, measures joint tracking RMSE,
and finds optimal parameters.

Storage: tmp/param_opt/
"""

import os
import threading
import time

import json
import shutil
import numpy as np
import optuna
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

from common.workspace import get_workspace_root

# Suppress Optuna's verbose trial logging
optuna.logging.set_verbosity(optuna.logging.WARNING)


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

        # Optimization state
        self._optimizing = False
        self._opt_abort = False
        self._opt_thread = None
        self._opt_config = None       # dict from /api/opt/start
        self._opt_study = None        # optuna.Study
        self._opt_trial_num = 0       # current trial number (1-based)
        self._opt_n_trials = 0        # total requested
        self._opt_current_params = {} # param name → value being tested
        self._opt_best_params = {}    # param name → best value found
        self._opt_best_value = None   # best RMSE
        self._opt_best_trial = None   # trial number of best
        self._opt_history = []        # list of {trial, params, value, status}
        self._opt_message = ""        # status message for UI
        self._opt_consecutive_failures = 0
        self._opt_exp_dir = None       # current experiment directory

        # Safety monitoring via dashboard_client services (topics are inactive)
        self._safety_mode = 1   # 1 = NORMAL
        self._robot_mode = 7    # 7 = RUNNING
        self._safety_answer = ""
        self._robot_answer = ""
        self._safety_client = None
        self._robot_mode_client = None
        self._safety_timer = None
        if node is not None:
            try:
                from ur_dashboard_msgs.srv import GetSafetyMode, GetRobotMode
                self._safety_client = node.create_client(
                    GetSafetyMode, "/dashboard_client/get_safety_mode")
                self._robot_mode_client = node.create_client(
                    GetRobotMode, "/dashboard_client/get_robot_mode")
                # Poll every 1 second
                self._safety_timer = node.create_timer(
                    1.0, self._poll_robot_state,
                    callback_group=ReentrantCallbackGroup(),
                )
            except ImportError:
                print("[auto_opt] ur_dashboard_msgs not available — no safety monitoring")

    def _poll_robot_state(self):
        """Poll dashboard services for safety and robot mode."""
        if self._safety_client is not None and self._safety_client.service_is_ready():
            try:
                from ur_dashboard_msgs.srv import GetSafetyMode
                future = self._safety_client.call_async(GetSafetyMode.Request())
                future.add_done_callback(self._on_safety_response)
            except Exception:
                pass
        if self._robot_mode_client is not None and self._robot_mode_client.service_is_ready():
            try:
                from ur_dashboard_msgs.srv import GetRobotMode
                future = self._robot_mode_client.call_async(GetRobotMode.Request())
                future.add_done_callback(self._on_robot_mode_response)
            except Exception:
                pass

    def _on_safety_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self._safety_mode = resp.safety_mode.mode
                self._safety_answer = resp.answer
        except Exception:
            pass

    def _on_robot_mode_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self._robot_mode = resp.robot_mode.mode
                self._robot_answer = resp.answer
        except Exception:
            pass

    @property
    def is_robot_ok(self) -> bool:
        """True if safety_mode is NORMAL (1) and robot_mode is RUNNING (7)."""
        return self._safety_mode == 1 and self._robot_mode == 7

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
                "optimizing": self._optimizing,
                "opt_trial": self._opt_trial_num,
                "opt_n_trials": self._opt_n_trials,
                "opt_current_params": dict(self._opt_current_params),
                "opt_best_params": dict(self._opt_best_params),
                "opt_best_value": self._opt_best_value,
                "opt_best_trial": self._opt_best_trial,
                "opt_message": self._opt_message,
                "safety_mode": self._safety_mode,
                "robot_mode": self._robot_mode,
                "safety_answer": self._safety_answer,
                "robot_answer": self._robot_answer,
                "robot_ok": self.is_robot_ok,
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

    # ------------------------------------------------------------------
    # Optimization
    # ------------------------------------------------------------------
    @property
    def is_optimizing(self) -> bool:
        return self._optimizing

    def opt_history(self):
        with self._lock:
            return list(self._opt_history)

    def start_optimization(self, config: dict, param_client_fn, resume_exp: str = None):
        """Start the optimization loop.

        config: {
            "controller": str,
            "params": { "param.name": {"min": float, "max": float, "ref": float}, ... },
            "n_trials": int,
            "alignment_time_sec": float,
            "settle_time_sec": float,
            "max_consecutive_failures": int,
        }
        param_client_fn: callable(controller_name) → ParametersClient
        resume_exp: if set, resume from this experiment directory name
        """
        with self._lock:
            if self._optimizing:
                raise RuntimeError("Already optimizing")
            if self._recording:
                raise RuntimeError("Cannot optimize while recording")
            if self._replaying:
                raise RuntimeError("Cannot optimize while replaying")
            path = _traj_path()
            if not os.path.isfile(path):
                raise FileNotFoundError("No saved trajectory")

            controller = config.get("controller", "").strip()
            params = config.get("params", {})
            n_trials = int(config.get("n_trials", 20))
            if not controller:
                raise ValueError("controller required")
            if not params:
                raise ValueError("At least one parameter required")
            if n_trials < 1:
                raise ValueError("n_trials must be ≥ 1")
            for pname, bounds in params.items():
                if bounds["min"] >= bounds["max"]:
                    raise ValueError(f"{pname}: min must be < max")

            self._opt_config = {
                "controller": controller,
                "params": params,
                "n_trials": n_trials,
                "alignment_time_sec": float(config.get("alignment_time_sec", 2.0)),
                "settle_time_sec": float(config.get("settle_time_sec", 0.5)),
                "max_consecutive_failures": int(config.get("max_consecutive_failures", 3)),
            }
            self._optimizing = True
            self._opt_abort = False
            self._opt_trial_num = 0
            self._opt_n_trials = n_trials
            self._opt_current_params = {}
            self._opt_best_params = {}
            self._opt_best_value = None
            self._opt_best_trial = None
            self._opt_history = []
            self._opt_message = "Starting..."
            self._opt_consecutive_failures = 0

            # Create or resume experiment directory
            if resume_exp:
                self._opt_exp_dir = os.path.join(_param_opt_dir(), resume_exp)
                if not os.path.isdir(self._opt_exp_dir):
                    raise FileNotFoundError(f"Experiment not found: {resume_exp}")
                # Count existing trials to set offset
                trial_dir = os.path.join(self._opt_exp_dir, "trial_data")
                existing = len([f for f in os.listdir(trial_dir) if f.endswith(".npz")]) if os.path.isdir(trial_dir) else 0
                self._opt_trial_num = existing
                self._opt_message = f"Resuming from trial {existing}..."
                # Rebuild history from existing trial files
                self._opt_history = self._load_trial_history(trial_dir)
                # Find best from history
                for h in self._opt_history:
                    if h["status"] == "ok" and h["value"] is not None:
                        if self._opt_best_value is None or h["value"] < self._opt_best_value:
                            self._opt_best_value = round(h["value"], 6)
                            self._opt_best_params = dict(h["params"])
                            self._opt_best_trial = h["trial"]
            else:
                exp_name = "exp_" + time.strftime("%Y-%m-%d_%H-%M-%S")
                self._opt_exp_dir = os.path.join(_param_opt_dir(), exp_name)
                os.makedirs(self._opt_exp_dir, exist_ok=True)

        self._opt_thread = threading.Thread(
            target=self._opt_loop,
            args=(param_client_fn,),
            daemon=True,
        )
        self._opt_thread.start()

    def stop_optimization(self):
        with self._lock:
            if not self._optimizing:
                return
            self._opt_abort = True

    def _load_trial_history(self, trial_dir):
        """Load trial history from existing npz files."""
        history = []
        if not os.path.isdir(trial_dir):
            return history
        for fname in sorted(os.listdir(trial_dir)):
            if not fname.endswith(".npz"):
                continue
            try:
                with np.load(os.path.join(trial_dir, fname), allow_pickle=True) as z:
                    history.append({
                        "trial": int(z["trial"]),
                        "params": dict(zip(z["params_keys"].tolist(), z["params_values"].tolist())),
                        "value": float(z["rmse"]) if float(z["rmse"]) >= 0 else None,
                        "status": str(z["status"]),
                    })
            except Exception:
                pass
        return history

    def _opt_loop(self, param_client_fn):
        cfg = self._opt_config
        controller = cfg["controller"]
        params_spec = cfg["params"]
        n_trials = cfg["n_trials"]
        alignment_sec = cfg["alignment_time_sec"]
        settle_sec = cfg["settle_time_sec"]
        max_failures = cfg["max_consecutive_failures"]

        # Load trajectory
        with np.load(_traj_path(), allow_pickle=True) as z:
            frames_t = z["cmd_joint_t"].copy()
            frames_q = z["cmd_joint_q"].copy()

        if len(frames_t) == 0:
            self._opt_message = "Error: no trajectory frames"
            self._optimizing = False
            return

        # Get param client
        try:
            param_client = param_client_fn(controller)
        except Exception as e:
            self._opt_message = f"Error: {e}"
            self._optimizing = False
            return

        # Create Optuna study with SQLite persistence
        db_path = os.path.join(self._opt_exp_dir, "optuna.db")
        storage = f"sqlite:///{db_path}"
        study_name = f"opt_{controller}"
        try:
            self._opt_study = optuna.create_study(
                study_name=study_name,
                storage=storage,
                direction="minimize",
                load_if_exists=True,
            )
        except Exception as e:
            self._opt_message = f"Optuna error: {e}"
            self._optimizing = False
            return

        param_names = sorted(params_spec.keys())
        joint_names = self.robot.config.joint_names

        # Persistent joint state recording for all trials
        actual_data = []
        rec_active = [False]

        def joint_state_cb(msg):
            if not rec_active[0]:
                return
            q = [0.0] * len(joint_names)
            for jname, p in zip(msg.name, msg.position):
                if jname in joint_names:
                    q[joint_names.index(jname)] = float(p)
            actual_data.append((time.monotonic(), q))

        # Create subscription once — stays alive for all trials
        opt_sub = self.node.create_subscription(
            JointState, "/joint_states", joint_state_cb, 50,
            callback_group=ReentrantCallbackGroup(),
        )

        trial_counter = [self._opt_trial_num]  # local counter for this run (may start > 0 on resume)
        existing_trials = trial_counter[0]
        remaining_trials = max(1, n_trials - existing_trials)

        print(f"[auto_opt] Optimization START: {controller}, {remaining_trials} remaining of {n_trials} trials, "
              f"{len(param_names)} params: {param_names}"
              f"{' (RESUME)' if existing_trials > 0 else ''}")

        def objective(trial):
            if self._opt_abort:
                return 100.0  # dummy value; study.stop() prevents further trials

            # Check robot safety before starting trial
            if not self.is_robot_ok:
                self._opt_message = f"FAULT: safety_mode={self._safety_mode}. Stopping."
                print(f"[auto_opt] Robot fault detected (safety_mode={self._safety_mode}), aborting optimization")
                self._opt_abort = True
                return 100.0

            self._opt_trial_num = trial_counter[0] + 1
            trial_counter[0] += 1

            # Suggest params
            suggested = {}
            for pname in param_names:
                spec = params_spec[pname]
                suggested[pname] = trial.suggest_float(pname, spec["min"], spec["max"])
            self._opt_current_params = dict(suggested)
            self._opt_message = f"Trial {self._opt_trial_num}/{n_trials}"

            # Set params on controller
            try:
                tuples = []
                current_vals = param_client.get_parameters(list(suggested.keys()))
                for (k, new_val), cur_val in zip(suggested.items(), current_vals):
                    if isinstance(cur_val, float) and isinstance(new_val, int):
                        new_val = float(new_val)
                    tuples.append((k, new_val))
                param_client.set_parameters(tuples)
            except Exception as e:
                print(f"[auto_opt] Trial {trial.number}: set_parameters failed: {e}")
                self._record_trial(self._opt_trial_num, suggested, None, "param_fail",
                                   None, None, None, None)
                self._opt_consecutive_failures += 1
                if self._opt_consecutive_failures >= max_failures:
                    self._opt_message = f"Paused: {max_failures} consecutive failures"
                    self._opt_abort = True
                return 100.0

            # Small pause for params to take effect
            time.sleep(0.3)

            # Clear and start recording actual joint states
            actual_data.clear()
            rec_active[0] = True
            eval_start = [0.0]  # will be set by _run_trial_replay

            try:
                error = self._run_trial_replay(
                    frames_t, frames_q, alignment_sec, settle_sec,
                    actual_data, rec_active, eval_start,
                )
            finally:
                rec_active[0] = False

            if error is None:
                # Aborted mid-trial
                self._record_trial(self._opt_trial_num, suggested, None, "aborted",
                                   None, None, None, None)
                return 100.0  # penalty; study.stop() prevents further trials

            # Record success
            self._opt_consecutive_failures = 0
            self._record_trial(self._opt_trial_num, suggested, error, "ok",
                               frames_t, frames_q, actual_data, eval_start[0])

            if self._opt_best_value is None or error < self._opt_best_value:
                self._opt_best_value = round(error, 6)
                self._opt_best_params = dict(suggested)
                self._opt_best_trial = trial.number + 1
                self._opt_message = (f"Trial {self._opt_trial_num}/{n_trials} — "
                                     f"NEW BEST: {error:.6f}")
            else:
                self._opt_message = (f"Trial {self._opt_trial_num}/{n_trials} — "
                                     f"RMSE: {error:.6f} (best: {self._opt_best_value:.6f})")

            # Inter-trial pause (check for faults during wait)
            for _ in range(50):  # 50 × 0.02s = 1s
                if self._opt_abort or not self.is_robot_ok:
                    if not self.is_robot_ok:
                        self._opt_message = f"FAULT detected between trials. Stopping."
                        self._opt_abort = True
                    break
                time.sleep(0.02)
            return error

        def stop_callback(study, trial):
            if self._opt_abort:
                study.stop()

        try:
            self._opt_study.optimize(
                objective,
                n_trials=remaining_trials,
                callbacks=[stop_callback],
            )
        except Exception as e:
            print(f"[auto_opt] Optimization error: {e}")
        finally:
            # Always clean up — even on crash
            rec_active[0] = False
            try:
                self.node.destroy_subscription(opt_sub)
            except Exception:
                pass

            # Restore ref params
            try:
                ref_tuples = [(pname, params_spec[pname]["ref"]) for pname in param_names]
                current_vals = param_client.get_parameters([t[0] for t in ref_tuples])
                restore_tuples = []
                for (k, new_val), cur_val in zip(ref_tuples, current_vals):
                    if isinstance(cur_val, float) and isinstance(new_val, int):
                        new_val = float(new_val)
                    restore_tuples.append((k, new_val))
                param_client.set_parameters(restore_tuples)
                print("[auto_opt] Restored ref params")
            except Exception as e:
                print(f"[auto_opt] Failed to restore ref params: {e}")

            with self._lock:
                self._optimizing = False
                if self._opt_abort:
                    self._opt_message = (f"Stopped at trial {self._opt_trial_num}/{n_trials}."
                                         + (f" Best: {self._opt_best_value:.6f}" if self._opt_best_value else ""))
                elif self._opt_best_value is not None:
                    self._opt_message = (f"Done. Best RMSE: {self._opt_best_value:.6f} "
                                         f"(trial {self._opt_best_trial}/{n_trials})")
                else:
                    self._opt_message = "Done. No successful trials."
                self._opt_current_params = {}

            print(f"[auto_opt] Optimization finished. Best: {self._opt_best_value}")

            # Generate report
            try:
                report_path = self._generate_report(cfg, param_names)
                self._opt_message += f" Report ready."
                print(f"[auto_opt] Report generated: {report_path}")
            except Exception as e:
                print(f"[auto_opt] Report generation failed: {e}")

    def _run_trial_replay(self, frames_t, frames_q, alignment_sec, settle_sec,
                          actual_data, rec_active, eval_start_out):
        """Replay trajectory and compute RMSE. Returns error or None if aborted."""
        first_q = frames_q[0]
        last_q = frames_q[-1]

        # Alignment — don't record for error
        t0 = time.monotonic()
        while time.monotonic() - t0 < alignment_sec:
            if self._opt_abort or not self.is_robot_ok:
                if not self.is_robot_ok:
                    print(f"[auto_opt] Robot fault during alignment (safety_mode={self._safety_mode})")
                    self._opt_abort = True
                return None
            self._set_cmd_target(list(first_q))
            time.sleep(0.02)

        # Clear actual_data collected during alignment
        eval_start = time.monotonic()
        eval_start_out[0] = eval_start
        actual_data.clear()

        # Evaluation — replay + record actual
        for i in range(len(frames_t)):
            if self._opt_abort or not self.is_robot_ok:
                if not self.is_robot_ok:
                    print(f"[auto_opt] Robot fault during evaluation (safety_mode={self._safety_mode})")
                    self._opt_abort = True
                return None
            target_rel = float(frames_t[i] - frames_t[0])
            now_rel = time.monotonic() - eval_start
            sleep_for = target_rel - now_rel
            if sleep_for > 0:
                time.sleep(min(sleep_for, 0.1))
            self._set_cmd_target(list(frames_q[i]))
            self._replay_progress = (i + 1) / len(frames_t)

        # Settle
        t1 = time.monotonic()
        while time.monotonic() - t1 < settle_sec:
            if self._opt_abort or not self.is_robot_ok:
                if not self.is_robot_ok:
                    print(f"[auto_opt] Robot fault during settle (safety_mode={self._safety_mode})")
                    self._opt_abort = True
                return None
            self._set_cmd_target(list(last_q))
            time.sleep(0.02)

        # Stop recording
        rec_active[0] = False

        # Compute RMSE
        return self._compute_rmse(frames_t, frames_q, actual_data, eval_start)

    def _compute_rmse(self, ref_t, ref_q, actual_data, eval_start):
        """Compute weighted joint RMSE between reference and actual trajectories."""
        if len(actual_data) < 10:
            print(f"[auto_opt] Too few actual samples: {len(actual_data)}")
            return 100.0

        actual_t = np.array([t - eval_start for t, _ in actual_data])
        actual_q = np.array([q for _, q in actual_data])
        ref_t_rel = ref_t - ref_t[0]

        # Interpolate reference to actual timestamps
        n_joints = ref_q.shape[1]
        ref_interp = np.zeros((len(actual_t), n_joints))
        for j in range(n_joints):
            ref_interp[:, j] = np.interp(actual_t, ref_t_rel, ref_q[:, j])

        # Weighted RMSE: shoulder joints weighted heavier
        weights = np.array([5.0, 5.0, 5.0, 1.0, 1.0, 1.0])
        diff = ref_interp - actual_q
        weighted_sq = (diff ** 2) * weights
        rmse = float(np.sqrt(np.mean(weighted_sq)))
        return rmse

    def _record_trial(self, trial_num, params, value, status,
                       ref_t, ref_q, actual_data, eval_start):
        entry = {
            "trial": trial_num,
            "params": dict(params),
            "value": round(value, 6) if value is not None else None,
            "status": status,
        }
        with self._lock:
            self._opt_history.append(entry)

        # Save per-trial data
        trial_dir = os.path.join(self._opt_exp_dir, "trial_data")
        os.makedirs(trial_dir, exist_ok=True)
        trial_path = os.path.join(trial_dir, f"trial_{trial_num:03d}.npz")
        save_dict = {
            "trial": trial_num,
            "params_keys": list(params.keys()),
            "params_values": [float(v) for v in params.values()],
            "rmse": float(value) if value is not None else -1.0,
            "status": status,
        }
        if ref_t is not None and actual_data is not None and eval_start is not None:
            actual_t_arr = np.array([t - eval_start for t, _ in actual_data])
            actual_q_arr = np.array([q for _, q in actual_data]) if actual_data else np.zeros((0, 6))
            ref_t_rel = ref_t - ref_t[0]
            save_dict["ref_t"] = ref_t_rel.astype(np.float64)
            save_dict["ref_q"] = ref_q.astype(np.float64)
            save_dict["actual_t"] = actual_t_arr.astype(np.float64)
            save_dict["actual_q"] = actual_q_arr.astype(np.float64)
        try:
            np.savez(trial_path, **save_dict)
        except Exception as e:
            print(f"[auto_opt] Failed to save trial data: {e}")

    def apply_best(self, param_client_fn):
        """Apply the best found params to the controller."""
        with self._lock:
            if not self._opt_best_params or not self._opt_config:
                raise RuntimeError("No best params to apply")
            controller = self._opt_config["controller"]
            best = dict(self._opt_best_params)

        client = param_client_fn(controller)
        current_vals = client.get_parameters(list(best.keys()))
        tuples = []
        for (k, new_val), cur_val in zip(best.items(), current_vals):
            if isinstance(cur_val, float) and isinstance(new_val, int):
                new_val = float(new_val)
            tuples.append((k, new_val))
        client.set_parameters(tuples)
        self._opt_message = f"Applied best params (RMSE {self._opt_best_value:.6f})"
        print(f"[auto_opt] Applied best params: {best}")
        return best

    # ------------------------------------------------------------------
    # Experiment management
    # ------------------------------------------------------------------
    def list_experiments(self):
        """List all experiment directories with summary info."""
        base = _param_opt_dir()
        result = []
        for name in sorted(os.listdir(base), reverse=True):
            exp_dir = os.path.join(base, name)
            if not os.path.isdir(exp_dir) or not name.startswith("exp_"):
                continue
            entry = {"name": name, "path": exp_dir}
            summary_path = os.path.join(exp_dir, "opt_summary.json")
            if os.path.isfile(summary_path):
                try:
                    with open(summary_path) as f:
                        s = json.load(f)
                    entry["controller"] = s.get("controller", "")
                    entry["n_trials"] = s.get("n_trials", 0)
                    entry["n_completed"] = s.get("n_completed", 0)
                    entry["best_rmse"] = s.get("best_rmse")
                    entry["timestamp"] = s.get("timestamp", "")
                    entry["param_names"] = s.get("param_names", [])
                except Exception:
                    entry["error"] = "unreadable summary"
            entry["has_report"] = os.path.isfile(os.path.join(exp_dir, "report.html"))
            result.append(entry)
        return result

    def delete_experiment(self, name: str):
        """Delete an experiment directory."""
        if not name.startswith("exp_"):
            raise ValueError("Invalid experiment name")
        exp_dir = os.path.join(_param_opt_dir(), name)
        if not os.path.isdir(exp_dir):
            raise FileNotFoundError(f"Experiment not found: {name}")
        shutil.rmtree(exp_dir)
        print(f"[auto_opt] Deleted experiment: {name}")

    def get_experiment_report_path(self, name: str) -> str:
        """Get path to an experiment's report.html."""
        exp_dir = os.path.join(_param_opt_dir(), name)
        report_path = os.path.join(exp_dir, "report.html")
        if not os.path.isfile(report_path):
            raise FileNotFoundError(f"No report for {name}")
        return report_path

    def _generate_report(self, cfg, param_names):
        """Generate a self-contained HTML report with optimization results."""
        history = list(self._opt_history)
        joint_names = self.robot.config.joint_names

        # Load per-trial trajectory data for the best trial
        best_trial_num = None
        best_rmse = None
        for h in history:
            if h["status"] == "ok" and h["value"] is not None:
                if best_rmse is None or h["value"] < best_rmse:
                    best_rmse = h["value"]
                    best_trial_num = h["trial"]

        # Collect trial data for charts
        trial_numbers = []
        trial_rmses = []
        trial_params = {pn: [] for pn in param_names}
        best_so_far = []
        running_best = None
        for h in history:
            if h["status"] != "ok" or h["value"] is None:
                continue
            trial_numbers.append(h["trial"])
            trial_rmses.append(h["value"])
            if running_best is None or h["value"] < running_best:
                running_best = h["value"]
            best_so_far.append(running_best)
            for pn in param_names:
                trial_params[pn].append(h["params"].get(pn))

        # Load best trial trajectory for joint tracking plot
        best_traj = None
        if best_trial_num is not None:
            trial_path = os.path.join(self._opt_exp_dir, "trial_data", f"trial_{best_trial_num:03d}.npz")
            if os.path.isfile(trial_path):
                try:
                    with np.load(trial_path, allow_pickle=True) as z:
                        best_traj = {
                            "ref_t": z["ref_t"].tolist() if "ref_t" in z.files else [],
                            "ref_q": z["ref_q"].tolist() if "ref_q" in z.files else [],
                            "actual_t": z["actual_t"].tolist() if "actual_t" in z.files else [],
                            "actual_q": z["actual_q"].tolist() if "actual_q" in z.files else [],
                        }
                except Exception:
                    pass

        # Build report data
        report_data = {
            "controller": cfg["controller"],
            "param_names": param_names,
            "param_bounds": {pn: {"min": cfg["params"][pn]["min"],
                                   "max": cfg["params"][pn]["max"],
                                   "ref": cfg["params"][pn]["ref"]}
                             for pn in param_names},
            "n_trials": cfg["n_trials"],
            "n_completed": len(trial_numbers),
            "best_trial": best_trial_num,
            "best_rmse": best_rmse,
            "best_params": dict(self._opt_best_params) if self._opt_best_params else {},
            "trial_numbers": trial_numbers,
            "trial_rmses": trial_rmses,
            "best_so_far": best_so_far,
            "trial_params": trial_params,
            "joint_names": joint_names,
            "best_traj": best_traj,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "history": history,
        }

        # Save JSON summary
        json_path = os.path.join(self._opt_exp_dir, "opt_summary.json")
        with open(json_path, "w") as f:
            json.dump(report_data, f, indent=2)

        # Generate HTML
        report_path = os.path.join(self._opt_exp_dir, "report.html")
        html = self._build_report_html(report_data)
        with open(report_path, "w") as f:
            f.write(html)
        return report_path

    def _build_report_html(self, data):
        """Build self-contained HTML report with Chart.js."""
        d = json.dumps(data)
        best_rmse_str = f"{data['best_rmse']:.6f}" if data["best_rmse"] else "—"
        best_trial_str = str(data["best_trial"]) if data["best_trial"] else "—"

        # Build param summary rows
        param_rows = ""
        for pn in data["param_names"]:
            bv = data["best_params"].get(pn)
            bv_str = f"{bv:.4f}" if isinstance(bv, float) else "—"
            bd = data["param_bounds"][pn]
            param_rows += f'    <tr><td>{pn}</td><td>ref: {bd["ref"]} | best: <span class="param-val">{bv_str}</span> | range: [{bd["min"]}, {bd["max"]}]</td></tr>\n'

        # Build param chart divs
        param_charts = ""
        for i, pn in enumerate(data["param_names"]):
            param_charts += f'  <div class="card"><h2>{pn}</h2><canvas id="chartParam{i}"></canvas></div>\n'

        # Build history header
        hist_header = "".join(f"<th>{pn}</th>" for pn in data["param_names"])

        return f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Optimization Report — {data["controller"]}</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  body {{ font-family: system-ui, sans-serif; background: #1a1a2e; color: #eee; margin: 0; padding: 20px; }}
  h1 {{ color: #e94560; }} h2 {{ color: #888; font-size: 1.1em; text-transform: uppercase; margin-top: 30px; }}
  .card {{ background: #16213e; border-radius: 8px; padding: 16px; margin: 12px 0; }}
  table {{ border-collapse: collapse; width: 100%; font-family: monospace; font-size: 0.9em; }}
  th {{ text-align: left; color: #888; padding: 6px 10px; border-bottom: 1px solid #0f3460; }}
  td {{ padding: 5px 10px; border-bottom: 1px solid rgba(255,255,255,0.05); }}
  .best {{ color: #4caf50; font-weight: bold; }}
  .fail {{ color: #e94560; }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
  canvas {{ background: #0a0f1a; border-radius: 6px; }}
  .meta {{ color: #888; font-size: 0.85em; }}
  .param-val {{ color: #2196f3; }}
</style>
</head>
<body>
<h1>Optimization Report</h1>
<div class="meta">Controller: <b>{data["controller"]}</b> | Trials: {data["n_completed"]}/{data["n_trials"]} | Generated: {data["timestamp"]}</div>

<div class="card">
  <h2>Summary</h2>
  <table>
    <tr><td>Best RMSE</td><td class="best">{best_rmse_str}</td></tr>
    <tr><td>Best Trial</td><td>{best_trial_str}</td></tr>
{param_rows}  </table>
</div>

<div class="grid">
  <div class="card"><h2>RMSE per Trial</h2><canvas id="chartRmse"></canvas></div>
  <div class="card"><h2>Best RMSE So Far</h2><canvas id="chartBest"></canvas></div>
</div>

<div class="grid">
{param_charts}</div>

<div class="card" id="trajSection" style="display:none;">
  <h2>Joint Tracking — Best Trial</h2>
  <div class="grid" id="trajCharts"></div>
</div>

<div class="card">
  <h2>Trial History</h2>
  <div style="max-height:400px;overflow-y:auto;">
    <table>
      <thead><tr><th>#</th><th>RMSE</th><th>Status</th>{hist_header}</tr></thead>
      <tbody id="histBody"></tbody>
    </table>
  </div>
</div>

<script>
const D = ''' + d + ''';
const colors = ['#e94560','#4caf50','#2196f3','#ff9800','#9c27b0','#00bcd4','#f44336','#8bc34a'];

// RMSE chart
new Chart('chartRmse', {type:'scatter', data:{datasets:[{label:'RMSE', data:D.trial_numbers.map((n,i)=>({x:n,y:D.trial_rmses[i]})), backgroundColor:'#2196f3', pointRadius:4}]}, options:{scales:{x:{title:{display:true,text:'Trial',color:'#888'},ticks:{color:'#888'}},y:{title:{display:true,text:'RMSE',color:'#888'},ticks:{color:'#888'}}},plugins:{legend:{display:false}}}});

// Best so far
new Chart('chartBest', {type:'line', data:{labels:D.trial_numbers, datasets:[{label:'Best RMSE', data:D.best_so_far, borderColor:'#4caf50', fill:false, tension:0}]}, options:{scales:{x:{title:{display:true,text:'Trial',color:'#888'},ticks:{color:'#888'}},y:{title:{display:true,text:'RMSE',color:'#888'},ticks:{color:'#888'}}},plugins:{legend:{display:false}}}});

// Parameter charts
D.param_names.forEach((pn, i) => {
  const vals = D.trial_params[pn];
  const bd = D.param_bounds[pn];
  new Chart('chartParam'+i, {type:'scatter',
    data:{datasets:[
      {label:pn, data:D.trial_numbers.map((n,j)=>({x:n,y:vals[j]})), backgroundColor:colors[i%colors.length], pointRadius:4},
    ]},
    options:{scales:{x:{title:{display:true,text:'Trial',color:'#888'},ticks:{color:'#888'}},y:{min:bd.min,max:bd.max,title:{display:true,text:pn,color:'#888'},ticks:{color:'#888'}}},
      plugins:{legend:{display:false},
        annotation:{annotations:{ref:{type:'line',yMin:bd.ref,yMax:bd.ref,borderColor:'#ff9800',borderDash:[5,5],label:{content:'ref',display:true}}}}
      }}});
});

// Joint tracking for best trial
if (D.best_traj && D.best_traj.ref_t && D.best_traj.ref_t.length > 0) {
  document.getElementById('trajSection').style.display = '';
  const grid = document.getElementById('trajCharts');
  const jnames = D.joint_names;
  const ds = 10; // downsample for chart performance
  for (let j = 0; j < Math.min(jnames.length, 6); j++) {
    const div = document.createElement('div');
    div.className = 'card';
    div.innerHTML = '<canvas id="trajJ'+j+'"></canvas>';
    grid.appendChild(div);
    const ref_t = D.best_traj.ref_t.filter((_,i)=>i%ds===0);
    const ref_q = D.best_traj.ref_q.filter((_,i)=>i%ds===0).map(q=>q[j]*180/Math.PI);
    const act_t = D.best_traj.actual_t.filter((_,i)=>i%ds===0);
    const act_q = D.best_traj.actual_q.filter((_,i)=>i%ds===0).map(q=>q[j]*180/Math.PI);
    new Chart('trajJ'+j, {type:'line',
      data:{datasets:[
        {label:'Reference', data:ref_t.map((t,i)=>({x:t,y:ref_q[i]})), borderColor:'#4caf50', pointRadius:0, borderWidth:1.5},
        {label:'Actual', data:act_t.map((t,i)=>({x:t,y:act_q[i]})), borderColor:'#2196f3', pointRadius:0, borderWidth:1.5},
      ]},
      options:{scales:{x:{type:'linear',title:{display:true,text:'Time (s)',color:'#888'},ticks:{color:'#888'}},y:{title:{display:true,text:jnames[j]+' (deg)',color:'#888'},ticks:{color:'#888'}}},
        plugins:{legend:{labels:{color:'#ccc'}}}}});
  }
}

// History table
const tb = document.getElementById('histBody');
D.history.forEach(h => {
  const cls = h.status==='ok'?(h.trial===D.best_trial?'best':''):'fail';
  let row = '<tr class="'+cls+'"><td>'+h.trial+'</td><td>'+(h.value!==null?h.value.toFixed(6):'—')+'</td><td>'+h.status+'</td>';
  D.param_names.forEach(pn => { row += '<td>'+(h.params[pn]!==undefined?h.params[pn].toFixed(4):'—')+'</td>'; });
  row += '</tr>';
  tb.innerHTML += row;
});
</script>
</body>
</html>'''

"""Auto optimization framework for controller parameters.

Records a reference trajectory via Alicia teleop, replays it with different
controller parameters suggested by Optuna, measures joint tracking RMSE,
and finds optimal parameters.

Storage: tmp/param_opt/
"""

import os
import threading
import time

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

    def start_optimization(self, config: dict, param_client_fn):
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
        db_path = os.path.join(_param_opt_dir(), "optuna.db")
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

        trial_counter = [0]  # local counter for this run

        print(f"[auto_opt] Optimization START: {controller}, {n_trials} trials, "
              f"{len(param_names)} params: {param_names}")

        def objective(trial):
            if self._opt_abort:
                raise optuna.TrialPruned()

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
                self._record_trial(trial.number, suggested, None, "param_fail")
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

            try:
                error = self._run_trial_replay(
                    frames_t, frames_q, alignment_sec, settle_sec,
                    actual_data, rec_active,
                )
            finally:
                rec_active[0] = False

            if error is None:
                # Aborted
                self._record_trial(trial.number, suggested, None, "aborted")
                raise optuna.TrialPruned()

            # Record success
            self._opt_consecutive_failures = 0
            self._record_trial(trial.number, suggested, error, "ok")

            if self._opt_best_value is None or error < self._opt_best_value:
                self._opt_best_value = round(error, 6)
                self._opt_best_params = dict(suggested)
                self._opt_best_trial = trial.number + 1
                self._opt_message = (f"Trial {self._opt_trial_num}/{n_trials} — "
                                     f"NEW BEST: {error:.6f}")
            else:
                self._opt_message = (f"Trial {self._opt_trial_num}/{n_trials} — "
                                     f"RMSE: {error:.6f} (best: {self._opt_best_value:.6f})")

            # Inter-trial pause
            time.sleep(1.0)
            return error

        def stop_callback(study, trial):
            if self._opt_abort:
                study.stop()

        try:
            self._opt_study.optimize(
                objective,
                n_trials=n_trials,
                callbacks=[stop_callback],
            )
        except Exception as e:
            if "Trial" not in str(e):
                print(f"[auto_opt] Optimization error: {e}")

        # Clean up persistent subscription
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
            print(f"[auto_opt] Restored ref params")
        except Exception as e:
            print(f"[auto_opt] Failed to restore ref params: {e}")

        with self._lock:
            self._optimizing = False
            if self._opt_best_value is not None:
                self._opt_message = (f"Done. Best RMSE: {self._opt_best_value:.6f} "
                                     f"(trial {self._opt_best_trial}/{n_trials})")
            else:
                self._opt_message = "Done. No successful trials."
            self._opt_current_params = {}

        print(f"[auto_opt] Optimization complete. Best: {self._opt_best_value}")

    def _run_trial_replay(self, frames_t, frames_q, alignment_sec, settle_sec,
                          actual_data, rec_active):
        """Replay trajectory and compute RMSE. Returns error or None if aborted."""
        first_q = frames_q[0]
        last_q = frames_q[-1]

        # Alignment — don't record for error
        t0 = time.monotonic()
        while time.monotonic() - t0 < alignment_sec:
            if self._opt_abort:
                return None
            self._set_cmd_target(list(first_q))
            time.sleep(0.02)

        # Clear actual_data collected during alignment
        eval_start = time.monotonic()
        actual_data.clear()

        # Evaluation — replay + record actual
        for i in range(len(frames_t)):
            if self._opt_abort:
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
            if self._opt_abort:
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

    def _record_trial(self, trial_num, params, value, status):
        entry = {
            "trial": trial_num + 1,
            "params": dict(params),
            "value": round(value, 6) if value is not None else None,
            "status": status,
        }
        with self._lock:
            self._opt_history.append(entry)

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

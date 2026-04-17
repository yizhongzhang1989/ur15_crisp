# Auto Optimization Framework — Design Plan (v3)

## Overview

Integrate Optuna-based automatic controller parameter optimization into `web_control`.
Records a reference trajectory via Alicia teleop, replays it with different parameters,
measures tracking error, and finds optimal parameters — all from the web dashboard.

## Current Status

Phase 1 (trajectory recording + replay) is **complete and tested**. Phase 2 (Optuna
optimization loop) is the remaining work.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  Web Dashboard (:8080)                                           │
│  ┌──────────────────────────┬───────────────────────────────┐   │
│  │ Controller Parameters    │ Auto Optimization              │   │
│  │ - Select controller      │ - [Start/Stop Recording]       │   │
│  │ - View/edit values       │ - [Save] [Replay] [Delete]     │   │
│  │ - Save / Reset           │ - Status / saved trajectory    │   │
│  │                          │ - (Phase 2: optimization UI)   │   │
│  └──────────────────────────┴───────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
        │ /api/params/*              │ /api/opt/*
        ▼                            ▼
┌──────────────────────────────────────────────────────────────────┐
│  web_server.py + auto_opt.py (OptimizationManager)               │
└──────────────────────────────────────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────────────────────────────────────┐
│  Disk: tmp/param_opt/                                            │
│    reference_trajectory.npz                                      │
│    (Phase 2: optuna.db, config.yaml, trial_data/)                │
└──────────────────────────────────────────────────────────────────┘
```

## Control Data Flow (Implemented)

All joint target sources funnel through a single pipeline:

```
Sources → _set_cmd_target(joints) → _cmd_target_raw → timer 250Hz → publish everything
```

| Source | Trigger |
|--------|---------|
| Alicia teleop | `/arm_joint_state` callback → `_set_cmd_target()` |
| Joint sliders | `POST /api/set_target_joint` → `_set_cmd_target()` |
| Replay | replay thread → `_set_cmd_target()` |
| move_joint_delta | `POST /api/move_joint_delta` → `_set_cmd_target()` |

The 250 Hz timer (`_cmd_pub_timer_cb`) always:
1. Publishes `_cmd_target_raw` → `/cmd_target_joint`
2. FK → publishes `/cmd_target_pose`
3. `robot.set_target(pose=...)` → feeds cartesian impedance controller
4. `robot.set_target_joint(q)` → feeds joint impedance controller
5. Trapezoidal profile → `/forward_position_controller/commands`

**Replay guard**: During replay, `_alicia_cb` checks `opt_mgr.is_replaying` and
suppresses teleop writes. Teleop flag is preserved and resumes automatically
when replay finishes.

## OptimizationManager (Implemented)

**File**: `src/web_control/web_control/auto_opt.py`

**Dependencies injected from WebControlServer**:
- `robot` — crisp_py Robot
- `node` — ROS node for subscriptions
- `set_cmd_target_fn` — callable `_set_cmd_target(joints_list)`
- `owner` — WebControlServer instance (for `is_replaying` guard)

**State** (thread-safe via `threading.Lock`):
- Recording: `_recording`, `_rec_start_wall`, `_rec_duration`, `_rec_cmd_joint`, `_rec_cmd_pose`, `_rec_actual`
- Replay: `_replaying`, `_replay_progress`, `_replay_abort`, `_replay_thread`

**Methods**:

| Method | Description |
|--------|-------------|
| `status()` | Returns recording/replay state, frame counts, saved metadata |
| `start_recording()` | Creates ROS subscriptions, starts buffering |
| `stop_recording()` | Destroys subscriptions, buffer stays in memory |
| `save()` | Writes buffer to `tmp/param_opt/reference_trajectory.npz` |
| `delete()` | Removes file + clears buffer |
| `start_replay(alignment, settle)` | Loads from disk, starts replay thread |
| `abort_replay()` | Sets abort flag, thread exits cleanly |
| `is_replaying` (property) | Thread-safe check for teleop guard |

## API Endpoints (Implemented)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/opt/status` | Recording/replay state, saved trajectory metadata |
| POST | `/api/opt/record` | Start recording (no params needed) |
| POST | `/api/opt/stop_record` | Stop recording (buffer in memory) |
| POST | `/api/opt/save` | Save buffer to disk |
| POST | `/api/opt/delete` | Delete file + clear buffer |
| POST | `/api/opt/replay` | Start replay (optional: `alignment_time_sec`, `settle_time_sec`) |
| POST | `/api/opt/stop_replay` | Abort running replay |

## Trajectory Recording (Implemented)

- User-controlled duration: click Start Recording → robot moves via Alicia teleop → click Stop Recording
- Creates temporary ROS subscriptions for `/cmd_target_joint`, `/cmd_target_pose`, `/joint_states`
- Stores with `time.monotonic()` relative timestamps
- Buffer stays in memory after stop; explicit Save writes to disk
- Format: `reference_trajectory.npz` with arrays:
  - `cmd_joint_t` (N,), `cmd_joint_q` (N,6) — joint targets at ~250 Hz
  - `cmd_pose_t` (M,), `cmd_pose_v` (M,7) — pose targets [x,y,z,qx,qy,qz,qw]
  - `actual_t` (K,), `actual_q` (K,6) — actual joint states at ~500 Hz
  - `duration` — total recording time in seconds
  - `recorded_at` — human-readable timestamp

## Trajectory Replay (Implemented)

Joint-only replay. Cartesian targets are computed automatically by the 250 Hz FK timer.

| Phase | Duration | Action |
|-------|----------|--------|
| Alignment | 2.0s (configurable) | Send first frame continuously via `_set_cmd_target()` |
| Evaluation | trajectory duration | Replay `cmd_joint_q` frames at original timing |
| Settle | 0.5s (configurable) | Hold last frame |

- Uses `time.monotonic()` for timing; sleeps to match original timestamps
- Replay thread writes to `_set_cmd_target()` → same pipeline as teleop
- Abortable at any point via `abort_replay()`

## Frontend UI (Implemented)

Controller Parameters and Auto Optimization panels are side-by-side in a 2-column grid.

Auto Optimization panel has:
- **Start/Stop Recording** toggle button (single button, state-driven label)
- **Save** button — writes buffer to disk
- **Replay** toggle button — starts replay / stops replay
- **Delete** button — removes file + buffer (with confirm dialog)
- Live status line (polled 500ms): `● REC 5.2s cmd_joint=1300` or `▶ REPLAY 45.2%` or `◆ BUFFER unsaved`
- Saved trajectory metadata: duration, frame counts, timestamp

---

## Phase 2: Optuna Optimization Loop (TODO)

### Objective Function (one trial)

1. Get suggested params from Optuna (TPE sampler)
2. Set params via `ParametersClient`, wait `settle_time_sec`
3. **Alignment** (2s): send first frame continuously, error NOT measured
4. **Evaluation**: replay `cmd_joint` via `_set_cmd_target()` at original timestamps; record actual `/joint_states` at full rate
5. **Settle** (0.5s): hold last target
6. Compute tracking error (RMSE, skip alignment frames)
7. Save per-trial data to `trial_data/trial_NNN.npz`
8. Return error to Optuna
9. On failure: return penalty (100.0), increment `consecutive_failures`

### New Dependencies Needed

- `param_client_fn` — function to get ParametersClient for a controller name (already available via `_get_param_client()` in web_server)
- Record actual `/joint_states` during replay for error computation (subscribe in thread start, destroy in finally)

### Error Metric

```python
def compute_rmse(cmd_joint_q, actual_q, alignment_sec):
    # Skip frames within alignment_sec
    weights = [5, 5, 5, 1, 1, 1]  # shoulder joints weighted heavier
    errors = [sum(w*(r-a)**2 for w,r,a in zip(weights,ref_q,act_q))
              for ref_q, act_q in aligned_pairs]
    return float(np.sqrt(np.mean(errors)))  # RMSE in rad
```

### New API Endpoints (Phase 2)

| Method | Path | Description |
|--------|------|-------------|
| POST | `/api/opt/config` | Set controller, param bounds, n_trials |
| POST | `/api/opt/start` | Start optimization loop in background |
| POST | `/api/opt/stop` | Stop after current trial |
| POST | `/api/opt/apply_best` | Apply best params to controller + save config |
| GET | `/api/opt/history` | Trial history (params, values, status) |

### POST /api/opt/config

```json
{
  "controller": "joint_impedance_controller",
  "params": {
    "nullspace.stiffness": {"min": 100, "max": 400},
    "nullspace.damping": {"min": 15, "max": 60}
  },
  "n_trials": 50,
  "alignment_time_sec": 2.0,
  "settle_time_sec": 0.5,
  "max_consecutive_failures": 3
}
```

### Safety & Auto Recovery

**Before each trial**:
| Robot State | Action |
|-------------|--------|
| RUNNING | Proceed |
| PROTECTIVE_STOP | Auto recover: power_on → wait 5s → brake_release → wait 10s → re-activate controller → wait 2s → proceed |
| EMERGENCY_STOP | Pause optimization, alert UI |
| OTHER | Wait 5s, retry 3×, then pause |

**After failure**:
1. Return penalty 100.0 to Optuna
2. Auto recover
3. Increment `consecutive_failures`; if ≥ max → pause + alert
4. On success → reset `consecutive_failures = 0`

**Between trials**: 1s pause for system to settle.

### Persistence

**Storage**: `tmp/param_opt/`
```
reference_trajectory.npz   # recorded trajectory (already implemented)
config.yaml                # optimization config
optuna.db                  # Optuna SQLite (all trials)
trial_data/
  trial_000.npz            # actual joint data per trial
  trial_001.npz
```

| Data | Written | Size |
|------|---------|------|
| reference_trajectory.npz | On Save | ~1-5 MB |
| config.yaml | On /api/opt/config | ~1 KB |
| optuna.db | After each trial | ~100 KB |
| trial_NNN.npz | After each trial | ~200 KB |

### Frontend UI Additions (Phase 2)

Add to the existing Auto Optimization panel:
- Controller dropdown + param checkboxes with min/max bounds
- Trials count input
- Start/Stop optimization buttons
- Progress bar, best result, current trial info
- History table (scrollable)
- Apply Best button

### Implementation Steps (Phase 2)

| Step | Description | Effort |
|------|-------------|--------|
| 1 | `pip3 install optuna` | 1 min |
| 2 | Add optimization loop to `auto_opt.py` — subscribe to `/joint_states` during replay, compute RMSE, Optuna study management, persistence | 2-3 hrs |
| 3 | Add `/api/opt/config`, `/api/opt/start`, `/api/opt/stop`, `/api/opt/apply_best`, `/api/opt/history` routes | 1 hr |
| 4 | Frontend: param checkboxes, bounds inputs, progress bar, history table, apply best | 2 hrs |
| 5 | Safety: auto recovery from protective stops, consecutive failure tracking | 1 hr |
| 6 | Testing: 5-trial test, abort/resume, recovery test | 1 hr |
| **Total** | | **~7-8 hrs** |

### File Changes (Phase 2)

| File | Change |
|------|--------|
| `web_control/auto_opt.py` | Add optimization loop, RMSE, persistence (~200 lines) |
| `web_control/web_server.py` | Add 5 new routes, pass param_client_fn (~60 lines) |
| `web_control/static/index.html` | Optimization config UI, progress, history (~150 HTML + ~100 JS) |

## Open Questions / Future Work

1. **Multi-objective**: tracking error vs smoothness vs energy. Worth adding?
2. **Per-joint weights**: expose joint weights as a config option?
3. **Trajectory diversity**: optimize across multiple trajectories?
4. **Sim pre-screening**: if MuJoCo becomes available, warm-start from sim?
5. **Optuna Dashboard**: `pip install optuna-dashboard` for study inspection?
# Auto Optimization Framework — Design Plan (v2)

## Overview

Integrate Optuna-based automatic controller parameter optimization into `web_control`.
Records a reference trajectory, replays it with different parameters, measures tracking
error, and finds optimal parameters — all from the web dashboard. Fully persistent:
survives shutdown, auto-recovers from protective stops.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  Web Dashboard (:8080) — Controller Parameters (split L/R)       │
│  ┌─────────────────────┬────────────────────────────────────┐   │
│  │ LEFT: Manual Edit   │ RIGHT: Auto Optimization           │   │
│  │ - Select controller │ - Select params + bounds            │   │
│  │ - View/edit values  │ - Record reference trajectory       │   │
│  │ - Set / Save        │ - Start/Stop/Resume optimization    │   │
│  │                     │ - Progress, best result, history    │   │
│  │                     │ - Apply best / Previous studies     │   │
│  └─────────────────────┴────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
        │ /api/params/*              │ /api/opt/*
        ▼                            ▼
┌──────────────────────────────────────────────────────────────────┐
│  web_server.py + auto_opt.py (OptimizationManager)               │
└──────────────────────────────────────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────────────────────────────────────┐
│  Disk: config/opt_studies/<study_name>/                           │
│    config.yaml, reference_trajectory.npz, optuna.db, trial_data/ │
└──────────────────────────────────────────────────────────────────┘
```

## OptimizationManager

**File**: `src/web_control/web_control/auto_opt.py`

**Dependencies injected from WebControlServer**:
- `robot` — crisp_py Robot (set_target_joint, set_target, joint_values, end_effector_pose)
- `param_client_fn` — function to get ParametersClient for a controller name
- `_set_cmd_target` — function to publish /cmd_target_joint
- `node` — ROS node for subscriptions
- `controller_switcher` — for re-activating controllers after recovery

**State** (thread-safe via Lock):
- `reference` — recorded trajectory dict
- `opt_config` — controller, mode, param bounds, n_trials, etc.
- `study` — optuna.Study (SQLite-backed)
- `running`, `paused`, `paused_reason`
- `current_trial`, `best_params`, `best_value`, `best_trial`
- `consecutive_failures`
- `study_dir` — path to persistence directory
- `history` — list of trial dicts

**Objective function** (one trial):
1. Check robot state → auto-recover if protective stop
2. Get suggested params from Optuna
3. Set params via ParametersClient, wait `settle_time_sec`
4. **Alignment phase** (`alignment_time_sec`, default 2s):
   - Send first frame of trajectory as target continuously
   - Robot moves to start position via existing trapezoidal/impedance path
   - Error NOT measured
5. **Evaluation phase** (remaining trajectory duration):
   - Joint mode → replay `cmd_joint` via `_set_cmd_target()` at original timestamps
   - Cartesian mode → replay `cmd_pose` via `robot.set_target(pose=...)` at original timestamps
   - Record actual `/joint_states` at full rate
6. **Settle phase** (0.5s): hold last target
7. Compute tracking error (RMSE, skip alignment frames)
8. Save per-trial data to `trial_data/trial_NNN.npz`
9. Return error to Optuna
10. On failure: return penalty (100.0), increment `consecutive_failures`

## API Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/opt/status` | Running state, progress, best result, failures |
| POST | `/api/opt/config` | Set controller, mode, param bounds, n_trials |
| POST | `/api/opt/record` | Record reference trajectory for N seconds |
| POST | `/api/opt/start` | Start optimization in background thread |
| POST | `/api/opt/stop` | Stop after current trial completes |
| POST | `/api/opt/apply_best` | Apply best params to controller + save config |
| GET | `/api/opt/history` | Full trial history (params, values, status) |
| GET | `/api/opt/studies` | List all persisted studies on disk |
| POST | `/api/opt/resume` | Resume interrupted study by name |
| DELETE | `/api/opt/studies/<name>` | Delete study and all data |

### POST /api/opt/config
```json
{
  "controller": "joint_impedance_controller",
  "mode": "joint",
  "params": {
    "nullspace.stiffness": {"min": 100, "max": 400, "optimize": true},
    "nullspace.damping": {"min": 15, "max": 60, "optimize": true},
    "filter.q": {"min": 0.1, "max": 1.0, "optimize": false}
  },
  "n_trials": 50,
  "alignment_time_sec": 2.0,
  "settle_time_sec": 0.5,
  "max_consecutive_failures": 3
}
```

**`mode`**:
- `"joint"` → replay `/cmd_target_joint`, joint RMSE, optimize joint impedance params
- `"cartesian"` → replay `/cmd_target_pose`, Cartesian RMSE, optimize task stiffness/damping

**Validation**: min < max, min ≥ 0 for gains, controller loaded, ≥1 param optimized

### POST /api/opt/record
```json
{"duration_sec": 5.0}
```
Records at full publish rate:
- `/cmd_target_joint` (JointState) — joint targets
- `/cmd_target_pose` (PoseStamped) — Cartesian targets
- `/joint_states` (JointState) — actual robot state

**Preconditions**: robot moving (teleop active), controller active, target topics publishing.
Saved immediately to `reference_trajectory.npz`.

## Trajectory Recording

- Create temporary ROS subscriptions for the recording duration
- Store with ROS header timestamps (not wall clock) for deterministic replay
- Format: `{cmd_joint: [(stamp, [6 floats]),...], cmd_pose: [(stamp, {x,y,z,qx,qy,qz,qw}),...], actual: [(stamp, [6 floats]),...]}`
- Verify: ≥1s data, both cmd topics have frames

## Trajectory Replay

| Phase | Duration | Action | Record Error |
|-------|----------|--------|-------------|
| Alignment | `alignment_time_sec` (2s) | Send first frame continuously | No |
| Evaluation | trajectory duration − alignment | Replay frames at original timing | Yes |
| Settle | 0.5s | Hold last frame | No |

- Use `time.monotonic()` for replay timing
- Monitor `/joint_states` rate during replay — if 0 for >1s → fault

## Error Metric

```python
def compute_error(reference, actual, mode, alignment_sec):
    # Skip frames within alignment_sec
    if mode == "joint":
        weights = [5, 5, 5, 1, 1, 1]  # shoulder heavier
        errors = [sum(w*(r-a)**2 for w,r,a in zip(weights,ref_q,act_q))
                  for ref_q, act_q in aligned_pairs]
        return float(np.sqrt(np.mean(errors)))  # RMSE in rad
    elif mode == "cartesian":
        pos_err = [np.linalg.norm(rp-ap) for rp,ap in pos_pairs]
        ori_err = [quat_angular_dist(rq,aq) for rq,aq in ori_pairs]
        return float(np.sqrt(np.mean(np.array(pos_err)**2))
                     + 0.01 * np.sqrt(np.mean(np.array(ori_err)**2)))
```
RMSE so units are interpretable (rad for joint, m for Cartesian).

## Safety & Auto Recovery

**Before each trial**:
| Robot State | Action |
|-------------|--------|
| RUNNING | Proceed |
| PROTECTIVE_STOP | Auto recover: power_on → wait 5s → brake_release → wait 10s → re-activate controller → wait 2s → proceed |
| EMERGENCY_STOP | Pause optimization, set `paused_reason="emergency_stop"`, alert UI |
| OTHER | Wait 5s, retry 3×, then pause |

**During replay**: monitor `/joint_states` publish rate. If 0 for >1s → abort trial as failed.

**After failure**:
1. Record trial as `status: "failed"`, return penalty 100.0 to Optuna
2. Auto recover
3. Increment `consecutive_failures`
4. If `consecutive_failures ≥ max_consecutive_failures` → pause + alert
5. On successful trial → reset `consecutive_failures = 0`

**Between trials**: 1s pause for system to settle.

## Full Persistence & Resume

**Storage**: `config/opt_studies/<study_name>/`
```
config.yaml                # optimization config
reference_trajectory.npz   # recorded trajectory
optuna.db                  # Optuna SQLite (all trials)
trial_data/
  trial_000.npz            # actual data per trial
  trial_001.npz
```

| Data | Written | Size |
|------|---------|------|
| config.yaml | On /api/opt/config | ~1 KB |
| reference_trajectory.npz | On /api/opt/record | ~1-5 MB |
| optuna.db | After each trial (automatic) | ~100 KB |
| trial_NNN.npz | After each trial | ~200 KB |

**Resume**: On startup, scan `config/opt_studies/`. User selects study →
load config + trajectory + `optuna.load_study()` → `study.optimize(n_trials=remaining)`.

## Frontend UI (Right Panel)

```
┌─────────────────────────────────────────┐
│ AUTO OPTIMIZATION                       │
│ Controller: [joint_impedance_ctrl ▼]    │
│ Mode: [joint ▼]                         │
│                                         │
│ Parameters to Optimize:                 │
│ ☑ nullspace.stiffness  [100] — [400]   │
│ ☑ nullspace.damping    [15]  — [60]    │
│ ☐ filter.q             [0.1] — [1.0]   │
│ ☑ max_delta_tau        [1.0] — [5.0]   │
│                                         │
│ Trials: [50]  Alignment: [2.0]s         │
│                                         │
│ Reference: [Record 5s] [Record 10s]     │
│ Status: 500 frames, 5.0s ✓             │
│                                         │
│ [Start] [Stop] [Apply Best]            │
│                                         │
│ Progress: 15/50  ████████░░░ 30%        │
│ Best: 0.021 (trial 12)                 │
│ Current: 0.025  Failures: 0/3          │
│                                         │
│ Studies:                                │
│ joint_imp_04-16 50/50 ✓ [Load] [Del]  │
│ cart_imp_04-15  23/50 ⚠ [Resume] [Del] │
│                                         │
│ History (scroll):                       │
│ # | error | stiffness | damping         │
│ 12| 0.021 | 295       | 22          ★  │
│ 15| 0.025 | 310       | 18             │
│  2| FAIL  | 487       | 8           ✗  │
└─────────────────────────────────────────┘
```

## Implementation Steps

| Step | Description | Effort |
|------|-------------|--------|
| 1 | `pip3 install optuna` | 1 min |
| 2 | Create `auto_opt.py` — OptimizationManager class (record, replay, objective, recovery, persistence) | 3-4 hrs |
| 3 | Add `/api/opt/*` routes to `web_server.py`, init OptimizationManager, add opt status to SSE | 1 hr |
| 4 | Frontend: split params panel L/R, build optimization UI with dynamic param checkboxes, progress, studies list, history table | 2-3 hrs |
| 5 | Testing: mock test, 5-trial real test, abort/resume test, recovery test | 1-2 hrs |
| **Total** | | **~8-10 hrs** |

## File Changes

| File | Change |
|------|--------|
| `web_control/auto_opt.py` | **New** — OptimizationManager (~400 lines) |
| `web_control/web_server.py` | Import + init OptimizationManager, add /api/opt/* routes (~100 lines) |
| `web_control/static/index.html` | Split params panel, add optimization UI (~200 HTML + ~150 JS lines) |
| `web_control/package.xml` | Add comment noting optuna pip dependency |

## Dependencies

- **New**: `optuna` (`pip3 install optuna`)
- **Existing**: `crisp_py`, `common`, `flask`, `numpy`, `scipy`, `yaml`

## Open Questions / Future Work

1. **Multi-objective**: Optuna supports multi-objective (tracking error vs smoothness vs energy). Worth adding later?
2. **Per-joint optimization**: Different joints may need different weight emphasis. Expose joint weights as a config option?
3. **Trajectory diversity**: Optimize across multiple trajectories to avoid overfitting to one motion pattern?
4. **Sim pre-screening**: If MuJoCo becomes available, run 100 cheap sim trials first, warm-start real robot from sim-optimal?
5. **Optuna Dashboard**: `pip install optuna-dashboard` provides a web UI for study inspection. Integrate or keep separate?

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

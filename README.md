# Module cup-mover

Drives an arm to move a cup through a configurable sequence of waypoints, optionally interacting with a gripper at each step. Waypoints can be saved as Cartesian **poses** (planned via the Motion service) or as raw **joint positions** (commanded directly on the arm), selected per switch via a `mode` attribute.

The module ships three models:

| Model                                                | API                  | Purpose                                                            |
|------------------------------------------------------|----------------------|--------------------------------------------------------------------|
| `viam-labs:cup-mover:cup-mover`                      | `rdk:service:generic`| Orchestrator. Walks a configured step list and triggers gripper actions. Mode-agnostic — it references switch waypoints by name. |
| `viam-labs:cup-mover:multi-poses-execution-switch`   | `rdk:component:switch`| Stores N named waypoints; `SetPosition(i)` moves to waypoint `i`. In `pose` mode it plans to an absolute pose via the Motion service; in `joint` mode it commands the arm's joints directly. |
| `viam-labs:cup-mover:dial-control-motion`            | `rdk:service:generic`| Interactive arm jog + pose/joint capture, suitable for tuning waypoints before saving them. |

Typical wiring: dial-control to capture waypoints (`get_pose` or `get_joints`) → paste them into the switch → orchestrator runs the sequence.

## Model viam-labs:cup-mover:cup-mover

A `rdk:service:generic` that walks an ordered list of `{ waypoint, grip }` steps. Each step moves to the named switch waypoint (a pose or a joint position, depending on the switch's mode), then optionally triggers a gripper action. If a gripper is configured, it's also opened once at the very start of every run so the arm approaches the first waypoint with an open gripper.

The orchestrator is mode-agnostic: it only references waypoints by name, so the same step list works against a pose-mode or joint-mode switch.

### Configuration

```jsonc
{
  "pose_switch_name": "cup-pose-switch",
  "gripper_name": "gripper",
  "steps": [
    { "waypoint": "home" },
    { "waypoint": "pickup-appr" },
    { "waypoint": "pickup",      "grip": "grab" },
    { "waypoint": "stand1",      "grip": "open_grab" },
    { "waypoint": "stand2",      "grip": "open_grab" },
    { "waypoint": "dump-base" },
    { "waypoint": "dump" },
    { "waypoint": "pickup",      "grip": "open" },
    { "waypoint": "pickup-appr" },
    { "waypoint": "home" }
  ],
  "pause_secs": 1.0
}
```

The trailing `pickup-appr → home` retracts the gripper through the approach waypoint and parks it, leaving the arm in a known state for the next run.

> The legacy `"pose"` key is still accepted as an alias for `"waypoint"`, so existing configs keep working.

#### Attributes

| Name               | Type   | Inclusion | Description                                                                                                                                              |
|--------------------|--------|-----------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| `pose_switch_name` | string | Required  | Name of a switch component (typically `multi-poses-execution-switch`) that owns the poses.                                                               |
| `gripper_name`     | string | Optional  | If set, the gripper is opened once at the start of every run, and any per-step `grip` actions are dispatched to it. Required if any step has `grip` set. |
| `steps`            | Step[] | Optional  | Ordered list of `{ waypoint, grip }`. If unset, every waypoint on the switch is visited in switch order with no gripper action.                          |
| `pause_secs`       | float  | Optional  | Seconds to wait between steps. Defaults to 0.                                                                                                            |

##### Step

| Field      | Description                                                                                                                                                                          |
|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `waypoint` | Waypoint name on the switch. (The legacy key `pose` is accepted as an alias.)                                                                                                       |
| `grip`     | Optional gripper action on arrival. One of `"grab"` (close), `"open"`, `"open_grab"` (open, wait 1s for the gripper to settle, then close), or omitted. Same waypoint may be reused. |

### DoCommand

```json
{ "run": true }
```

Blocks until every step has been visited (or until the call's context is cancelled). Returns:

```json
{ "visited": ["home", "pickup-appr", "pickup", "stand1", "stand2", "dump-base", "dump", "pickup", "pickup-appr", "home"] }
```

Only one `run` may be in flight at a time; concurrent calls return an error.

## Model viam-labs:cup-mover:multi-poses-execution-switch

A `rdk:component:switch` that exposes N named waypoints as switch positions. `SetPosition(i)` moves to waypoint `i`. The `mode` attribute selects how waypoints are interpreted:

- **`pose`** (default) — each waypoint is an absolute Cartesian pose, planned via the Motion service. `component_name` is the frame to move.
- **`joint`** — each waypoint is an array of joint positions (radians), commanded directly via the arm's `MoveToJointPositions`. No Motion service is involved, so the planner / obstacle handling does not apply. `component_name` is the arm.

### Configuration

Pose mode:

```jsonc
{
  "mode": "pose",
  "component_name": "gripper",
  "motion": "builtin",
  "reference_frame": "world",
  "poses": [
    { "name": "home",        "x": 250, "y":   0, "z": 300, "o_z": -1, "theta":   0 },
    { "name": "pickup-appr", "x": 400, "y":   0, "z": 100, "o_z": -1, "theta":   0 },
    { "name": "pickup",      "x": 400, "y":   0, "z":  50, "o_z": -1, "theta":   0 },
    { "name": "stand1",      "x": 200, "y": 200, "z": 200, "o_z": -1, "theta":   0 },
    { "name": "stand2",      "x":   0, "y": 300, "z": 200, "o_z": -1, "theta":   0 },
    { "name": "dump-base",   "x": 180, "y":-150, "z": 192, "o_y": -1, "theta":   0 },
    { "name": "dump",        "x": 180, "y":-150, "z": 192, "o_y": -1, "theta": 180 }
  ]
}
```

Joint mode (joint values are in radians, one per arm joint; capture them with the dial-control `get_joints` command):

```jsonc
{
  "mode": "joint",
  "component_name": "arm",
  "poses": [
    { "name": "home",        "joints": [0,     0,      0,     0,     0,    0] },
    { "name": "pickup-appr", "joints": [0,     0.52,  -0.35,  0,     0.61, 0] },
    { "name": "pickup",      "joints": [0,     0.78,  -0.52,  0,     0.79, 0] },
    { "name": "stand1",      "joints": [0.79,  0.35,  -0.26,  0,     0.44, 0] },
    { "name": "dump",        "joints": [-0.79, 0.35,  -0.26,  0,     0.44, 3.14] }
  ]
}
```

> The waypoint array key remains `poses` in both modes for backward compatibility.

#### Attributes

| Name              | Type     | Inclusion              | Description                                                          |
|-------------------|----------|------------------------|----------------------------------------------------------------------|
| `mode`            | string   | Optional               | `"pose"` (default) or `"joint"`.                                     |
| `component_name`  | string   | Required               | In pose mode, the frame to move (`"gripper"` for gripper-tip destinations, `"arm"` for the flange). In joint mode, the arm component to command. |
| `motion`          | string   | Required in pose mode  | Name of the motion service to use (`"builtin"` for the default). Ignored in joint mode. |
| `reference_frame` | string   | Optional               | Frame the poses are expressed in (pose mode only). Defaults to `"world"`. |
| `poses`           | object[] | Required               | Named waypoints. Each requires `name`. Pose mode uses XYZ + OV degrees + theta; joint mode uses `joints` (radians). |

### DoCommand

```json
{ "set_position_by_name": "stand1" }
{ "get_current_position_name": true }
```

The standard switch API (`SetPosition`/`GetPosition`/`GetNumberOfPositions`) is also implemented.

### Obstacle handling

In **pose** mode the switch passes no `WorldState` to `motion.Move`, so the planner uses the machine's full frame system. Anything you've configured with `Geometries` (e.g. `erh:vmodutils:obstacle` for tables / walls / stands) is considered automatically.

In **joint** mode there is no planner — joint targets are sent straight to the arm, which interpolates between the current and target joint positions. There is no collision avoidance, so make sure each joint waypoint (and the straight-line joint interpolation between consecutive waypoints) is collision-free before running.

## Model viam-labs:cup-mover:dial-control-motion

A `rdk:service:generic` for interactively nudging an arm — useful for tuning waypoints before saving them into the switch. Supports Cartesian jogging plus capture helpers for both pose and joint waypoints:

- **Direct jog** — `{ "jog_z": 5 }` moves the arm +5mm on Z (negative values jog the other way). `jog_orientation` moves along the gripper's current orientation vector ("toward / away from where the gripper is pointing").
- **Stream Deck dial** — `{ "dial_move_z": 47 }` is the absolute dial position; the service infers direction from the delta and moves by the configured step. Handles rollover at the dial's bounds.
- **Joint jog** — `{ "jog_joint": 2, "by": 0.05 }` nudges joint index 2 by +0.05 radians (negative jogs the other way), leaving the other joints in place. Returns the resulting full joint vector.
- **Capture pose** — `{ "get_pose": true }` returns the current pose of the configured `frame_name` (defaults to the arm) as `{ x, y, z, o_x, o_y, o_z, theta }` — drop-in compatible with a pose-mode `multi-poses-execution-switch` entry.
- **Capture joints** — `{ "get_joints": true }` returns the arm's current joint positions as `{ "joints": [...] }` (radians) — drop-in compatible with a joint-mode switch entry.

### Configuration

```jsonc
{
  "arm_name": "arm",
  "frame_name": "gripper",
  "motion_service_name": "builtin",
  "dial_move_x_mm": 5,
  "dial_move_y_mm": 5,
  "dial_move_z_mm": 5,
  "dial_move_orientation_mm": 5,
  "dial_max_position": 100
}
```

| Name                       | Type   | Inclusion | Description                                                                                                                                                |
|----------------------------|--------|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `arm_name`                 | string | Required  | Arm component to move.                                                                                                                                     |
| `frame_name`               | string | Optional  | Frame queried by `get_pose`. Defaults to `arm_name`. Set to `"gripper"` to capture gripper-tip poses (matches a switch configured with `component_name: "gripper"`). |
| `motion_service_name`      | string | Optional  | Motion service used by `get_pose`. Defaults to `"builtin"`.                                                                                                 |
| `dial_move_*_mm`           | float  | Optional  | Step size per dial tick (defaults to 1mm). Only used by the `dial_move_*` commands.                                                                        |
| `dial_max_position`        | float  | Optional  | Max value the dial reports before wrapping (defaults to 100). Used for rollover math.                                                                       |

### Tuning workflow

1. Configure the dial-control service alongside your switch. For pose capture, set `frame_name` to match the switch's `component_name` (typically `"gripper"`) so captured poses go directly into the switch's `poses` array.
2. From the Control tab, jog the arm to the target:
   - Cartesian: `jog_x` / `jog_y` / `jog_z` / `jog_orientation` (start with `mm: 5`, drop to `mm: 1` near the target).
   - Joint: `{ "jog_joint": <index>, "by": <radians> }` to nudge an individual joint.
3. Once the arm is where you want it, capture the waypoint:
   - Pose-mode switch: `{ "get_pose": true }` → copy the response into a new `poses` entry (give it a `name`).
   - Joint-mode switch: `{ "get_joints": true }` → copy `joints` into a new `poses` entry as `{ "name": ..., "joints": [...] }`.
4. Repeat for each waypoint, then exercise the sequence with the cup-mover service → `{ "run": true }`.

## Build

```bash
make setup     # go mod tidy
make           # builds bin/cup-mover for the host arch
make test      # go test ./...
make module    # tests + packages module.tar.gz
```

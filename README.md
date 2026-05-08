# Module cup-mover

Drives an arm to move a cup through a configurable sequence of poses, optionally interacting with a gripper at each step.

The module ships three models:

| Model                                                | API                  | Purpose                                                            |
|------------------------------------------------------|----------------------|--------------------------------------------------------------------|
| `viam-labs:cup-mover:cup-mover`                      | `rdk:service:generic`| Orchestrator. Walks a configured step list and triggers gripper actions. |
| `viam-labs:cup-mover:multi-poses-execution-switch`   | `rdk:component:switch`| Stores N named absolute poses; `SetPosition(i)` moves a frame to pose `i` via the Motion service. |
| `viam-labs:cup-mover:dial-control-motion`            | `rdk:service:generic`| Interactive arm jog + pose capture, suitable for tuning poses before saving them. |

Typical wiring: dial-control to capture poses → paste them into the switch → orchestrator runs the sequence.

## Model viam-labs:cup-mover:cup-mover

A `rdk:service:generic` that walks an ordered list of `{ pose, grip }` steps. Each step moves the configured frame (via the switch) to the named pose, then optionally triggers a gripper action. If a gripper is configured, it's also opened once at the very start of every run so the arm approaches the first pose with an open gripper.

### Configuration

```jsonc
{
  "pose_switch_name": "cup-pose-switch",
  "gripper_name": "gripper",
  "steps": [
    { "pose": "home" },
    { "pose": "pickup-appr" },
    { "pose": "pickup",      "grip": "grab" },
    { "pose": "stand1",      "grip": "open_grab" },
    { "pose": "stand2",      "grip": "open_grab" },
    { "pose": "dump-base" },
    { "pose": "dump" },
    { "pose": "pickup",      "grip": "open" },
    { "pose": "pickup-appr" },
    { "pose": "home" }
  ],
  "pause_secs": 1.0
}
```

The trailing `pickup-appr → home` retracts the gripper through the approach pose and parks it, leaving the arm in a known state for the next run.

#### Attributes

| Name               | Type   | Inclusion | Description                                                                                                                                              |
|--------------------|--------|-----------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| `pose_switch_name` | string | Required  | Name of a switch component (typically `multi-poses-execution-switch`) that owns the poses.                                                               |
| `gripper_name`     | string | Optional  | If set, the gripper is opened once at the start of every run, and any per-step `grip` actions are dispatched to it. Required if any step has `grip` set. |
| `steps`            | Step[] | Optional  | Ordered list of `{ pose, grip }`. If unset, every pose on the switch is visited in switch order with no gripper action.                                  |
| `pause_secs`       | float  | Optional  | Seconds to wait between steps. Defaults to 0.                                                                                                            |

##### Step

| Field  | Description                                                                                                                                                                          |
|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `pose` | Pose name on the switch.                                                                                                                                                             |
| `grip` | Optional gripper action on arrival. One of `"grab"` (close), `"open"`, `"open_grab"` (open, wait 1s for the gripper to settle, then close), or omitted. Same pose may be reused. |

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

A `rdk:component:switch` that exposes N named absolute poses as switch positions. `SetPosition(i)` moves the configured frame to pose `i` via the Motion service.

### Configuration

```jsonc
{
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

#### Attributes

| Name              | Type     | Inclusion | Description                                                          |
|-------------------|----------|-----------|----------------------------------------------------------------------|
| `component_name`  | string   | Required  | Frame to move. Use `"gripper"` to specify destinations at the gripper tip; `"arm"` to specify them at the arm flange. |
| `motion`          | string   | Required  | Name of the motion service to use (`"builtin"` for the default).     |
| `reference_frame` | string   | Optional  | Frame the poses are expressed in. Defaults to `"world"`.             |
| `poses`           | object[] | Required  | Named absolute poses. Each requires `name`; XYZ + OV degrees + theta.|

### DoCommand

```json
{ "set_position_by_name": "stand1" }
{ "get_current_position_name": true }
```

The standard switch API (`SetPosition`/`GetPosition`/`GetNumberOfPositions`) is also implemented.

### Obstacle handling

The switch passes no `WorldState` to `motion.Move`, so the planner uses the machine's full frame system. Anything you've configured with `Geometries` (e.g. `erh:vmodutils:obstacle` for tables / walls / stands) is considered automatically.

## Model viam-labs:cup-mover:dial-control-motion

A `rdk:service:generic` for interactively nudging an arm — useful for tuning poses before saving them into the switch. Supports two flows plus a capture helper:

- **Direct jog** — `{ "jog_z": 5 }` moves the arm +5mm on Z (negative values jog the other way). `jog_orientation` moves along the gripper's current orientation vector ("toward / away from where the gripper is pointing").
- **Stream Deck dial** — `{ "dial_move_z": 47 }` is the absolute dial position; the service infers direction from the delta and moves by the configured step. Handles rollover at the dial's bounds.
- **Capture** — `{ "get_pose": true }` returns the current pose of the configured `frame_name` (defaults to the arm) as `{ x, y, z, o_x, o_y, o_z, theta }` — drop-in compatible with a `multi-poses-execution-switch` pose entry.

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

1. Configure the dial-control service alongside your switch. Set `frame_name` to match the switch's `component_name` (typically `"gripper"`) so captured poses go directly into the switch's `poses` array.
2. From the Control tab, send `jog_x` / `jog_y` / `jog_z` / `jog_orientation` DoCommands (start with `mm: 5`, drop to `mm: 1` near the target).
3. Once the arm is where you want it, send `{ "get_pose": true }`. Copy the response into a new entry under the switch's `poses` (give it a `name`).
4. Repeat for each waypoint, then exercise the sequence with the cup-mover service → `{ "run": true }`.

## Build

```bash
make setup     # go mod tidy
make           # builds bin/cup-mover for the host arch
make test      # go test ./...
make module    # tests + packages module.tar.gz
```

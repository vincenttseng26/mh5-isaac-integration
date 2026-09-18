# Robot Integration Module

This module provides the transport-neutral safety and interface layers for connecting to the MH5 robot.

## Interface Contract: Transform Chain

To accurately calculate object positions in the workspace, we maintain a strict transform chain. The definitions below dictate how coordinate frames are related.

### Frames
- `camera`: The optical center of the camera.
- `board`: The calibration board (fiducial marker) used for calibration.
- `robot_base`: The origin (0,0,0) of the robot base coordinate system.
- `tool` (TCP): The Tool Center Point (end effector).

### Transforms
1. **`T_camera_to_board`**: Obtained via camera intrinsics and PnP (Perspective-n-Point) on the calibration board.
2. **`T_robot_base_to_tool`**: Read from the robot controller's forward kinematics (TCP Pose).
3. **`T_tool_to_camera`** (Eye-in-Hand): The static transform from the robot TCP to the mounted camera.
4. **`T_robot_base_to_camera`** (Eye-to-Hand): The static transform from the robot base to the fixed camera.

## Eye-in-Hand vs Eye-to-Hand Data Exchange

Depending on the camera mounting, the required data exchange differs.

### Eye-in-Hand (Camera mounted on the robot arm)
- **Calibration Phase**:
  - Robot moves to multiple distinct poses.
  - At each pose, software captures `T_camera_to_board` and `T_robot_base_to_tool`.
  - Used to solve $AX=XB$ to find the static `T_tool_to_camera` ($X$).
- **Runtime Phase**:
  - To find an object's pose relative to the base: 
    `T_base_to_object` = `T_robot_base_to_tool` * `T_tool_to_camera` * `T_camera_to_object`

### Eye-to-Hand (Camera fixed in the environment)
- **Calibration Phase**:
  - The calibration board is mounted on the robot tool.
  - Robot moves to multiple distinct poses.
  - Software captures `T_camera_to_board` (which is `T_camera_to_tool`) and `T_robot_base_to_tool`.
  - Used to solve $AX=XB$ to find the static `T_robot_base_to_camera` ($X$).
- **Runtime Phase**:
  - To find an object's pose relative to the base:
    `T_base_to_object` = `T_robot_base_to_camera` * `T_camera_to_object`

## Safety
All motion commands are disabled by default (`NotImplementedError`). The `SafetyGate` class enforces state checks (E-stop, Servo, Alarm) and workspace boundaries before any command is considered valid.

## MH5 live mode gateway

The CYC gateway always starts in `MIRROR` mode and disarmed. `MIRROR` permits
physical-to-Isaac feedback only. `COMMAND` permits a single guarded trajectory
and automatically disarms after publication or disconnect.

```bash
# Deploy/restart without moving the robot
./robot_integration/start_bidirectional_mh5.sh

# Read mode, physical joint feedback and FS100 readiness
python3 robot_integration/send_mh5_trajectory.py --status

# Show the latest accepted/rejected physical-command audit events
python3 robot_integration/send_mh5_trajectory.py --audit

# Return immediately to the read-only mode
python3 robot_integration/send_mh5_trajectory.py --mirror
```

Do not use `--execute` for continuous policy output. Each accepted request is a
bounded point-to-point trajectory whose first waypoint must match fresh physical
feedback. The gateway rejects stale feedback, E-stop, controller error,
`motion_possible=0`, an already-moving robot, malformed joint order, and any
segment above `0.035 rad` (about 2 degrees).

The controller-side audit file is `/tmp/mh5_trajectory_audit.jsonl` inside the
ROS 1 runtime container. Published commands receive a unique `command_id` and
record their physical start, goal, duration and waypoint count. Rejections
record the same command ID and reason; observed motion completion additionally
records final feedback and maximum joint error.

For the Isaac GUI, launch `isaac_mh5_command_panel.py` with Isaac's Python
launcher. The panel starts in `MIRROR / DISARMED`; a real command requires
`PREVIEW VIRTUAL`, `ARM PREVIEW`, and `EXECUTE ONCE` within five seconds.

## MoveIt planning and fixed-workcell collision protection

Load the reviewed physical workcell into MoveIt's planning scene, then start
the plan-only gateway:

```bash
./robot_integration/apply_workcell_scene.sh
./robot_integration/start_moveit_plan_gateway.sh
```

The scene currently contains the worktable, floor, camera-frame post and beam,
and a conservative keep-out cylinder around the opposing MH5. The ZED bracket
and loose tabletop objects are intentionally excluded until their dimensions
and poses are measured.

Every request is checked by MoveIt's `/check_state_validity` service at both
the physical start state and requested goal before planning. A collision returns
an error naming the contacting bodies and is never passed to the execution
gateway. The Isaac panel uses the returned MoveIt/OMPL path for preview; physical
execution still requires the separate five-second one-shot arm action.

## ZED object detection and virtual grasp staging

`grasp_pipeline.py` turns an aligned point cloud into a top-down grasp. It is
pure geometry: it imports no transport, opens no socket and emits no motion.

The grasp convention is derived from the URDF and pinned by
`test_grasp_pipeline.py`, which asserts it end to end rather than in prose:

* `grasp_joint` applies `Ry(-90 deg)`, so **grasp_link +X is the approach
  direction**. A top-down grasp needs grasp_link +X aligned with base_link -Z.
* The RG2-FT fingers **separate along grasp_link +Y** (measured dot product
  1.000000 against the URDF chain).
* In `top_down_matrix(yaw)` the +Y axis is `(-sin yaw, cos yaw, 0)`, so aligning
  the fingers across an object's narrow side means **yaw is the angle of the
  object's long axis in the XY plane**.

### The ZED stream serves one client at a time

The CYC sender accepts a single connection, so a standalone detector cannot run
while the Isaac panel is up: the panel holds the socket and the second reader
times out with no error from the sender. Detection therefore runs *inside* the
panel, reusing the cloud `LiveZedOverlay` has already transformed into
base_link. `probe_live_zed_object_pose.py` remains useful only when the panel is
closed.

### Panel buttons

`DETECT OBJECT (ZED)` segments the profile colour in the live cloud, keeps the
largest voxel-connected cluster, and reports the base_link centroid, extent and
grasp yaw. It writes `zed_object_detection_latest.json` and draws a green
centroid marker plus blue/yellow stage markers in Isaac, so the overlay can be
compared against the physical object.

`PLAN + PLAY GRASP (VIRTUAL)` chains three plan-only MoveIt requests
(pre-grasp, grasp, lift) and animates the result in Isaac. It never opens the
execution gateway.

### Pose goals and chained virtual starts

The plan gateway now accepts a Cartesian goal in addition to a joint goal:

```bash
# joint goal, unchanged
python3 robot_integration/request_moveit_plan.py --target 0 0 0 0 -1.57 0
```

```python
{'target_pose': {'position_m': [x, y, z], 'yaw_rad': yaw}}   # grasp_link, base_link
{'target_pose': {...}, 'start_rad': [...]}                   # chain a virtual stage
```

Goals are solved with `/compute_ik` under `avoid_collisions=True` against the
fixed workcell scene, then planned and collision-checked exactly as joint goals
are. A reply carries `virtual_start: true` when the caller supplied the start.

A synthetic start cannot reach the robot: the execution gateway independently
rejects any trajectory whose first waypoint differs from fresh physical feedback
by more than `MAX_START_ERROR_RAD` (0.02 rad).

### The one-shot channel is a 2-degree jog, by design

`MAX_SEGMENT_RAD` (0.035 rad) bounds not only each segment but the trajectory's
TOTAL displacement:

```python
if max(abs(a-b) for a,b in zip(parsed[-1][0], actual)) > MAX_SEGMENT_RAD:
    raise ValueError('total trajectory displacement exceeds step limit')
```

Subdividing a long path therefore does not help. Measured against the blue tape
measure, a grasp needs far more than that:

| stage | worst axis | over the 2-degree cap |
| --- | --- | --- |
| pre-grasp | 50.57 deg | 25x |
| grasp | 16.45 deg | 8.2x |
| lift | 19.92 deg | 9.9x |

## The staged execution channel

Rather than widening the cap above -- which would silently weaken every existing
path -- there is a SEPARATE channel with its own, larger, explicitly-bounded
envelope. The original `COMMAND` path is untouched.

`staged_envelope.py` holds the rules and imports no ROS, so it is unit-tested
off the robot by `test_staged_envelope.py`:

| limit | value |
| --- | --- |
| `STAGED_MAX_VELOCITY_RAD_S` | 0.10 (the reviewed first-contact velocity) |
| `STAGED_MAX_TOTAL_RAD` | 1.20 (about 69 deg) per stage |
| `STAGED_MAX_SEGMENT_RAD` | 0.15 |
| `STAGED_MAX_START_ERROR_RAD` | 0.02 |
| `STAGED_MAX_DURATION_S` | 30 |

Properties worth stating explicitly:

* **Timestamps are recomputed, not validated.** Client timing is discarded
  entirely; the only stamps that reach the controller are derived from the
  velocity ceiling, so a client cannot ask for a faster move.
* **One stage per arm.** Arming in `STAGED` mode issues a single-use nonce that
  the trajectory must quote. The gateway disarms and returns to `MIRROR` the
  moment it publishes, so a replayed or duplicated request cannot fire again.
* **It is not a collision checker.** Collision-freedom comes from the MoveIt
  plan gateway. This envelope is kinematic only and cannot rescue a bad path.

### Running a stage

`run_staged_grasp.py` performs exactly one stage and never chains them. It
re-reads live feedback and re-plans that stage from where the robot actually is
-- the Isaac panel's virtual chain plans each stage from the *planned* end of
the previous one, which is right for animation and wrong for hardware.

```bash
# Plan and print the motion without touching the robot
python3 robot_integration/run_staged_grasp.py --stage pre_grasp

# Same, but publish it
python3 robot_integration/run_staged_grasp.py --stage pre_grasp --confirm
```

It refuses to run while the object profile still carries
`robot_motion_allowed: false`, if the detection report is stale, or if the
gateway is not found in `MIRROR` / disarmed with a healthy controller.

Do the stages in order and check the robot between each: `pre_grasp` first,
which stops 100 mm above the object in free space and so exposes a calibration
error where it is visible but harmless; only then `grasp` and `lift`.

## Guarded RG2-FT stages

The gripper has a separate one-shot gateway.  It starts in `MIRROR / DISARMED`,
requires `COMMAND -> arm -> nonce -> move` on one TCP connection, and returns to
`MIRROR` after one accepted move.  A disconnect revokes the nonce.  `stop`
remains available without a token.  The gateway accepts a measured target width
rather than only full-open/full-close, limits force to 3--12 N, and obtains
width/busy/grip-detected from Modbus registers 280--282.

Deploying the updated gateway does not move the gripper:

```bash
./robot_integration/start_gripper_gateway.sh
python3 robot_integration/run_staged_gripper.py --stage status
```

The first physical validation should be done with the arm at `home`, an empty
gripper, and the operator at E-stop.  Preview first; only the second command can
move the gripper:

```bash
python3 robot_integration/run_staged_gripper.py --stage open
python3 robot_integration/run_staged_gripper.py --stage open --confirm
python3 robot_integration/run_staged_gripper.py --stage status
```

`open` always means fully open (100 mm).  The separate `prepare` stage reads a
fresh ZED report and uses its computed `gripper_open_m`, so it may move inward
and must not be confused with full-open.  `close` defaults to 6 N but is
intentionally a separate confirmed invocation.
`verify` passes only when motion has stopped and the RG2-FT reports
`grip_detected=true`:

```bash
python3 robot_integration/run_staged_gripper.py --stage close --confirm
python3 robot_integration/run_staged_gripper.py --stage verify
```

The arm runner now applies cross-device interlocks as well: physical `grasp`
is rejected unless the gripper is stationary and open to within 2 mm of the
computed requirement; physical `lift` is rejected unless it is stationary and
reports a detected grip.  These checks intentionally make the old gateway
incompatible with physical `grasp`/`lift` until the guarded gateway is deployed.

### Default live-cloud alignment

The validated tabletop correction adds a runtime translation of
`[+0.025, 0, 0] m` in `base_link` (25 mm toward +X).  It is now the Isaac panel
default; the original correction JSON is unchanged.  Override it explicitly
with `--zed-translation-delta-m DX DY DZ` when testing another candidate.

## Pure Mock Read-Only Probe CLI
A mock/read-only probe CLI is available to test the JSON schema and read-only behavior. It outputs status, joint pose (in radians), TCP pose (in mm/radians), and timestamp as JSON. It strictly blocks any motion.

```bash
python3 -m robot_integration.probe_cli --dry-run
```

# teleop

XR teleoperation for the UR arm. `teleop.py` reads an XR controller through XRoboToolkit, drives the arm
directly over RTDE (up to 500 Hz, no ROS driver in the loop), and drives the DH gripper over the UR
tool-communication serial port. While it runs, it writes a JSON state file that the
[recorder](../recorder/README.md) samples as the action stream, and it starts, stops, saves, and discards
recorder episodes from the controller buttons.

Pipeline: **teleop** → recorder → converter → policy_client.

This directory is not a colcon package: run the scripts with `python3` from the workspace root. Building
and sourcing the workspace are covered in the [top-level README](../../README.md).

## How it works

```text
XR controller → XRoboToolkit PC Service → xrobotoolkit_sdk → teleop.py
    ├─ RTDE speedJ / servoJ / servoL ──→ UR arm
    ├─ /tmp/ttyUR (Modbus) ────────────→ DH gripper
    ├─ /tmp/ur_teleop_state.json ──────→ recorder (action + observation)
    └─ ros2 service call /recorder/* ──→ recorder (start / stop / save / discard)
```

- **Clutch.** Holding the right grip engages motion. Teleop stores the controller and TCP poses as
  references, and the TCP target then follows the controller's displacement (× `--scale-factor`) and
  rotation (× `--rotation-scale`). Releasing the grip stops the arm; re-engaging sets new references, so
  the arm does not jump.
- **Frames.** A fixed rotation (`transforms.py`) maps the XR frame to the robot base. Start the XR client
  facing the robot base +X axis.
- **Control modes** (`--control-mode`). `speed_j_dls` (recommended) turns the TCP pose error into a
  velocity twist, converts it to joint velocities with a damped least-squares (DLS) inverse of the
  Jacobian, limits them, and sends `speedJ`. `servo_j` (the default) converts the TCP target to joints with
  the UR controller's IK and sends `servoJ`. `servo_l` sends the TCP target with `servoL`.
- **DLS near singularities.** `qdot = Jᵀ (J Jᵀ + λ² I)⁻¹ v`. Below `--dls-singularity-threshold` (smallest
  singular value of J), the twist is scaled down linearly and λ rises from `--dls-damping` towards
  `--dls-max-damping`. At `--dls-hard-stop-threshold`, motion stops.
- **SpeedJ backends** (`--speedj-backend`). `direct` calls RTDE `speedJ` on every Python cycle.
  `register_bridge` uploads a URScript that runs `speedj` in a 2 ms loop on the robot and reads joint
  velocities from RTDE input integer registers, so Python timing jitter does not reach the arm. The script
  sends zero velocity when updates stop for `--speedj-watchdog-sec` and exits after
  `--speedj-stale-stop-sec`. Teleop re-uploads it when needed.
- **Jacobian** (`--dls-jacobian-source`). `local` builds the URDF for `--ur-type` from `ur_description` and
  computes the Jacobian with Pinocchio, including the robot's active TCP offset. At startup it checks local
  FK against the RTDE TCP pose. `rtde` uses `getJacobian` and does not work with `register_bridge`.

## Requirements

- UR controller in Remote Control mode, with the ToolComm Forwarder URCap running for the gripper.
- XRoboToolkit PC Service on the workstation (default `/opt/apps/roboticsservice`), XR client on the headset.
- ROS 2 Jazzy with `ur_description` and `ur_client_library`, sourced before teleop (the recorder is called
  through the `ros2` CLI).
- A Python environment `<CONDA_ENV>` with `xrobotoolkit_sdk`, `ur_rtde`, `numpy`, and `pyserial`; plus
  `pinocchio` and `xacro` for the local Jacobian, and `ruckig` for Ruckig smoothing.
- The DH gripper driver from [`src/controllers`](../controllers/README.md), imported from the source tree.
- For stable 500 Hz control: a low-latency kernel, the `performance` CPU governor, and real-time limits
  for your user (`ulimit -r` at least the requested priority, `ulimit -l unlimited`).

```bash
uname -r                                     # expect a *-lowlatency kernel
ulimit -r; ulimit -l                         # e.g. 99 and unlimited
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c
python -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

## Quick start

Run each step in its own terminal from `<UR_WORKSPACE>`.

```bash
# 1. XRoboToolkit PC Service with SCHED_FIFO priority (--restart restarts it)
src/teleop/start_xr_service_rt.sh

# 2. Power on, release brakes, wait for RUNNING / NORMAL (pendant in Remote Control)
python3 src/teleop/start_ur.py --robot-ip <ROBOT_IP>

# 3. Expose the gripper serial port as /tmp/ttyUR; keep this terminal open
source /opt/ros/jazzy/setup.bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py <ROBOT_IP> --device-name /tmp/ttyUR
```

Start teleop with the recommended settings: 500 Hz `speed_j_dls` through the register bridge, Ruckig
smoothing, recorder control, and replay. Flags equal to their defaults are left out, except the gripper
port and state-file settings, which must match ToolComm and the recorder.

```bash
conda activate <CONDA_ENV>
source /opt/ros/jazzy/setup.bash
source <UR_WORKSPACE>/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip <ROBOT_IP> --ur-type <UR_TYPE> \
  --control-mode speed_j_dls --speedj-backend register_bridge \
  --realtime-priority 80 --lock-memory --xr-service-realtime-priority 60 \
  --grip-threshold 0.8 --grip-release-threshold 0.55 --xr-timeout-sec 0.35 \
  --scale-factor 1 \
  --max-step-translation 0.001 --max-step-rotation 0.0019 \
  --target-smoothing-mode ruckig --target-smoothing-sec 0.008 \
  --ruckig-max-linear-acceleration 1.6 \
  --ruckig-target-prediction-sec 0 --ruckig-velocity-hold-sec 0 \
  --max-target-lag-translation 0.016 --max-target-lag-rotation 0.055 \
  --dls-linear-gain 14.5 --dls-angular-gain 8 --dls-damping 0.12 \
  --dls-orientation-deadband 0.006 \
  --max-tcp-angular-speed 0.95 --max-joint-speed 0.68 --max-joint-acceleration 3.0 \
  --gripper-port /tmp/ttyUR --state-file /tmp/ur_teleop_state.json --state-hz 30 \
  --enable-recorder-control \
  --return-home-max-joint-step 0.003 --return-home-timeout-sec 30 \
  --replay-after-return-home --replay-delay-sec 0
```

A simpler variant: `direct` speedJ backend, default DLS gains and limits, no real-time tuning.

```bash
python3 src/teleop/teleop.py \
  --robot-ip <ROBOT_IP> --ur-type <UR_TYPE> --control-mode speed_j_dls \
  --scale-factor 0.8 --rotation-scale 0.6 \
  --max-step-translation 0.002 --max-step-rotation 0.004 \
  --enable-recorder-control --return-home-timeout-sec 30
```

Then start the recorder with `record_hz:=30.0` and `teleop_state_file:=/tmp/ur_teleop_state.json` (see the
[recorder README](../recorder/README.md)). Stop teleop with Ctrl+C or the stop button.

## Controller mapping

| Input | Action |
| --- | --- |
| Right grip (held) | Arm follows the right controller |
| Right trigger | Gripper: released = open, pressed = closed (proportional by default); works without the grip |
| Right stick click | Software stop: stops the arm and exits teleop |
| A / X / Y | `/recorder/start` / `/recorder/save` / `/recorder/discard` |
| B | Return home, then `/recorder/stop` |
| Left grip | Replay the last episode (with `--replay-after-return-home`) |

A, B, X, and Y need `--enable-recorder-control`.

**Return home and replay.** On **B**, teleop disengages and moves the arm to `--return-home-joints` at a
limited speed. This motion is written with status `return_home`, so it is part of the episode. When every
joint stays within `--return-home-tolerance` for `--return-home-hold-sec` (or after
`--return-home-timeout-sec`), teleop waits `--return-home-stop-grace-sec` and calls `/recorder/stop`
(blocking, 3 s timeout). With `--replay-after-return-home`, if the episode was started with **A**, home was
reached, and the stop succeeded, teleop loads the trajectory the recorder wrote to
`/tmp/ur_sampled_replay.json`. `--replay-button` then replays its joint positions and gripper targets after
`--replay-delay-sec`; release the grip afterwards before teleop resumes. `--disable-return-home-before-stop`
makes **B** call `/recorder/stop` immediately.

## Options

Units are metres, radians, and seconds. "Per cycle" limits apply once per control cycle, so they scale
with `--rate-hz`: at 500 Hz, 0.001 m per cycle is 0.5 m/s. The legacy `--replay-record-hz` is omitted; it
does not affect the replayed trajectory.

### Robot and runtime

| Flag | Default | Description |
| --- | --- | --- |
| `--robot-ip` | lab address (hard-coded) | UR controller address; always pass `<ROBOT_IP>` |
| `--ur-type` | `ur7e` | Robot model used to build the local URDF |
| `--control-mode` | `servo_j` | `speed_j_dls`, `servo_j`, or `servo_l` |
| `--rate-hz` | `500.0` | Control loop rate; also the RTDE interface frequency |
| `--realtime-priority` / `--lock-memory` | `0` / off | SCHED_FIFO priority for teleop (`0` disables it) / lock memory with `mlockall` to avoid page-fault stalls |
| `--xr-service-realtime-priority` | `0` | SCHED_FIFO priority applied to running `RoboticsService` processes at startup |
| `--control-latency-warning-sec` | `0.02` | Warn when the gap between control cycles exceeds this; `0` disables it |
| `--max-velocity`, `--max-acceleration`, `--lookahead-time`, `--servo-gain` | `0.5`, `1.0`, `0.06`, `500.0` | Arguments passed to `servoJ` / `servoL` |
| `--ik-max-position-error` / `--ik-max-orientation-error` | `1e-4` / `1e-3` | IK tolerances in `servo_j` |

### SpeedJ and DLS (`speed_j_dls`)

| Flag | Default | Description |
| --- | --- | --- |
| `--speedj-backend` | `direct` | `direct` or `register_bridge` |
| `--speedj-register-start` | `42` | First of 5 RTDE input integer registers; only `18` or `42` are accepted |
| `--speedj-watchdog-sec` / `--speedj-stale-stop-sec` | `0.06` / `1.0` | Without updates, the bridge sends zero velocity / exits; stale-stop must exceed watchdog |
| `--speedj-velocity-scale` / `--speedj-upload-port` | `10000.0` / `30001` | Fixed-point scale of the register encoding; port for the URScript upload |
| `--speed-slider-fraction` | `1.0` | UR speed slider set when the bridge starts (`register_bridge` only) |
| `--dls-linear-gain` / `--dls-angular-gain` | `8.0` / `6.0` | Gains from TCP position (m) / orientation (rad) error to commanded velocity |
| `--dls-damping` / `--dls-max-damping` | `0.03` / `0.25` | DLS damping λ away from singularities / at the hard stop |
| `--dls-position-deadband` / `--dls-orientation-deadband` | `0.0` / `0.0` | Errors smaller than this are ignored |
| `--dls-singularity-threshold` / `--dls-hard-stop-threshold` | `0.05` / `0.015` | Smallest singular value at which slowdown starts / motion stops |
| `--dls-jacobian-source` | `local` | `local` (Pinocchio) or `rtde`; `register_bridge` requires `local` |
| `--dls-frame-name` / `--ur-description-path` | `tool0` / ROS package | URDF frame for the local Jacobian (TCP offset added); `ur_description` location |
| `--max-tcp-linear-speed` / `--max-tcp-angular-speed` | `0.25` / `0.8` | Caps on the commanded twist (m/s, rad/s); the linear cap also bounds Ruckig |
| `--max-joint-speed` / `--max-joint-acceleration` | `0.8` / `2.0` | Joint velocity (rad/s) and acceleration (rad/s²) caps; also used by return home and replay |

### XR input

Analog inputs: `left_grip`, `right_grip`, `left_trigger`, `right_trigger`. Buttons: `A`, `B`, `X`, `Y`,
`left_axis_click`, `right_axis_click`, `left_menu_button`, `right_menu_button`.

| Flag | Default | Description |
| --- | --- | --- |
| `--pose-source` | `right_controller` | `left_controller`, `right_controller`, or `headset` |
| `--grip-name` / `--trigger-name` | `right_grip` / `right_trigger` | Analog input that enables motion / controls the gripper |
| `--emergency-button` | `right_axis_click` | Button that stops the arm and exits teleop |
| `--grip-threshold` / `--grip-release-threshold` | `0.95` / threshold − 0.15 | Grip value that engages motion / below which it disengages |
| `--grip-release-grace-sec` | `0.0` | How long the grip may dip below the release threshold before disengaging |
| `--xr-timeout-sec` | `0.2` | Stop when the XR timestamp has not changed for this long |
| `--xr-target-interpolation-sec` | `0.0` | During XR gaps, extrapolate the target at the last velocity for up to this long |
| `--scale-factor` / `--rotation-scale` | `0.6` / `1.0` | Controller-to-TCP scaling of translation / rotation |

### Target shaping and safety limits

| Flag | Default | Description |
| --- | --- | --- |
| `--target-smoothing-mode` | `linear` | `none`: step limits only. `linear`: plus first-order smoothing. `ruckig`: jerk-limited translation, step-limited rotation |
| `--target-smoothing-sec` | `0.0` | Smoothing time constant (`linear`, and rotation in `ruckig`) |
| `--max-step-translation` / `--max-step-rotation` | `0.01` / `0.05` | Max TCP target change per cycle (m / rad) |
| `--max-joint-step` | `0.04` | Max joint change per cycle in `servo_j` and servo-mode replay (rad) |
| `--ruckig-max-linear-acceleration` / `--ruckig-max-linear-jerk` | `1.5` / `30.0` | Ruckig limits (m/s², m/s³) |
| `--ruckig-target-prediction-sec` / `--ruckig-velocity-hold-sec` | `0.006` / `0.015` | Look-ahead on the target / how long the last velocity is kept when XR samples repeat |
| `--max-target-lag-translation` / `--max-target-lag-rotation` | `0.0` / `0.0` | Max lead of the target over the measured TCP (`speed_j_dls`); `0` disables it |
| `--workspace-min` / `--workspace-max` | none | TCP position bounds `x,y,z` in the base frame |
| `--unsafe-disable-teleop-safety-limits` | off | Bypass all teleop-side limits (see [Safety notes](#safety-notes)) |

### Gripper

| Flag | Default | Description |
| --- | --- | --- |
| `--disable-gripper` | off | Run without the gripper |
| `--gripper-port` / `--gripper-baudrate` | `/tmp/ttyUR` / `115200` | Serial port created by `tool_communication.py` |
| `--gripper-rate-hz` / `--gripper-read-hz` | `30.0` / `30.0` | Command and position-read rates; both capped at 50 Hz |
| `--gripper-open-raw` / `--gripper-closed-raw` | `1000` / `0` | Raw positions (0–1000) for trigger released / pressed |
| `--gripper-binary-control` / `--gripper-trigger-threshold` | off / `0.5` | Snap open/closed at the threshold instead of proportional control |
| `--gripper-speed` / `--gripper-force` | `100` / `50` | DH speed (1–100) and force (20–100), in percent |
| `--gripper-deadband` / `--gripper-init-timeout-sec` | `4` / `8.0` | Skip target changes smaller than this (raw); initialization timeout |

### State file, recorder control, return home, replay

| Flag | Default | Description |
| --- | --- | --- |
| `--state-file` / `--state-hz` | `/tmp/ur_teleop_state.json` / `30.0` | Path and write rate; match the recorder's `teleop_state_file` and `record_hz` |
| `--disable-state-file` | off | Do not write the state file |
| `--enable-recorder-control` | off | Map XR buttons to the recorder services |
| `--recorder-{start,stop,save,discard}-button` | `A` / `B` / `X` / `Y` | Button for each recorder action |
| `--recorder-debounce-sec` | `0.5` | Minimum time between two calls of the same action |
| `--disable-return-home-before-stop` | off | Stop button calls `/recorder/stop` without returning home |
| `--return-home-joints` | built-in | Home pose `q1,...,q6`; default ≈ `-π/2,-π/2,-π/2,-π/2,π/2,-0.03` |
| `--return-home-max-joint-step` | `0.0006` | Max joint change per cycle while returning home (0.3 rad/s at 500 Hz) |
| `--return-home-tolerance` / `--return-home-hold-sec` | `0.01` / `0.3` | Home is reached when all joint errors stay below the tolerance for this long |
| `--return-home-timeout-sec` / `--return-home-stop-grace-sec` | `20.0` / `0.2` | Give up and stop the recorder after this long / wait before calling stop |
| `--replay-after-return-home` | off | Enable replay of the last episode |
| `--replay-button` / `--replay-trigger-threshold` | `left_grip` / `0.8` | Replay input (comma-separated list allowed); threshold for analog inputs |
| `--replay-delay-sec` / `--replay-speed-scale` | `2.0` / `1.0` | Delay after the button press; time scale (above 1 is faster) |
| `--replay-joint-gain` / `--replay-tolerance` | `10.0` / `0.02` | Joint error feedback gain (`speed_j_dls`); final joint error that ends the replay |

## Files and interfaces

| Interface | Use |
| --- | --- |
| RTDE receive / control | Joint and TCP state; `speedJ`, `servoJ`, `servoL` |
| URScript upload (`--speedj-upload-port`), input int registers 42–46 or 18–22 | `register_bridge` only; replaces the running RTDE control script |
| `/tmp/ttyUR` | DH gripper over Modbus |
| `--state-file` | Written for the recorder; replaced atomically on each write |
| `/recorder/{start,stop,save,discard}` | `std_srvs/srv/Trigger`, called through `ros2 service call` |
| `/tmp/ur_sampled_replay.json` | Read for replay; written by the recorder on stop (fixed path) |

State file fields. The recorder ignores a file older than its `teleop_state_timeout_sec` (default 1 s).

| Field | Meaning |
| --- | --- |
| `wall_time`, `status`, `control_mode` | Unix write time; `running`, `idle`, `replay_ready`, `xr_timeout`, `invalid_xr_pose`, `return_home`, `replay_wait`, or `replay`; active mode |
| `actual_q`, `actual_tcp_pose` | Measured joints and TCP pose (rotation vector); the recorder's observation |
| `target_q`, `target_tcp_pose` | Commanded target; the recorder's action. Only in `running`, `return_home`, and `replay` |
| `gripper_target_raw`, `gripper_current_raw` | Gripper command and measured position (0–1000), or `null` |
| `motion_grip_value`, `motion_enabled` | Grip value and whether motion is engaged |
| `xr_age_sec`, `xr_target_interpolated` | Time since the last new XR sample; set when the target was extrapolated |

In `speed_j_dls` the arm is commanded by velocity, so `target_q` is a prediction one state period ahead:
`actual_q + qdot / state_hz`.

## Helper scripts

| Script | Purpose | Options |
| --- | --- | --- |
| `start_xr_service_rt.sh` | Start `RoboticsServiceProcess` under `chrt -f`, or boost a running instance | `--restart`; env `XR_SERVICE_DIR` (`/opt/apps/roboticsservice`), `XR_SERVICE_RT_PRIORITY` (`60`), `XR_SERVICE_LOG_FILE` (`/tmp/xrobotics_service.log`) |
| `start_ur.py` | Via the Dashboard Server: check Remote Control, power on, release brakes, wait for `RUNNING` and `NORMAL` | `--robot-ip`, `--dashboard-port` (`29999`), `--unlock-protective-stop`, `--state-timeout-sec` (`15`); status only: `--skip-power-on --skip-brake-release --no-wait-running` |
| `diagnose_xr_stream.py` | Measure the real XR update rate, interval percentiles, and gaps ≥ 50/100/200 ms; exits 1 on gaps | `--duration-sec` (`20`), `--poll-hz` (`1000`), `--warn-gap-sec` (`0.05`), `--pose-source` |

The XR stream is much slower than the 500 Hz control loop; teleop reuses the latest sample every cycle.

## Safety notes

- Only one program may command the arm and only one process may own `/tmp/ttyUR`. Do not run
  `ur_robot_driver`, MoveIt Servo, `controllers`, or `policy_client` alongside teleop.
- The stop button is a software stop that ends the process. Keep the teach-pendant e-stop within reach.
- Per-cycle limits scale with `--rate-hz`. At 500 Hz the default `--max-joint-step 0.04` allows 20 rad/s,
  so `servo_j` effectively relies on the UR controller's own limits.
- `--unsafe-disable-teleop-safety-limits` bypasses smoothing, workspace clamp, step and lag limits, TCP
  speed caps, DLS deadbands, and joint velocity and acceleration limits. The stop button, UR controller
  protections, and the DLS singularity hard stop stay active.
- After an XR timeout or invalid pose, the grip must be engaged again. The timeout is not applied while
  the SDK timestamp is 0 (client connected but not tracking).
- Replay starts by driving from home to the episode's first pose at up to `--max-joint-speed`. Clear the
  path before pressing the replay button.
- `register_bridge` writes RTDE input integer registers 42–46 (or 18–22); keep other programs off them.
- Use `start_ur.py --unlock-protective-stop` only after checking the cell.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `xrobotoolkit_sdk is not importable` | Activate `<CONDA_ENV>`; start the PC Service |
| `XR input timed out; holding robot.` | Headset client, Wi-Fi, `diagnose_xr_stream.py`; raise `--xr-timeout-sec` |
| XR timestamp stays `0` | PC Service is up but the headset client is not sending tracking data |
| Arm does not move while gripping | Remote Control, `start_ur.py`, grip above `--grip-threshold` (`motion_grip_value`), other RTDE clients; after a replay, release the grip first |
| `Local DLS kinematics is not aligned with RTDE TCP pose` | `--ur-type`, robot calibration, TCP offset, `--ur-description-path` (aborts above 0.08 m or 0.35 rad) |
| `speed_j_dls requires pinocchio and xacro` / `Ruckig target smoothing requires ...` | Install `pinocchio`, `xacro`, or `ruckig` in `<CONDA_ENV>`; source ROS |
| `speedj register_bridge replaces the RTDE control script` | Use `--dls-jacobian-source local` |
| `DLS singularity damping active` | Move away from the singular pose (aligned wrist or straight elbow) |
| `DH gripper disabled: ...` | ToolComm terminal running, `/tmp/ttyUR` exists and is free; or pass `--disable-gripper` |
| `ros2 command is not available` / recorder call timed out | Source ROS before teleop; check that the recorder node is running |
| Recorder reports a missing or stale action | `--state-file` matches `teleop_state_file`; targets are only written while the grip is held |
| `Replay skipped: ...` | Press A before B; home must be reached and `/recorder/stop` must succeed |
| `Teleop control loop latency gap` or jitter | Low-latency kernel, `--realtime-priority`, `--lock-memory`, CPU governor, network |

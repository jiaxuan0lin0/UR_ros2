# recorder

ROS 2 package that records teleoperated demonstrations: RealSense color images, observed UR arm and gripper state, and the action targets that `teleop.py` is commanding. Each recording is saved as one episode folder.

Pipeline: [teleop](../teleop/README.md) → **recorder** → [converter](../converter/README.md) → [policy_client](../policy_client/README.md)

The recorder never commands the arm or the gripper. Workspace setup and build are covered in the [top-level README](../../README.md).

## How it works

- `teleop.py` writes its latest state to a JSON file (default `/tmp/ur_teleop_state.json`) at `--state-hz`.
- The `recorder` node ticks at `record_hz`. On each tick it grabs one color frame from every connected RealSense camera, stamps `time.time()`, reads the teleop state file, optionally reads its own RTDE connection, and queues one sample for a background writer.
- **Observation** comes from the teleop file (`actual_q`, `actual_tcp_pose`, `gripper_current_raw`). The recorder's own RTDE reader (`use_rtde_reader:=true`) only fills fields the teleop file lacks.
- **Action** comes from the teleop file (`target_q`, `target_tcp_pose`, `gripper_target_raw`).
- A tick is **skipped** if the state file is missing, unreadable, older than `teleop_state_timeout_sec`, has no `target_q`/`target_tcp_pose` (for example teleop is `idle` because the grip is released), or any observation field is missing. Skipped ticks use no `frame_idx`. As a result `frame_idx` has no gaps but `timestamp` can. Late ticks are not caught up.
- Episode states: `idle` → **start** → `recording` → **stop** → `stopped` → **save** or **discard** → `idle`.

## Quick start

1. Start teleop with a state file and recorder control enabled. The full command is in the [teleop README](../teleop/README.md). The relevant flags are `--state-file /tmp/ur_teleop_state.json --state-hz 30 --gripper-read-hz 30 --enable-recorder-control`.

2. Launch the recorder. Use a shell where the workspace is sourced and the Python environment with `pyrealsense2` is active (`ur-rtde` is only needed with `use_rtde_reader:=true`):

```bash
ros2 launch recorder record.launch.py \
  dataset_root:=<DATASET_DIR> \
  record_hz:=30.0 \
  color_fps:=30 \
  teleop_state_file:=/tmp/ur_teleop_state.json
```

3. Record with the XR buttons (A start, B stop, X save, Y discard), the keyboard client, or the services listed under [Interfaces](#interfaces):

```bash
ros2 run recorder keyboard_control
```

4. Check the saved episodes. This uses only the standard library, so TensorFlow is not needed:

```bash
python3 <UR_WORKSPACE>/src/converter/scripts/RLDS.py --dataset-root <DATASET_DIR> --inspect-only
```

Only `record_hz` can be changed while the node runs:

```bash
ros2 param set /recorder record_hz 15.0
```

## Parameters

Launch arguments for `record.launch.py`. All of them except `python_executable` are passed on as ROS parameters of the `/recorder` node.

| Name | Default | Description |
| --- | --- | --- |
| `dataset_root` | `$UR_WORKSPACE/dataset` | Where episode folders are created. If `UR_WORKSPACE` is unset, the launch file uses `~/UR/dataset`. |
| `record_hz` | `30.0` | Sample rate. Must be > 0 and ≤ `color_fps`: the node refuses to start, and `ros2 param set` is rejected, otherwise. Can be changed at runtime. |
| `color_width` | `640` | RealSense color width (px). |
| `color_height` | `480` | RealSense color height (px). |
| `color_fps` | `30` | RealSense color frame rate. |
| `teleop_state_file` | `/tmp/ur_teleop_state.json` | Must match teleop `--state-file`. |
| `teleop_state_timeout_sec` | `1.0` | Maximum age of the state file (taken from its `wall_time` field) for a sample to count as fresh. |
| `use_rtde_reader` | `false` | Open a separate RTDE receive connection, used only as a fallback for observation fields. |
| `robot_ip` | hard-coded lab IP | UR controller address. Used only when `use_rtde_reader:=true`. Pass `robot_ip:=<ROBOT_IP>`. |
| `rtde_frequency` | `-1.0` | RTDE receive frequency. `-1.0` uses the library default. A warning is logged if it is > 0 and below `record_hz`. |
| `diagnostics_sec` | `2.0` | Interval for the "Recorder timing max" log line. `0` disables it. |
| `sample_warning_sec` | `0.04` | Logs "Recorder slow sample" when one sample takes longer than this. `0` disables it. |
| `python_executable` | `$VIRTUAL_ENV/bin/python3`, or `/usr/bin/python3` if no virtualenv is active | Interpreter used to run `python -m recorder.record`. It must be able to import `pyrealsense2`, `cv2` and `numpy`, plus `rtde_receive` when `use_rtde_reader:=true`. |

Rate rules: `record_hz` ≤ `color_fps`. Teleop `--state-hz` and `--gripper-read-hz` should be ≥ `record_hz`, otherwise consecutive samples repeat the same values.

If you run the node without the launch file (`ros2 run recorder record`), `dataset_root` defaults to `~/UR/dataset`.

## Output format

```text
<DATASET_DIR>/
  episode_<start_unix_ms>/
    cam_0/00000.jpg, 00001.jpg, ...
    cam_1/00000.jpg, ...
    robot_states.json
```

- **Images**: BGR frames from RealSense, written with OpenCV as normal color JPEGs at `color_width`×`color_height`. The file name is `frame_idx` zero-padded to 5 digits.
- **Camera numbering**: `cam_N` follows RealSense device enumeration order. Camera serial numbers are not saved, so keep the cameras on the same ports between sessions. If a camera returns no frame on a tick, that image is missing but the record is still written, and the converter will reject the episode.
- **`robot_states.json`**: a JSON list with one object per sample. It is written only on **save**. Images are written while recording.

Per-sample fields:

| Key | Type | Units | Source |
| --- | --- | --- | --- |
| `frame_idx` | int | — | 0 … N-1, no gaps |
| `timestamp` | float | Unix seconds | `time.time()` taken just after the camera frames are grabbed |
| `status` | str, optional | — | Teleop status. Only statuses that carry targets get recorded: `running`, `return_home`, `replay` |
| `control_mode` | str, optional | — | Teleop `--control-mode` (`servo_j`, `servo_l`, `speed_j_dls`) |
| `observation.joint_positions` | 6 floats | rad | Teleop `actual_q` (RTDE `getActualQ`) |
| `observation.tcp_pose` | 6 floats | m, rad | Teleop `actual_tcp_pose` (RTDE `getActualTCPPose`): `[x, y, z, rx, ry, rz]`, rotation vector, base frame |
| `observation.gripper_position` | float | 0–1 | Teleop `gripper_current_raw` / 1000 |
| `action.joint_positions` | 6 floats | rad | Teleop `target_q` |
| `action.tcp_pose` | 6 floats | m, rad | Teleop `target_tcp_pose`, same convention as the observation |
| `action.gripper_position` | float | 0–1 | Teleop `gripper_target_raw` / 1000. Falls back to the observed value if the target is missing |

Gripper raw values are clipped to `[0, 1000]` and divided by 1000. With the teleop defaults (`--gripper-open-raw 1000 --gripper-closed-raw 0`), `1.0` means open and `0.0` means closed. `action.gripper_position` is the commanded target at that tick, not the next observed position (see `fix_gripper_targets` below).

```json
{"frame_idx": 0, "timestamp": 1710000000.123,
 "observation": {"joint_positions": [0, 0, 0, 0, 0, 0], "tcp_pose": [0, 0, 0, 0, 0, 0], "gripper_position": 0.42},
 "action": {"joint_positions": [0, 0, 0, 0, 0, 0], "tcp_pose": [0, 0, 0, 0, 0, 0], "gripper_position": 0.45},
 "status": "running", "control_mode": "speed_j_dls"}
```

**Sampled replay file.** On **stop**, the recorder writes `/tmp/ur_sampled_replay.json`, and it deletes this file on **start**. The path is hard-coded and is not a parameter. Teleop loads the file for `--replay-after-return-home`. It contains `{episode_dir, point_count, points}`. Each point holds `time` (seconds since the first sample), the action `q`/`tcp`, the observation `observation_q`/`observation_tcp`, raw gripper values (`gripper_raw` = `gripper_target_raw`, and `gripper_observation_raw`), and `phase` (the record's `status`).

## Interfaces

The node does not publish or subscribe to any topics. It provides four services, all of type `std_srvs/srv/Trigger`:

| Service | Valid in state | Effect |
| --- | --- | --- |
| `/recorder/start` | `idle` | Creates `episode_<ms>/` and starts sampling. Clears the replay file. |
| `/recorder/stop` | `recording` | Stops sampling, waits for queued images to finish writing, and writes the replay file. |
| `/recorder/save` | `stopped` | Writes `robot_states.json`. An episode with zero samples is deleted instead, and the call returns `success=false`. |
| `/recorder/discard` | `stopped` | Deletes the episode folder. |

```bash
ros2 service call /recorder/start std_srvs/srv/Trigger {}
```

| Keyboard (`keyboard_control`) | XR (teleop `--enable-recorder-control`) | Action |
| --- | --- | --- |
| `r` | `A` | start |
| `c` | `B`: drives the arm home first, then stops (stops at once with `--disable-return-home-before-stop`) | stop |
| `s` | `X` | save |
| `q` | `Y` | discard |
| `x` | — | exit the keyboard client |

XR buttons can be remapped with teleop's `--recorder-{start,stop,save,discard}-button` flags.

Shutting down the recorder (Ctrl-C) **discards** any episode that has not been saved.

## Utility scripts

- **`fix_gripper_targets`**: rewrites `action.gripper_position` to the next frame's `observation.gripper_position` in every `robot_states.json` found recursively under the dataset root. The last frame keeps its own observation. Use it only for next-state-prediction datasets. It edits the files in place and the change cannot be undone, so back up the dataset first. Without `--dataset-root` it scans `~/UR/dataset`.
  ```bash
  ros2 run recorder fix_gripper_targets --dataset-root <DATASET_DIR> --dry-run   # report only
  ros2 run recorder fix_gripper_targets --dataset-root <DATASET_DIR>
  ```
- **`sampled_replay.py`**: library module with no CLI. `/recorder/stop` calls it to build the sampled replay file.
- **`gripper_target_alignment.py`**: library module with no CLI. It holds the alignment logic used by `fix_gripper_targets`.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `Skipping sample without fresh teleop action: missing` | Teleop is not running, or `teleop_state_file` does not match teleop `--state-file`. |
| `... stale teleop state age=…` | Teleop has stalled or `--state-hz` is too low. Compare with `teleop_state_timeout_sec`. |
| `... missing target_joint_positions or target_tcp_pose` (status `idle`) | The arm is not engaged, because the right grip is released. This is expected between motions, and those ticks are not recorded. |
| `Skipping malformed recorder sample.` | No gripper reading. Teleop runs with `--disable-gripper`, or the gripper has not reported yet, and `use_rtde_reader:=false`. |
| Node exits: `record_hz=… cannot exceed color_fps=…` | Lower `record_hz` or raise `color_fps`. |
| `No RealSense cameras detected` / camera start fails | Check the USB connection and close any other process using the camera, such as `realsense-viewer`. The recorder opens every detected device. |
| `ModuleNotFoundError: rtde_receive` / `pyrealsense2` | `python_executable` points at an interpreter without these packages. Activate the environment before launching, or pass `python_executable:=…`. `rtde_receive` is only imported with `use_rtde_reader:=true`. |
| Save replies `No samples recorded; discarded empty episode` | Every tick was skipped. See the rows above. |
| Start replies `Current episode is stopped. Save or discard it first.` | Call save or discard. |
| Episodes saved in an unexpected directory | `UR_WORKSPACE` is unset, so the launch file used `~/UR/dataset`. Set `UR_WORKSPACE` or pass `dataset_root:=`. |
| Episode folder without `robot_states.json` | The recorder was killed before save. Delete the folder, because the converter rejects it. |
| No `return_home` frames at the end | Check the status counts with `RLDS.py --inspect-only`, and check teleop `--return-home-stop-grace-sec`. |
| Frequent `Recorder slow sample` warnings | Usually the camera wait. Lower `record_hz` or the resolution, and make sure the cameras are on USB 3. |

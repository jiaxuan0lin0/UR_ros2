# policy_client

`policy_client` runs a trained policy on the real UR7e. It is the last stage of the workspace pipeline (`teleop` → `recorder` → `converter` → `policy_client`).

The node reads RealSense images and the UR state over RTDE, sends them to a policy HTTP server through a model-specific adapter, and executes the returned 7-D action chunk with RTDE `servoJ` at 500 Hz. It does not use `ur_robot_driver`. The package itself is model-agnostic. Server commands, payload formats and model quirks are documented in [adapters/](adapters/README.md).

For workspace setup and build, see the [top-level README](../../README.md).

`node.py` wires the node together and runs one control step. Parameters, the start/stop loop, async inference and execution history live in the mixins `node_parameters.py`, `control_loop.py`, `async_policy.py` and `execution_history.py`; log formatting is in `step_summary.py`. The DH gripper driver comes from the `controllers` package, so build that package too (a full `colcon build` does).

## How it works

```text
RealSense RGB + RTDE (q, gripper) -> observation (7-D proprio)
  -> adapter (policy_script) -> HTTP POST /act -> action chunk (N x 7)
  -> first chunk_size actions -> optional RTC blend with the previous chunk
  -> knots every joint_point_duration_sec (30 Hz), starting at the measured q
  -> PCHIP interpolation + per-sample step limit -> RTDE servoJ at 500 Hz
  -> gripper targets -> DH gripper thread on /tmp/ttyUR
```

### 7-D contract

| Field | Shape | Meaning |
| --- | --- | --- |
| `proprio` (observation) | 7 | `[q1..q6, gripper]`: measured joint positions (rad, UR joint order) and gripper position normalized 0–1 |
| `action` | 7 | `[q1..q6, gripper]`: absolute joint targets (rad) and gripper target. Required in every response |
| `action_chunk` | N x 7 | Optional. If absent, `[action]` is used. Row `i` (0-based) is scheduled at `(i + 1) * joint_point_duration_sec` |
| gripper | 1 | Normalized 0–1 by default (`gripper_input_mode=normalized`, clipped). With the default raw range, 0.0 = raw 0 (closed) and 1.0 = raw 1000 (open) |

All values must be finite. Model-specific conversions (for example HiF-VLA's 8-D state) happen inside the adapter.

### Control loop

Each step does the following:

1. Get an action chunk.
2. Keep the first `chunk_size` rows (or only `action` when `use_action_chunk:=false`).
3. Apply the RTC blend.
4. Stream the chunk. The call blocks until the whole chunk has been sent.
5. Store it for the next RTC blend.

The loop runs at `rate_hz`. The default 0.6 Hz ≈ 1.67 s ≈ 50 × 1/30 s. A step that runs longer than the period is followed immediately by the next one. With `stop_on_error` (default) the loop stops at the first failed step.

- **Sync inference.** Read an observation, call `/act`, wait for the reply.
- **Async inference** (`async_policy:=true`, default). While a chunk executes, a side thread samples observations at `history_capture_hz`. After `async_policy_prefetch_after_frames` samples, the next `/act` request is sent in the background. The request uses the latest sample as the observation, the sampled `[q, gripper]` states as `executed_actions`, and the last `max_pending_history_frames` primary images as `history_images`. The next step uses that result instead of calling the server again. If the chunk ends first, the request is sent at chunk end. Sampling, prefetch and history storage only happen with `wait_for_controller_result:=true` (default).
- **RTC blend** (`rtc_control_enabled:=true` in the launch file). Let `h = min(rtc_execution_horizon, len(new chunk), len(previous tail))`. The first `h` rows of the new chunk are blended with the last `h` rows of the previously executed chunk: `a_i = w_i * prev_i + (1 - w_i) * new_i`, on all 7 dimensions. The weights depend on `rtc_transition_schedule`: `exp` uses `exp(-rtc_exp_decay * t)` with `t` going 0→1, so `w_0 = 1`; `linear` uses `1 - t`; `ones` copies the old tail; `zeros` disables the blend. Inference latency is converted to `rtc_delay_steps` for logging only. The RTC state is reset by start, by `reset_policy`, and by `step` (through `reset_policy_on_start`).

### 500 Hz execution

Knot 0 is the measured joint position when streaming starts. Knot `i` is action `i` at `i * joint_point_duration_sec`. The path is sampled every `rtde_servoj_time_sec` (2 ms) with monotone PCHIP or linear interpolation. 50 actions give about 833 samples over 1.667 s.

Each sample is then limited so that the L2 norm of the 6-joint step does not exceed `max_joint_delta`. The default 0.00136 rad per 2 ms is about 0.68 rad/s. Limited samples are not re-timed, so a limited chunk ends short of its last target and the node logs `servoJ trajectory speed-limited`.

Each sample is sent with `servoJ(q, rtde_servoj_max_velocity, rtde_servoj_max_acceleration, t, rtde_servoj_lookahead_time, rtde_servoj_gain)`, paced by `initPeriod`/`waitPeriod`. If `rtde_servoj_time_sec` differs from `1/rtde_servoj_control_frequency` by more than 10 %, `t` is realigned and a warning is logged.

### Gripper bridge

With `direct_gripper:=true`, the node opens and initializes the DH gripper on `gripper_port` **at node startup**. Initialization waits up to `gripper_init_timeout_sec`, then sets speed (`gripper_velocity`, clamped 1–100) and force (`gripper_effort`, clamped 20–100) once.

While streaming, the gripper target follows the action point whose time has been reached. Changes smaller than `gripper_command_deadband` (normalized) are ignored, and the target is mapped to `raw = n * (gripper_raw_max - gripper_raw_min) + gripper_raw_min`. A background thread sends the newest target at `gripper_rate_hz` when it moves by at least `gripper_position_deadband_raw`. The same thread reads the position at `gripper_read_hz`. Both rates are capped at 50 Hz.

The observed gripper value is the DH reading, normalized. Before the first reading, the last commanded target is used instead. Without the direct gripper, the node falls back to RTDE output double register 12 divided by 1000. With `direct_gripper:=false`, gripper targets are ignored and a warning is logged once.

## Quick start

1. Start the policy server (see the adapter doc) and check it. If the server runs on a remote GPU machine, keep an SSH tunnel open in a separate terminal.

```bash
ssh -N -L <POLICY_PORT>:127.0.0.1:<POLICY_PORT> <SSH_USER>@<GPU_SERVER> -p <SSH_PORT>
curl http://127.0.0.1:<POLICY_PORT>/health
```

2. Prepare the robot: remote control, powered on, brakes released, no protective stop. Stop every other control entry. Create `/tmp/ttyUR` for the gripper and keep that terminal open. Skip this step with `direct_gripper:=false`.

```bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py <ROBOT_IP> --device-name /tmp/ttyUR
```

3. Find the camera serials with the Python that will run the node.

```bash
<PYTHON_EXECUTABLE> -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.serial_number)) for d in rs.context().devices]"
```

4. Launch. Run with `dry_run:=true` first, then repeat with `dry_run:=false`. `python_executable` must have `pyrealsense2`, `ur_rtde`, `numpy` and `requests`.

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=<PYTHON_EXECUTABLE> \
  robot_ip:=<ROBOT_IP> \
  policy_base_url:=http://127.0.0.1:<POLICY_PORT> \
  policy_script:=<ADAPTER> \
  instruction:="<TASK_INSTRUCTION>" \
  primary_camera_serial:=<PRIMARY_CAMERA_SERIAL> \
  wrist_camera_serial:=<WRIST_CAMERA_SERIAL> \
  dry_run:=true
```

5. Drive it with the services. `step` runs one chunk and blocks until it has finished; `start` runs continuously.

```bash
ros2 service call /policy_control/step std_srvs/srv/Trigger {}
ros2 service call /policy_control/start std_srvs/srv/Trigger {}
ros2 service call /policy_control/stop std_srvs/srv/Trigger {}
ros2 service call /policy_control/reset_policy std_srvs/srv/Trigger {}
```

Use `ros2 launch policy_client policy_joint_abs_control.launch.py --show-args` to list all arguments, and `ros2 param get /policy_control <name>` to check values. Parameters are read once at startup, so `ros2 param set` has no effect.

## Parameters

Launch arguments of `policy_joint_abs_control.launch.py`. Where the node default differs, it is given in parentheses.

| Argument | Default | Description |
| --- | --- | --- |
| `python_executable` | `$VIRTUAL_ENV/bin/python3`, else `/usr/bin/python3` | Interpreter used to run the node |
| `policy_base_url` | `http://127.0.0.1:8777` | Policy server URL (`http://` is added if missing) |
| `policy_script` | `''` (required) | Adapter: `policy_client.scripts.<name>`, or a full module path if it contains a dot |
| `policy_timeout_sec` | `120.0` (node `30.0`) | Timeout for every HTTP request |
| `instruction` | `''` | Language instruction. `start` fails if empty |
| `autostart` | `false` | Start the loop when the node comes up |
| `dry_run` | `false` | Infer and log actions without commanding the arm |
| `rate_hz` | `0.6` | Policy step rate (not the servo rate) |
| `async_policy` | `true` | Prefetch the next chunk during execution |
| `async_policy_prefetch_after_frames` | `8` | Samples to collect before prefetching (min 1) |
| `async_policy_log_wait_sec` | `0.05` | Log when waiting for a prefetch takes at least this long |
| `robot_ip` | lab IP hard-coded in the launch file | UR controller address. Always pass `robot_ip:=<ROBOT_IP>` |
| `primary_camera_serial` | `''` | Empty = lowest serial among connected cameras |
| `wrist_camera_serial` | `''` | Empty = first other serial |
| `require_wrist_image` | `true` | Read and send a wrist image. `false` = primary only |
| `use_action_chunk` | `true` | Execute `action_chunk`; `false` = only `action` |
| `chunk_size` | `50` | Max rows executed per step |
| `history_capture_hz` | `30.0` | Observation sampling rate during execution; `0` disables sampling |
| `max_pending_history_frames` | `8` | Max history images per request; `0` = no cap (actions are never capped) |
| `joint_point_duration_sec` | `0.03333333333333333` | Time between action knots (30 Hz) |
| `max_joint_delta` | `0.00136` | Max L2 norm (rad) of the 6-joint step per servo sample; `≤ 0` disables |
| `rtde_servoj_time_sec` | `0.002` | `servoJ` `t` |
| `rtde_servoj_control_frequency` | `500.0` | RTDE control/receive frequency and servo rate |
| `rtde_servoj_max_velocity` / `rtde_servoj_max_acceleration` | `1.05` / `1.4` | Passed to `servoJ` |
| `rtde_servoj_lookahead_time` / `rtde_servoj_gain` | `0.06` / `500.0` | Passed to `servoJ` |
| `rtde_servoj_interpolation` | `pchip` | `pchip` or `linear` |
| `rtde_servoj_stop_acceleration` | `2.0` | Declared but unused (`servoStop()` is called without arguments) |
| `rtc_control_enabled` | `true` (node `false`) | Enable the RTC blend |
| `rtc_execution_horizon` | `10` | Rows blended and tail length kept |
| `rtc_transition_schedule` | `exp` | `exp`, `linear`, `ones` or `zeros` |
| `rtc_exp_decay` | `4.0` | Decay for `exp` |
| `gripper_pad_value` | `0.0` | Passed to the adapter (for example HiF-VLA's 8-D pad slot) |
| `direct_gripper` | `true` | Drive the DH gripper on `gripper_port` |
| `gripper_port` / `gripper_baudrate` | `/tmp/ttyUR` / `115200` | DH gripper serial settings |
| `gripper_rate_hz` / `gripper_read_hz` | `30.0` / `30.0` | Command / read rate (capped at 50 Hz) |
| `gripper_velocity` / `gripper_effort` | `100.0` / `100.0` | DH speed / force, set once at init |
| `gripper_command_deadband` | `0.01` | Ignore normalized target changes smaller than this |
| `gripper_position_deadband_raw` | `4.0` | Resend only if the raw target moved at least this much |
| `gripper_init_timeout_sec` | `8.0` | DH initialization timeout |

The launch file also fixes `controller_mode=rtde_servoj_joint_abs`, `joint_command_mode=absolute`, `proprio_mode=joint_abs` and `send_initial_zero_action=false`; these are the only supported values. The following node parameters are not launch arguments. Change them in the launch file, or run the node with `ros2 run --prefix <PYTHON_EXECUTABLE> policy_client policy_control --ros-args -p <name>:=<value>`; in that case node defaults apply to everything else.

| Parameter | Default | Description |
| --- | --- | --- |
| `reset_policy_on_start` | `true` | `POST /reset` on every `start` **and every `step`** |
| `stop_on_error` | `true` | Stop the loop on the first failed step |
| `history_update_mode` | `last_action` | `server_auto` = never send `executed_actions` |
| `wait_for_controller_result` | `true` | Enables sampling, prefetch and history (execution always blocks) |
| `color_width` / `color_height` / `color_fps` | `640` / `480` / `30` | RealSense color stream, applied to all cameras |
| `camera_color_order` | `bgr` | Frame order from the camera; images are sent as RGB |
| `rtde_frequency` | `-1.0` | Observation RTDE receive frequency (`-1` = library default) |
| `gripper_input_mode` | `normalized` | `raw` = policy gripper values are in `[gripper_raw_min, gripper_raw_max]` |
| `gripper_raw_min` / `gripper_raw_max` | `0.0` / `1000.0` | Raw DH range |
| `controller_wait_timeout_sec` / `controller_result_timeout_sec` | `5.0` / `15.0` | Declared but unused |

## Interfaces

| Service (`std_srvs/srv/Trigger`) | Behavior |
| --- | --- |
| `/policy_control/start` | Requires `instruction`. Connects RTDE control (skipped in dry run), calls `/health` and connects RTDE receive and cameras (first time only), resets the policy and RTC, then starts the loop |
| `/policy_control/step` | Connects like `start` (but does not check `instruction`), resets the policy history (default), runs one chunk and returns when it has finished. Refused while the loop runs |
| `/policy_control/stop` | Sets the stop flag, which ends the servoJ stream of a running chunk at the next 2 ms sample and discards any chunk still waiting for inference. Then waits up to 2 s for the loop thread, drops any pending prefetch and calls `servoStop()`. Returns `success=false` if the loop was not running |
| `/policy_control/reset_policy` | Drops any pending prefetch, calls `POST /reset` and resets RTC and history |

HTTP API (JSON over `requests`; `HTTP(S)_PROXY` variables are ignored):

| Endpoint | Request | Response |
| --- | --- | --- |
| `GET /health` | none | Any JSON object (logged) |
| `POST /reset` | no body | Any JSON object |
| `POST /act` | Adapter-defined (see [adapters](adapters/README.md)) | JSON object with `action` (7), optional `action_chunk` (N x 7) and optional `history` (object) |

A non-2xx status, a non-JSON body or a non-object body fails the step. An `error` field in the response is included in the message. The node passes the following to the adapter's `act()`: `instruction`; `observation` (RGB `primary_image`, optional `wrist_image`, 7-D `proprio`, `camera_serials`, and `robot_state` with `joint_positions`, `tcp_pose`, `gripper_raw`, `gripper_position`); `executed_actions` (sampled `[q, gripper]` states from the previous chunk, or `None`); and `history_images` (primary RGB images, or `None`).

## Safety notes

- **One control entry at a time.** RTDE control conflicts with `teleop`, `ur_robot_driver`/`controllers` and a second `policy_client`. Only one process may use `/tmp/ttyUR`.
- **Dry run first.** `dry_run:=true` never sends `servoJ` commands, but it still connects RTDE receive and the cameras. With `direct_gripper:=true` the gripper is opened and initialized at startup, and DH initialization moves the fingers. Use `direct_gripper:=false` for a fully passive check.
- **Limited checks only.** Actions are executed as absolute joint targets after shape and finiteness checks. There is no joint-limit or workspace check. Motion is bounded only by `max_joint_delta` and the `servoJ` settings; keep the defaults until a policy is trusted.
- **Stop is not an emergency stop.** `/policy_control/stop` ends the loop's current chunk within one servo sample and keeps a chunk that arrives later from running, then calls `servoStop()`. A single `/policy_control/step` call is not interrupted: the services share one callback group, so stop runs only after the step returns. Keep the pendant e-stop within reach.
- **Use `step` before `start`.** `autostart` defaults to `false`.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `policy_script parameter must be non-empty` / `No module named 'policy_client.scripts.<name>'` | Set `policy_script` to an existing adapter (see [adapters](adapters/README.md)) |
| `/health` fails or `RemoteDisconnected` | Server process, SSH tunnel, `policy_base_url` |
| `/act` times out | Server load, image/history size, `policy_timeout_sec` |
| `ModuleNotFoundError: pyrealsense2` / `rtde_receive` at startup, or `ur_rtde is not importable` | `python_executable` must point at an environment with these packages |
| `No RealSense cameras detected` / `wrist camera serial not set and only one camera is available` | USB, serials, another process holding the camera, `require_wrist_image` |
| `controller action server not ready: direct_rtde_servoj` | Robot IP, remote control mode, another RTDE client connected |
| RTDE register or script conflicts | Stop `teleop`, `ur_robot_driver` and other RTDE clients (`policy_client` does not use `RTDEIOInterface`) |
| `DH gripper direct tty disabled` | ToolComm terminal running, `/tmp/ttyUR` free, `gripper_baudrate` |
| `servoJ trajectory speed-limited` | Policy asks for faster motion than `max_joint_delta` allows; check the action scale before raising it |
| Robot does not move | `dry_run`, remote control, brakes, protective stop, old processes: `pgrep -af "policy_control\|rtde\|tool_communication"` |

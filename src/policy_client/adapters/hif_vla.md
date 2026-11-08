# HiF-VLA adapter

`policy_script:=hif_vla` connects `policy_client` to the HiF-VLA FastAPI server (`deploy/server.py` in the HiF-VLA repository, outside this workspace). This page covers only the HiF-VLA-specific parts. The robot-side loop, services and safety notes are in the [policy_client README](../README.md), and workspace setup is in the [top-level README](../../../README.md).

| Item | Value |
| --- | --- |
| `policy_script` | `hif_vla` (must be set; the launch default is empty) |
| Module | `policy_client.scripts.hif_vla` |
| State sent | 8-D `[q1, q2, q3, q4, q5, q6, pad, gripper]`, where `pad = gripper_pad_value` |
| Action expected | 7-D `action` `[q1..q6, gripper]`, optional `N x 7` `action_chunk` |
| Image encoding | RGB `uint8` `(H, W, 3)` as nested JSON lists, without compression |

## Quick start

1. On the GPU server, start the HiF-VLA server from its repository. The launch script, its environment variables and the checkpoint selection (`<CHECKPOINT_DIR>`) are defined there, not in this workspace.

```bash
cd <HIF_VLA_REPO>
CUDA_VISIBLE_DEVICES=<GPU_ID> HOST=0.0.0.0 PORT=<POLICY_PORT> bash <SERVER_LAUNCH_SCRIPT>
```

2. On the robot workstation, open a tunnel in its own terminal and check the server.

```bash
ssh -N -L <POLICY_PORT>:127.0.0.1:<POLICY_PORT> <SSH_USER>@<GPU_SERVER> -p <SSH_PORT>
curl http://127.0.0.1:<POLICY_PORT>/health
```

3. Launch `policy_client` (start with `dry_run:=true`).

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=<PYTHON_EXECUTABLE> \
  robot_ip:=<ROBOT_IP> \
  policy_base_url:=http://127.0.0.1:<POLICY_PORT> \
  policy_script:=hif_vla \
  instruction:="<TASK_INSTRUCTION>" \
  primary_camera_serial:=<PRIMARY_CAMERA_SERIAL> \
  wrist_camera_serial:=<WRIST_CAMERA_SERIAL> \
  gripper_pad_value:=0.0 \
  dry_run:=true
```

4. Run one chunk with `/policy_control/step`, then use `/policy_control/start` (see the [policy_client README](../README.md#quick-start)).

## HTTP payload

`POST /act` body built by the adapter:

| Field | Sent when | Content |
| --- | --- | --- |
| `instruction` | always | Task string |
| `full_image` | always | Primary camera image |
| `wrist_image` | the observation has a wrist image (`require_wrist_image:=true`) | Wrist camera image |
| `state` | always | 8-D state (see above) |
| `history_full_images` | the node passes `history_images` (sync and async) | Primary images sampled during the previous chunk, at most `max_pending_history_frames` |
| `executed_actions` | the node passes `executed_actions` | 7-D measured `[q, gripper]` states sampled during the previous chunk |
| `last_action`, `reset` | never in practice | Supported by the adapter, but the node never sets them |

The response must contain a 7-D `action` and may contain an `N x 7` `action_chunk` and a `history` object. The robot executes the first `chunk_size` rows of `action_chunk` (or `action` alone when no chunk is returned).

Other endpoints used: `GET /health`, called once on the first `start`/`step`, and `POST /reset` with no body. `POST /record_action` (`{"actions": [[...7...], ...]}`) is implemented but never called by the node.

The node logs the full `/health` reply. If the server includes its checkpoint path, that path ends up in the robot logs.

## Parameters that matter for HiF-VLA

| Argument | Default | Notes |
| --- | --- | --- |
| `policy_script` | `''` | Set to `hif_vla` |
| `policy_base_url` | `http://127.0.0.1:8777` | Local end of the tunnel |
| `policy_timeout_sec` | `120.0` | JSON image payloads are large; keep this generous |
| `gripper_pad_value` | `0.0` | Value of the 8-D state's pad slot |
| `require_wrist_image` | `true` | `false` drops `wrist_image` from the payload |
| `history_capture_hz` / `max_pending_history_frames` | `30.0` / `8` | Rate and count of `history_full_images` (`0` frames = no cap) |
| `async_policy` | `true` | Prefetch the next chunk while the current one executes |
| `chunk_size` | `50` | Rows executed per chunk |
| `rtc_control_enabled` | `true` | Robot-side chunk blending |

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `/health` fails | Server process, tunnel terminal, port numbers on both ends |
| `/act` times out or waits are long | GPU load, payload size (each 640x480 image is ~0.9 M JSON integers), `max_pending_history_frames`, `policy_timeout_sec` |
| `proprio must have shape (7,)` / `proprio contains non-finite values` | Observation problem on the robot side (RTDE / gripper reading) |
| `action must have shape (7,)` / `action_chunk must have shape (N, 7)` | The server must reply in the 7-D robot convention, not the 8-D state layout |
| Server rejects the state size | `state` is always 8-D; check the server's expected layout and `gripper_pad_value` |

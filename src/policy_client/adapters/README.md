# Policy adapters

An adapter connects `policy_client` to one model server. The robot side always uses the 7-D convention described in the [policy_client README](../README.md). The adapter turns that into the server's HTTP payload and turns the reply back into 7-D actions. Server repos, start commands, payload details and model-side features belong in the per-adapter docs in this directory.

For workspace setup and build, see the [top-level README](../../../README.md).

## How adapters are loaded

- `policy_script:=<name>` imports `policy_client.scripts.<name>`. A name containing a dot is imported as given, so any module on the node's Python path works.
- The module must define `create_policy_client(...)`. The node calls it with exactly three keyword arguments: `base_url` (`policy_base_url`), `timeout_sec` (`policy_timeout_sec`) and `gripper_pad_value`. Accept `**kwargs` for forward compatibility.
- The factory returns a subclass of `policy_client.http_policy.PolicyClient`:

| Method | When the node calls it |
| --- | --- |
| `health() -> dict` | Once, on the first `start` or `step` (the result is logged) |
| `reset() -> dict` | On `start` and `step` (`reset_policy_on_start`, default `true`) and on `/policy_control/reset_policy` |
| `act(*, instruction, observation, reset=False, last_action=None, executed_actions=None, history_images=None) -> PolicyResult` | Every step. The node passes only `instruction`, `observation`, `executed_actions` and `history_images` |
| `record_action(actions) -> dict` | Never. It is abstract, so it must exist, but the node does not call it |

`act()` can run on a background thread (async prefetch), but the node never runs two calls at once.

## What `act()` receives

| Argument | Content |
| --- | --- |
| `instruction` | The `instruction` launch argument |
| `observation.primary_image` | RGB `uint8` array `(H, W, 3)`, 640x480 by default |
| `observation.wrist_image` | Same format, or `None` when `require_wrist_image:=false` |
| `observation.proprio` | 7-D `[q1..q6, gripper]` (rad, gripper normalized 0–1) |
| `observation.robot_state` | `joint_positions`, `tcp_pose`, `gripper_raw`, `gripper_position` (+ `gripper_source` when the DH reading is used) |
| `executed_actions` | `None`, or a list of 7-D `[q, gripper]` **measured states** sampled during the previous chunk |
| `history_images` | `None`, or up to `max_pending_history_frames` primary RGB images from the previous chunk |

`observation.validate()` checks the image shapes and the 7-D proprio.

## What `act()` must return

Return `PolicyResult.from_response(response_dict)`. The dict needs `action` (7 finite floats). It may contain `action_chunk` (N x 7, used when present) and `history` (object). Both must already be in the robot's 7-D convention: absolute joint targets plus gripper. Any conversion from the model's own format happens in the adapter.

`policy_client.http_policy.JsonHttpTransport(base_url, timeout_sec)` covers the plumbing. It provides `health()` (`GET /health`), `reset()` (`POST /reset`), `get_json(path)` and `post_json(path, payload)`. It adds `http://` when the scheme is missing, ignores proxy environment variables, and raises on non-2xx or non-JSON replies.

## Adding an adapter

1. Create `src/policy_client/policy_client/scripts/<adapter_name>.py` and `src/policy_client/adapters/<adapter_name>.md`.
2. Implement the client. Minimal skeleton (images must be encoded the way your server expects):

```python
from policy_client.http_policy import JsonHttpTransport, PolicyClient
from policy_client.types import PolicyResult


class MyPolicyClient(PolicyClient):
    def __init__(self, base_url: str, timeout_sec: float = 30.0):
        self.transport = JsonHttpTransport(base_url, timeout_sec=timeout_sec)

    def health(self):
        return self.transport.health()

    def reset(self):
        return self.transport.reset()

    def act(self, *, instruction, observation, **kwargs):
        observation.validate()
        payload = {
            'instruction': instruction,
            'state': observation.proprio.tolist(),
            # 'image': <encode observation.primary_image for your server>,
        }
        return PolicyResult.from_response(self.transport.post_json('/act', payload))

    def record_action(self, actions):
        return self.transport.post_json('/record_action', {'actions': actions})


def create_policy_client(*, base_url, timeout_sec=30.0, **_kwargs):
    return MyPolicyClient(base_url, timeout_sec=timeout_sec)
```

3. Rebuild `policy_client` (see the [top-level README](../../../README.md)) and launch with `dry_run:=true` first:

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=<PYTHON_EXECUTABLE> \
  robot_ip:=<ROBOT_IP> \
  policy_base_url:=http://<POLICY_HOST>:<POLICY_PORT> \
  policy_script:=<adapter_name> \
  instruction:="<TASK_INSTRUCTION>" \
  dry_run:=true
```

## Registered adapters

| Adapter | Doc | Module | Status |
| --- | --- | --- | --- |
| HiF-VLA | [hif_vla.md](hif_vla.md) | `policy_client.scripts.hif_vla` | Implemented |
| PI0.5 | [pi05.md](pi05.md) | none | Design notes only; no module yet |

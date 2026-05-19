# Policy Adapter 说明

本目录保存具体 policy/model 的接入说明。`policy_client` 主 README 只描述机器人侧通用链路；任何模型仓库路径、云端启动命令、payload 差异、tokenizer/视觉输入约定、RTC server 能力均应写入本目录。

## 1. Adapter 职责

Adapter 位于 Python 模块路径 `policy_client.scripts.<name>`。启动参数 `policy_script:=<name>` 会导入该模块，并调用：

```python
create_policy_client(
    base_url: str,
    timeout_sec: float,
    gripper_pad_value: float,
)
```

返回对象必须实现 `policy_client.http_policy.PolicyClient` 接口：

| 方法 | 作用 |
| --- | --- |
| `health()` | 检查 policy endpoint 状态 |
| `reset()` | 重置 policy server 的 episode/history |
| `act(...)` | 输入图像、proprio、语言指令和历史，返回 `PolicyResult` |
| `record_action(actions)` | 可选记录已经执行的动作历史 |

## 2. 通用 HTTP contract

### `/health`

```bash
curl http://<HOST>:<PORT>/health
```

返回 JSON object。字段由具体模型自定。

### `/reset`

```bash
curl -X POST http://<HOST>:<PORT>/reset
```

返回 JSON object。用于清空服务端历史状态。

### `/act`

请求由 adapter 构造。机器人侧通用输入为：

| 字段 | 语义 |
| --- | --- |
| `instruction` | 语言任务 |
| `observation.primary_image` | 主相机图像 |
| `observation.wrist_image` | 腕部相机图像，可选 |
| `observation.proprio` | 7D `[q1..q6, gripper]` |
| `executed_actions` | 已执行动作历史，可选 |
| `history_images` | 执行期间采集的历史图像，可选 |

响应必须能被解析为：

```json
{
  "action": [0, 0, 0, 0, 0, 0, 0],
  "action_chunk": [
    [0, 0, 0, 0, 0, 0, 0]
  ],
  "history": {}
}
```

`action` 和 `action_chunk` 均为机器人侧 7D convention。具体模型输出不是该 convention 时，转换必须在 adapter 内完成。

## 3. 新 adapter 创建步骤

### 通用文件

```text
src/policy_client/policy_client/scripts/<adapter_name>.py
src/policy_client/adapters/<adapter_name>.md
```

### 通用实现骨架

```python
from policy_client.http_policy import JsonHttpTransport, PolicyClient
from policy_client.types import PolicyObservation, PolicyResult


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
            "instruction": instruction,
            "state": observation.proprio.tolist(),
        }
        response = self.transport.post_json("/act", payload)
        return PolicyResult.from_response(response)

    def record_action(self, actions):
        return self.transport.post_json("/record_action", {"actions": actions})


def create_policy_client(*, base_url, timeout_sec=30.0, **_kwargs):
    return MyPolicyClient(base_url, timeout_sec=timeout_sec)
```

### 通用启动

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  policy_base_url:=http://127.0.0.1:8777 \
  policy_script:=<adapter_name> \
  instruction:="<TASK>"
```

### 示例

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  policy_base_url:=http://127.0.0.1:8777 \
  policy_script:=my_policy_adapter \
  instruction:="perform the demonstrated manipulation task"
```

## 4. 已登记 adapter 文档

| Adapter | 文档 | Python 模块 |
| --- | --- | --- |
| HiF-VLA | [hif_vla.md](hif_vla.md) | `policy_client.scripts.hif_vla` |
| PI0.5 | [pi05.md](pi05.md) | 待 endpoint contract 确认后实现 |

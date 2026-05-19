# HiF-VLA Adapter 使用说明

本文档只描述 HiF-VLA 的 server 启动、payload 约定和 `policy_client` 参数。通用机器人侧启动流程见 [../README.md](../README.md)。

## 1. Adapter 模块

| 项目 | 值 |
| --- | --- |
| `policy_script` | `hif_vla` |
| Python 模块 | `policy_client.scripts.hif_vla` |
| proprio 输入 | 机器人侧 7D 转换为 HiF-VLA 8D |
| 8D state 格式 | `[q1, q2, q3, q4, q5, q6, pad, gripper]` |
| action 输出 | 7D `[q1, q2, q3, q4, q5, q6, gripper]` |

`gripper_pad_value` 用于设置 8D state 的第 7 维 padding，默认 `0.0`。

## 2. 启动 HiF-VLA server

### 通用命令

```bash
cd <HIF_VLA_REPO>
CUDA_VISIBLE_DEVICES=<GPU_ID> \
HOST=0.0.0.0 \
PORT=<POLICY_PORT> \
bash <SERVER_LAUNCH_SCRIPT>
```

### 示例

```bash
cd /root/jiaxuanLin/HIF-VLA
CUDA_VISIBLE_DEVICES=0 \
HOST=0.0.0.0 \
PORT=8777 \
bash deploy/run_ur7e_server.sh
```

## 3. 建立机器人侧 SSH 隧道

### 通用命令

```bash
ssh -N \
  -L <LOCAL_PORT>:127.0.0.1:<REMOTE_POLICY_PORT> \
  <REMOTE_USER>@<REMOTE_HOST> \
  -p <REMOTE_SSH_PORT>
```

### 示例

```bash
ssh -N \
  -L 8777:127.0.0.1:8777 \
  <SSH_USER>@<GPU_SERVER> \
  -p <SSH_PORT>
```

## 4. 健康检查

### 通用命令

```bash
curl http://127.0.0.1:<LOCAL_PORT>/health
```

### 示例

```bash
curl http://127.0.0.1:8777/health
```

## 5. 启动 policy_client

### 通用命令

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=<PYTHON_EXECUTABLE> \
  policy_base_url:=http://127.0.0.1:<LOCAL_PORT> \
  policy_script:=hif_vla \
  instruction:="<TASK_INSTRUCTION>" \
  primary_camera_serial:=<PRIMARY_CAMERA_SERIAL> \
  wrist_camera_serial:=<WRIST_CAMERA_SERIAL> \
  gripper_pad_value:=0.0 \
  rtc_control_enabled:=true
```

### 示例

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=/data/jiaxuanLin/UR/.venv/bin/python3 \
  policy_base_url:=http://127.0.0.1:8777 \
  policy_script:=hif_vla \
  instruction:="pick up all the objects and put them into the basket" \
  primary_camera_serial:=220422302842 \
  wrist_camera_serial:=233522074389 \
  gripper_pad_value:=0.0 \
  rtc_control_enabled:=true
```

## 6. HiF-VLA payload

Adapter 发送到 `/act` 的 JSON 字段：

| 字段 | 来源 | 作用 |
| --- | --- | --- |
| `instruction` | launch 参数 | 语言任务 |
| `full_image` | primary RealSense | 主视角图像 |
| `wrist_image` | wrist RealSense | 腕部图像 |
| `state` | `hif_vla_proprio_from_base()` | 8D state |
| `history_full_images` | 执行期间采集图像 | 异步 policy 历史，可选 |
| `last_action` | 上一次 action | 可选 |
| `executed_actions` | 已执行 action 历史 | 可选 |
| `reset` | reset 请求 | 可选 |

响应必须包含 7D `action`，可包含 `action_chunk`。`action_chunk` 存在时，机器人侧默认执行前 `chunk_size` 个点。

## 7. 参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `policy_script` | `hif_vla` | 选择 HiF-VLA adapter |
| `policy_base_url` | `http://127.0.0.1:8777` | HiF-VLA server 地址 |
| `policy_timeout_sec` | launch 默认 `120.0` | 单次 HTTP 请求超时 |
| `gripper_pad_value` | `0.0` | 8D state 第 7 维 padding |
| `instruction` | 空字符串 | 语言任务 |
| `history_capture_hz` | `30.0` | 执行中采集历史图像/状态的频率 |
| `async_policy` | `true` | 当前 chunk 执行时预取下一次推理 |
| `rtc_control_enabled` | `true` | 启用部署侧 chunk 过渡 |

## 8. 故障处理

| 现象 | 检查项 |
| --- | --- |
| `/health` 无响应 | server 进程、SSH 隧道、端口映射 |
| `/act` 超时 | GPU 负载、`policy_timeout_sec`、图像尺寸 |
| state 维度错误 | `gripper_pad_value` 和 7D/8D 转换 |
| action 维度错误 | server 响应必须为 7D action 或 `N x 7` action chunk |

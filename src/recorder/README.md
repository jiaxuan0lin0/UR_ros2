# Recorder 使用说明

`src/recorder` 是 ROS2 数据采集包。该包同步采集 RealSense 图像、UR observation 和 `teleop.py` 写出的 action state，并保存为 episode 文件夹。该包不控制机械臂，不控制夹爪。

## 1. 数据流

```text
RealSense cameras
UR RTDE state or teleop actual state
/tmp/ur_teleop_state.json
  -> recorder
  -> dataset/episode_*/
      -> cam_*/
      -> robot_states.json
```

## 2. 与 teleop 的关系

| 项目 | 规则 |
| --- | --- |
| 机械臂控制 | 由 `src/teleop/teleop.py` 完成 |
| 夹爪控制 | 由 `src/teleop/teleop.py` 完成 |
| action 来源 | `teleop_state_file` 中的 `target_q`、`target_tcp_pose`、`gripper_target_raw` |
| observation 来源 | 优先使用 teleop state 中的 `actual_q`、`actual_tcp_pose` |
| 落盘频率 | `record_hz`，默认 `30.0` |

推荐频率：

```text
teleop --state-hz 30
teleop --gripper-read-hz 30
recorder record_hz:=30.0
recorder color_fps:=30
```

## 3. 工作站登录

### 通用命令

```bash
ssh <ROBOT_USER>@<ROBOT_HOST> -p <SSH_PORT>
cd <UR_WORKSPACE>
```

### 示例

```bash
ssh <SSH_USER>@<WORKSTATION_HOST> -p <SSH_PORT>
cd /data/jiaxuanLin/UR
```

## 4. 构建 recorder

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
source <PYTHON_VENV>/bin/activate
colcon build --symlink-install --packages-select recorder
source install/setup.bash
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/UR/.venv/bin/activate
colcon build --symlink-install --packages-select recorder
source install/setup.bash
```

## 5. 启动 recorder

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
source <PYTHON_VENV>/bin/activate
source install/setup.bash

ros2 launch recorder record.launch.py \
  robot_ip:=<ROBOT_IP> \
  dataset_root:=<DATASET_ROOT> \
  record_hz:=30.0 \
  color_fps:=30 \
  teleop_state_file:=<TELEOP_STATE_FILE> \
  use_rtde_reader:=false \
  diagnostics_sec:=2.0 \
  sample_warning_sec:=0.04 \
  python_executable:=<PYTHON_EXECUTABLE>
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/UR/.venv/bin/activate
source install/setup.bash

ros2 launch recorder record.launch.py \
  robot_ip:=192.168.1.102 \
  dataset_root:=/data/jiaxuanLin/UR/dataset \
  record_hz:=30.0 \
  color_fps:=30 \
  teleop_state_file:=/tmp/ur_teleop_state.json \
  use_rtde_reader:=false \
  diagnostics_sec:=2.0 \
  sample_warning_sec:=0.04 \
  python_executable:=/data/jiaxuanLin/UR/.venv/bin/python3
```

## 6. 控制 recorder

### 键盘控制

通用命令：

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
source <PYTHON_VENV>/bin/activate
source install/setup.bash
ros2 run recorder keyboard_control
```

示例：

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/UR/.venv/bin/activate
source install/setup.bash
ros2 run recorder keyboard_control
```

键盘：

| 按键 | 作用 |
| --- | --- |
| `r` | start |
| `c` | stop |
| `s` | save |
| `q` | discard |
| `x` | exit |

### Service 控制

```bash
ros2 service call /recorder/start std_srvs/srv/Trigger {}
ros2 service call /recorder/stop std_srvs/srv/Trigger {}
ros2 service call /recorder/save std_srvs/srv/Trigger {}
ros2 service call /recorder/discard std_srvs/srv/Trigger {}
```

`teleop.py --enable-recorder-control` 启用后，也可用 XR 手柄 `A/B/X/Y` 控制 recorder。

## 7. Launch 参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `robot_ip` | `192.168.1.102` | UR 控制器 IP |
| `rtde_frequency` | `-1.0` | RTDE receive 频率；`-1.0` 使用库默认值 |
| `dataset_root` | `$UR_WORKSPACE/dataset` | episode 保存目录 |
| `record_hz` | `30.0` | 最终样本落盘频率 |
| `color_width` | `640` | RealSense 彩色图宽度 |
| `color_height` | `480` | RealSense 彩色图高度 |
| `color_fps` | `30` | RealSense 彩色图 FPS |
| `teleop_state_file` | `/tmp/ur_teleop_state.json` | teleop action state 文件 |
| `teleop_state_timeout_sec` | `1.0` | state file 最大允许年龄 |
| `use_rtde_reader` | `false` | 是否由 recorder 另开 RTDE reader |
| `diagnostics_sec` | `2.0` | 分段耗时诊断输出周期；`0` 表示关闭 |
| `sample_warning_sec` | `0.04` | 单帧采样耗时 warning 阈值 |
| `python_executable` | `$VIRTUAL_ENV/bin/python3` 或 `/usr/bin/python3` | 启动 recorder node 的 Python |

运行中调整频率：

```bash
ros2 param set /recorder record_hz 15.0
```

`record_hz` 不应高于 `color_fps`。`teleop --state-hz` 不应低于 `record_hz`。

## 8. 输出目录

```text
dataset/
  episode_1710000000123/
    cam_0/
      00000.jpg
      00001.jpg
    cam_1/
      00000.jpg
      00001.jpg
    robot_states.json
```

空 episode 在 save 时删除。`discard` 会删除当前 episode。

## 9. `robot_states.json` 格式

```json
{
  "frame_idx": 0,
  "timestamp": 1710000000.123,
  "status": "running",
  "control_mode": "speed_j_dls",
  "observation": {
    "joint_positions": [0, 0, 0, 0, 0, 0],
    "tcp_pose": [0, 0, 0, 0, 0, 0],
    "gripper_position": 0.42
  },
  "action": {
    "joint_positions": [0, 0, 0, 0, 0, 0],
    "tcp_pose": [0, 0, 0, 0, 0, 0],
    "gripper_position": 0.45
  }
}
```

| 字段 | 作用 |
| --- | --- |
| `frame_idx` | episode 内连续帧号 |
| `timestamp` | recorder 采样时间 |
| `status` | teleop 状态，例如 `running`、`return_home`、`replay_wait` |
| `control_mode` | teleop 控制模式 |
| `observation.joint_positions` | 实测关节角 |
| `observation.tcp_pose` | 实测 TCP pose，UR rotvec |
| `observation.gripper_position` | 实测夹爪位置，归一化 |
| `action.joint_positions` | 控制目标关节角 |
| `action.tcp_pose` | 控制目标 TCP pose |
| `action.gripper_position` | 夹爪控制目标 |

夹爪 action 表示控制目标，不保存为下一帧 observation。

## 10. 对齐逻辑

1. recorder 按 `record_hz` 打拍。
2. 每一拍获取 RealSense 帧。
3. 同一拍读取 teleop state。
4. 同一拍读取 RTDE 或使用 teleop actual state。
5. 写入同一个 `frame_idx`。

state file 不存在、过期或缺少 action 字段时，该采样点跳过。

## 11. 检查 episode

### 通用命令

```bash
python3 - <<'PY'
import json
from pathlib import Path

p = Path("<EPISODE_DIR>/robot_states.json")
records = json.loads(p.read_text())
statuses = [record.get("status") for record in records]
print("frames:", len(records))
print("return_home:", sum(status == "return_home" for status in statuses))
print("tail:", statuses[-20:])
PY
```

### 示例

```bash
python3 - <<'PY'
import json
from pathlib import Path

p = Path("/data/jiaxuanLin/UR/dataset/episode_1710000000123/robot_states.json")
records = json.loads(p.read_text())
statuses = [record.get("status") for record in records]
print("frames:", len(records))
print("return_home:", sum(status == "return_home" for status in statuses))
print("tail:", statuses[-20:])
PY
```

## 12. 依赖检查

```bash
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/UR/.venv/bin/activate
source /data/jiaxuanLin/UR/install/setup.bash

python -c "import rclpy, rtde_receive, pyrealsense2; print('recorder deps ok')"
```

## 13. 故障处理

| 现象 | 检查项 |
| --- | --- |
| 没有数据写入 | teleop 是否运行，`teleop_state_file` 是否持续更新 |
| `missing target_joint_positions or target_tcp_pose` | 是否按住 grip 产生有效 action |
| `skipping sample` | state file 过期、缺字段或相机帧未到 |
| 相机打不开 | RealSense USB、序列号、占用进程 |
| 保存后没有 episode | 需要 start、stop、save，且 episode 非空 |
| 回位段缺失 | 检查 status 中是否包含 `return_home`，检查 `return-home-stop-grace-sec` |

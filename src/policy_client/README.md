# UR Policy Client 使用说明

`src/policy_client` 是机器人侧 policy 部署包。该包采集 RealSense 图像和 UR RTDE 状态，调用一个远端或本地 policy HTTP endpoint，并用 RTDE `servoJ` 直接驱动 UR 机械臂。该包不绑定具体模型；模型相关的启动方式、payload 适配和特殊参数统一放在 `src/policy_client/adapters/`。

## 1. 功能边界

### 数据流

```text
RealSense + UR RTDE
  -> policy_client observation
  -> policy adapter
  -> policy HTTP endpoint
  -> 7D action chunk [q1, q2, q3, q4, q5, q6, gripper]
  -> RTC chunk transition
  -> 30 Hz action knots
  -> PCHIP interpolation
  -> 500 Hz RTDE servoJ stream
```

### 统一动作语义

| 字段 | 维度 | 语义 |
| --- | --- | --- |
| `observation.proprio` | 7 | `[q1, q2, q3, q4, q5, q6, gripper]` |
| `action` | 7 | 当前 action，机械臂为 absolute joint target，夹爪为控制目标 |
| `action_chunk` | `N x 7` | 后续一段 absolute joint target chunk |
| `gripper` | 1 | 默认归一化到 `0.0-1.0`，`0.0` 为闭合侧，`1.0` 为张开侧 |

### Adapter 目录

具体模型的 server 启动命令、payload 约定、鉴权方式和特殊 RTC 能力统一放在 [adapters/](adapters/)。主文档只描述机器人侧通用链路。

## 2. 登录机器人侧工作站

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

后续机器人侧命令默认在 `<UR_WORKSPACE>` 下执行。

## 3. 构建 policy_client

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select policy_client
source install/setup.bash
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select policy_client
source install/setup.bash
```

### 检查 launch 参数

```bash
ros2 launch policy_client policy_joint_abs_control.launch.py --show-args
```

## 4. 准备 policy endpoint

`policy_client` 只要求 endpoint 满足 adapter 对应的 HTTP contract。具体模型的启动命令不得写在本文件中；应放在 `src/policy_client/adapters/<adapter>.md`。

### 通用本地 endpoint

```bash
export POLICY_BASE_URL=http://<POLICY_HOST>:<POLICY_PORT>
curl "${POLICY_BASE_URL}/health"
```

### 示例本地 endpoint

```bash
export POLICY_BASE_URL=http://127.0.0.1:8777
curl "${POLICY_BASE_URL}/health"
```

### 通用 SSH 隧道

```bash
ssh -N \
  -L <LOCAL_PORT>:127.0.0.1:<REMOTE_POLICY_PORT> \
  <REMOTE_USER>@<REMOTE_HOST> \
  -p <REMOTE_SSH_PORT>
```

### 示例 SSH 隧道

```bash
ssh -N \
  -L 8777:127.0.0.1:8777 \
  policy_user@policy.example.com \
  -p 22
```

隧道终端需要保持运行。机器人侧 `policy_base_url` 使用 `http://127.0.0.1:<LOCAL_PORT>`。

## 5. 准备机器人和夹爪

机器人需要处于 remote control、已上电、已释放刹车、无急停、无保护停状态。`policy_client` 使用 direct RTDE，不需要启动 `ur_robot_driver`。

### 通用 ToolComm 命令

```bash
source /opt/ros/jazzy/setup.bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py \
  <ROBOT_IP> \
  --device-name <TOOL_TTY>
```

### 示例 ToolComm 命令

```bash
source /opt/ros/jazzy/setup.bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py \
  192.168.1.102 \
  --device-name /tmp/ttyUR
```

没有夹爪或只验证机械臂时，launch 参数设置为：

```text
direct_gripper:=false
```

## 6. 启动 policy_client

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=<PYTHON_EXECUTABLE> \
  policy_base_url:=<POLICY_BASE_URL> \
  policy_script:=<POLICY_ADAPTER_MODULE> \
  instruction:="<TASK_INSTRUCTION>" \
  primary_camera_serial:=<PRIMARY_CAMERA_SERIAL> \
  wrist_camera_serial:=<WRIST_CAMERA_SERIAL> \
  autostart:=false \
  rtc_control_enabled:=true \
  direct_gripper:=true
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch policy_client policy_joint_abs_control.launch.py \
  python_executable:=/data/jiaxuanLin/UR/.venv/bin/python3 \
  policy_base_url:=http://127.0.0.1:8777 \
  policy_script:=<POLICY_ADAPTER_MODULE> \
  instruction:="perform the requested manipulation task" \
  primary_camera_serial:=220422302842 \
  wrist_camera_serial:=233522074389 \
  autostart:=false \
  rtc_control_enabled:=true \
  direct_gripper:=true
```

`policy_script` 的可用值由 adapter 决定。内置 adapter 说明见 `src/policy_client/adapters/`。

## 7. 查询相机序列号

### 通用命令

```bash
<PYTHON_EXECUTABLE> - <<'PY'
import pyrealsense2 as rs

ctx = rs.context()
for dev in ctx.devices:
    print(dev.get_info(rs.camera_info.serial_number))
PY
```

### 示例

```bash
/data/jiaxuanLin/UR/.venv/bin/python3 - <<'PY'
import pyrealsense2 as rs

ctx = rs.context()
for dev in ctx.devices:
    print(dev.get_info(rs.camera_info.serial_number))
PY
```

## 8. 运行控制服务

### 执行一个 action chunk

```bash
ros2 service call /policy_control/step std_srvs/srv/Trigger {}
```

### 连续运行

```bash
ros2 service call /policy_control/start std_srvs/srv/Trigger {}
```

### 停止运行

```bash
ros2 service call /policy_control/stop std_srvs/srv/Trigger {}
```

### 重置 policy 历史

```bash
ros2 service call /policy_control/reset_policy std_srvs/srv/Trigger {}
```

`/policy_control/start` 用于连续真机任务。`/policy_control/step` 只执行一个 chunk，适合联调和动作验证。

## 9. 30 Hz action 与 500 Hz RTDE 控制

policy 输出默认按 `joint_point_duration_sec=1/30` 解释为 30 Hz action knots。RTDE 执行层不会把 30 Hz action 直接阶梯发送给机器人，而是执行以下流程：

1. 将整个 action chunk 放到全局时间轴。
2. 使用 `rtde_servoj_interpolation=pchip` 做单调 cubic Hermite 插值。
3. 按 `rtde_servoj_control_frequency=500.0` 生成 2 ms servoJ target。
4. 通过 `max_joint_delta` 对每个 2 ms step 做关节步长限幅。

50 个 30 Hz action 点对应约 `50 / 30 = 1.6667 s`，500 Hz 下约生成 833 个 servoJ target。

## 10. RTC 控制

`rtc_control_enabled=true` 启用机器人侧 Real-Time Chunking 过渡层。该层在切换到新 action chunk 前，将新 chunk 开头 `rtc_execution_horizon` 帧与上一段已执行 chunk 的尾部做连续化，降低 chunk 边界跳变。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `rtc_control_enabled` | launch 默认 `true`，node 默认 `false` | 是否启用部署侧 RTC 过渡 |
| `rtc_execution_horizon` | `10` | 新旧 chunk 过渡帧数 |
| `rtc_transition_schedule` | `exp` | 过渡权重曲线，支持 `exp`、`linear`、`ones`、`zeros` |
| `rtc_exp_decay` | `4.0` | `exp` 曲线衰减系数 |

模型侧 flow-matching prefix guidance 属于具体模型 server 能力，应在对应 adapter 文档中说明。

## 11. Launch 参数

### Policy 与运行控制

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `python_executable` | `$VIRTUAL_ENV/bin/python3` 或 `/usr/bin/python3` | 启动 node 的 Python |
| `policy_base_url` | `http://127.0.0.1:8777` | policy HTTP endpoint |
| `policy_script` | 空字符串 | 选择具体 policy adapter；启动时显式设置，可用值见 `adapters/` |
| `policy_timeout_sec` | launch `120.0`，node `30.0` | HTTP 请求超时 |
| `instruction` | 空字符串 | 语言任务描述 |
| `autostart` | `false` | node 启动后是否自动开始控制 |
| `dry_run` | `false` | 只推理和构建动作，不发送机器人命令 |
| `rate_hz` | `0.6` | policy chunk 循环频率，不是 RTDE servoJ 频率 |
| `async_policy` | `true` | 当前 chunk 执行期间异步预取下一次推理 |
| `async_policy_prefetch_after_frames` | `8` | 执行观测累计到该帧数后开始预取 |
| `async_policy_log_wait_sec` | `0.05` | 异步等待日志节流时间 |

### 观测与相机

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `robot_ip` | `192.168.1.102` | UR 控制器 IP |
| `primary_camera_serial` | 空字符串 | 主相机序列号；为空时自动选择 |
| `wrist_camera_serial` | 空字符串 | 腕部相机序列号；为空时自动选择 |
| `require_wrist_image` | `true` | 是否要求腕部图像存在 |
| `color_width` | node 默认 `640` | 彩色图宽度 |
| `color_height` | node 默认 `480` | 彩色图高度 |
| `color_fps` | node 默认 `30` | 相机 FPS |
| `camera_color_order` | node 默认 `bgr` | 图像通道顺序 |
| `rtde_frequency` | node 默认 `-1.0` | RTDE receive 频率；`-1.0` 使用库默认值 |
| `proprio_mode` | `joint_abs` | proprio 表示，当前部署使用 joint absolute |
| `gripper_pad_value` | `0.0` | 供需要 8D proprio 的 adapter 插入 padding |

### Action chunk 与 500 Hz 执行

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `use_action_chunk` | `true` | 是否执行 `action_chunk`；否则只执行单步 `action` |
| `chunk_size` | `50` | 每次最多执行的 action 点数 |
| `history_capture_hz` | `30.0` | 执行期间写入 policy 历史的观测采样频率 |
| `max_pending_history_frames` | node 默认 `8` | 最多缓存历史帧数 |
| `joint_point_duration_sec` | `0.03333333333333333` | 相邻 action knot 时间间隔 |
| `max_joint_delta` | `0.00136` | 每个 2 ms servo sample 的关节步长限制 |
| `rtde_servoj_time_sec` | `0.002` | `servoJ` t 参数 |
| `rtde_servoj_control_frequency` | `500.0` | RTDE control interface 频率 |
| `rtde_servoj_max_velocity` | `1.05` | `servoJ` 速度参数 |
| `rtde_servoj_max_acceleration` | `1.4` | `servoJ` 加速度参数 |
| `rtde_servoj_lookahead_time` | `0.06` | `servoJ` lookahead time |
| `rtde_servoj_gain` | `500.0` | `servoJ` gain |
| `rtde_servoj_interpolation` | `pchip` | 30 Hz action 到 500 Hz servo target 的插值方式 |
| `rtde_servoj_stop_acceleration` | `2.0` | 停止 servo 时使用的加速度 |

### 夹爪

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `direct_gripper` | `true` | 是否直接通过 `/tmp/ttyUR` 控制 DH 夹爪 |
| `gripper_port` | `/tmp/ttyUR` | DH 夹爪串口 |
| `gripper_baudrate` | `115200` | DH 夹爪波特率 |
| `gripper_rate_hz` | `30.0` | 夹爪命令线程频率；代码限制不超过 50 Hz |
| `gripper_read_hz` | `30.0` | 夹爪当前位置读取频率；代码限制不超过 50 Hz |
| `gripper_input_mode` | node 默认 `normalized` | policy 输出夹爪值格式，支持 `normalized` 和 `raw` |
| `gripper_raw_min` | node 默认 `0.0` | raw 闭合侧或最小值 |
| `gripper_raw_max` | node 默认 `1000.0` | raw 张开侧或最大值 |
| `gripper_velocity` | `100.0` | DH 夹爪速度 |
| `gripper_effort` | `100.0` | DH 夹爪力 |
| `gripper_command_deadband` | `0.01` | 归一化夹爪目标变化死区 |
| `gripper_position_deadband_raw` | `4.0` | raw 目标重复发送死区 |
| `gripper_init_timeout_sec` | `8.0` | 夹爪初始化超时 |

### Controller 行为

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `controller_mode` | 固定 `rtde_servoj_joint_abs` | 控制器模式 |
| `joint_command_mode` | 固定 `absolute` | joint action 语义 |
| `controller_wait_timeout_sec` | node 默认 `5.0` | 等待控制器 ready 超时 |
| `controller_result_timeout_sec` | node 默认 `15.0` | 等待控制结果超时 |
| `wait_for_controller_result` | node 默认 `true` | 是否等待当前 chunk 执行完成 |

## 12. 检查命令

```bash
ros2 param get /policy_control policy_base_url
ros2 param get /policy_control policy_script
ros2 param get /policy_control chunk_size
ros2 param get /policy_control joint_point_duration_sec
ros2 param get /policy_control rtde_servoj_control_frequency
ros2 param get /policy_control rtde_servoj_interpolation
ros2 param get /policy_control rtc_control_enabled
ros2 param get /policy_control direct_gripper
```

## 13. 故障处理

| 现象 | 检查项 |
| --- | --- |
| `/health` 不通 | 检查 `policy_base_url`、SSH 隧道、policy server 进程 |
| `No RealSense cameras detected` | 检查 USB、相机序列号、是否被其他进程占用 |
| `ur_rtde is not importable` | `python_executable` 需要指向安装了 `rtde_control`、`rtde_receive` 的环境 |
| 夹爪不可用 | 检查 ToolComm 终端、`/tmp/ttyUR`、`direct_gripper` |
| RTDE IO input register 被占用 | 当前 direct servoJ 方案不依赖 RTDE IO input register |
| 机器人不动 | 检查 remote control、上电、刹车、保护停、旧控制进程 |

```bash
pgrep -af "policy_control|rtde|tool_communication"
```

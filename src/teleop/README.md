# Teleop

`src/teleop` 是当前推荐的高性能遥操作链路。它不经过 MoveIt Servo、`ros2_control` 或 ROS2 action，而是直接用 XR 输入生成目标位姿，再通过 RTDE 控制 UR 机械臂，同时直接通过 `/tmp/ttyUR` 控制 DH 夹爪。

## 数据流

```text
XRoboToolkit PC Service
  -> xrobotoolkit_sdk
  -> teleop.py
      -> RTDE servoJ / servoL 控制 UR
      -> /tmp/ttyUR Modbus 控制 DH 夹爪
      -> /tmp/ur_teleop_state.json 写出当前 action
  -> recorder 读取 action 并同步相机/机械臂 observation
```

`teleop.py` 写出的 state file 只作为 recorder 的动作源，不是最终训练数据。最终数据由 `recorder` 写成 `observation/action` 结构。

## 启动前要求

- 机器人 IP 可达，默认 `192.168.1.102`。
- XRoboToolkit PC Service 已启动，且当前 Python 环境可以 import `xrobotoolkit_sdk`。
- 如果使用 DH 夹爪，机器人端 ToolComm Forwarder URCap 需要运行，PC 端需要先创建 `/tmp/ttyUR`。
- 不要同时启动 `ur_control.launch.py`、MoveIt Servo 或 ROS2 `gripper` 节点来控制同一台机械臂/夹爪。

## 启动 XRoboToolkit PC Service

这台机器上已经安装了 XRoboToolkit PC Service，安装路径是：

```text
/opt/apps/roboticsservice
```

命令行启动服务：

```bash
/opt/apps/roboticsservice/runService.sh
```

也可以从桌面应用里启动：

```text
XRoboToolkit-PC-Service
```

检查服务是否已经运行：

```bash
pgrep -af RoboticsServiceProcess
```

如果能看到类似下面的进程，就说明 PC Service 已经起来了：

```text
./RoboticsServiceProcess
```

XR 头显/手柄端还需要打开 XRoboToolkit 的 Quest/PICO 客户端，并连接到这台 PC。PC Service 是本机服务，`xrobotoolkit_sdk` 会通过它读取手柄姿态、trigger、grip、按钮等数据。

**坐标对齐要求：Quest/PICO 里启动 XRoboToolkit 客户端时，软件界面的正前方应当朝向机械臂 base 坐标系的正 X 方向。** 这个初始朝向会影响手柄坐标到机器人 base 坐标的映射；如果朝向不对，就容易出现左右、前后不对齐，或者手柄直线移动但机械臂斜着走。

## Python 环境

这台机器上 `xrobotoolkit_sdk` 装在 conda 环境 `XRobo` 里。启动 teleop 前建议这样进入环境：

```bash
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
```

检查 SDK 是否可 import：

```bash
python -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

预期输出类似：

```text
/home/ustc/anaconda3/envs/XRobo/lib/python3.11/site-packages/xrobotoolkit_sdk.cpython-311-x86_64-linux-gnu.so
```

检查 SDK 和 PC Service 是否能连接：

```bash
python -c "import xrobotoolkit_sdk as xrt; xrt.init(); print(xrt.get_time_stamp_ns()); xrt.close()"
```

如果输出时间戳为 `0`，通常表示 SDK 已连上 PC Service，但当前头显/手柄数据还没有进入服务；打开并连接 XR 客户端后会变成有效时间戳。

## 推荐启动

先让机械臂进入可 RTDE 控制状态。示教器需要切到 `Remote Control`，然后通过 Dashboard Server 上电并释放刹车：

```bash
cd /home/ustc/UR
python3 src/teleop/start_ur.py --robot-ip 192.168.1.102
```

这一步不是启动 `ur_robot_driver`。`teleop.py` 启动时会通过 `RTDEControlInterface` 连接机器人并启动 RTDE 控制脚本。

如果机器人处于 `PROTECTIVE_STOP`，脚本默认不会自动解锁。确认现场安全后再显式执行：

```bash
python3 src/teleop/start_ur.py \
  --robot-ip 192.168.1.102 \
  --unlock-protective-stop
```

只想检查当前 dashboard 状态，不上电、不松刹车：

```bash
python3 src/teleop/start_ur.py \
  --robot-ip 192.168.1.102 \
  --skip-power-on \
  --skip-brake-release \
  --no-wait-running
```

先创建 `/tmp/ttyUR`：

```bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py \
  192.168.1.102 \
  --device-name /tmp/ttyUR
```

启动 teleop：

```bash
cd /home/ustc/UR
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
source /opt/ros/jazzy/setup.bash
source /home/ustc/UR/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip 192.168.1.102 \
  --control-mode servo_j \
  --rate-hz 100 \
  --max-velocity 0.08 \
  --max-acceleration 0.15 \
  --lookahead-time 0.12 \
  --servo-gain 200 \
  --scale-factor 0.8 \
  --rotation-scale 0.8 \
  --max-step-translation 0.001 \
  --max-step-rotation 0.005 \
  --max-joint-step 0.003 \
  --gripper-port /tmp/ttyUR \
  --state-file /tmp/ur_teleop_state.json \
  --enable-recorder-control \
  --return-home-max-joint-step 0.0006 \
  --return-home-timeout-sec 30
```

## 遥操作按键

默认控制：

- 右手 `grip`：按住才移动机械臂。
- 右手 `trigger`：控制夹爪开合。
- 右摇杆按下：急停 teleop。

开启 `--enable-recorder-control` 后，默认录制控制：

- `A`：调用 `/recorder/start`
- `B`：先自动回到 home 位姿，这段回位继续被 recorder 记录；到位后再调用 `/recorder/stop`
- `X`：调用 `/recorder/save`
- `Y`：调用 `/recorder/discard`

可以改按键：

```bash
python3 src/teleop/teleop.py \
  --enable-recorder-control \
  --recorder-start-button A \
  --recorder-stop-button B \
  --recorder-save-button X \
  --recorder-discard-button Y
```

按键触发使用上升沿和 debounce，不会因为长按反复调用 service。

默认 home 位姿是当前采集流程使用的结束位：

```text
[-1.5708139578448694, -1.5707999668517054, -1.570805549621582,
 -1.570810934106344, 1.5707950592041016, -0.02949411073793584]
```

如果临时想恢复旧行为，让 `B` 直接 stop，不自动回位：

```bash
python3 src/teleop/teleop.py \
  --enable-recorder-control \
  --disable-return-home-before-stop
```

## 完整参数说明

### 机械臂 RTDE 参数

- `--robot-ip`：UR 控制器 IP，默认 `192.168.1.102`。
- `--control-mode`：控制模式，默认 `servo_j`。
  - `servo_j`：teleop 先根据目标 TCP 位姿做 IK，再用 `servoJ` 发送关节目标。推荐用于当前采集流程，关节 action 更稳定。
  - `servo_l`：直接用 `servoL` 发送 TCP 位姿目标。TCP 跟随直观，但对轨迹连续性和奇异位姿更敏感。
- `--rate-hz`：主控制循环频率，默认 `250 Hz`。值越高，轨迹越细，但对网络、Python 调度和 RTDE 稳定性要求越高。调试/采集建议先用 `100 Hz`。
- `--max-velocity`：传给 UR `servoJ/servoL` 的速度参数，单位按 RTDE 接口定义。它不是最终运动速度上限，实际速度还会被 step 限幅影响。
- `--max-acceleration`：传给 UR `servoJ/servoL` 的加速度参数，单位按 RTDE 接口定义。
- `--lookahead-time`：UR servo 的前瞻时间，默认 `0.06 s`。大一些会更平滑、更慢一些；小一些响应更直接，但更容易抖。
- `--servo-gain`：UR servo 增益，默认 `500`。大一些跟随更紧，小一些更柔和。现在样例用 `200`，适合先把遥操作调稳。

### XR 输入和映射参数

启动 Quest/PICO 客户端时，请先确认软件界面的正前方朝向机械臂正 X 方向，再开始按住 grip 建立遥操作参考点。

- `--pose-source`：使用哪个 XR 位姿源，默认 `right_controller`。可选通常是 `right_controller`、`left_controller`、`headset`。
- `--grip-name`：机械臂运动使能的模拟量，默认 `right_grip`。只有 grip 值超过阈值才移动机械臂。
- `--trigger-name`：夹爪控制的模拟量，默认 `right_trigger`。松开为张开，越按越夹紧。
- `--emergency-button`：急停 teleop 的按钮，默认 `right_axis_click`。
- `--grip-threshold`：运动使能阈值，默认 `0.95`。如果手柄 grip 不容易按满，可以降低到 `0.7` 或 `0.8`。
- `--xr-timeout-sec`：XR 数据超时时间，默认 `0.2 s`。超过这个时间没有新数据时，teleop 会停止继续发送运动目标。
- `--scale-factor`：手柄平移到机器人 TCP 平移的比例，默认 `0.6`。例如 `0.8` 表示手柄移动 `10 cm`，目标 TCP 大约移动 `8 cm`。
- `--rotation-scale`：手柄旋转到 TCP 旋转的比例，默认 `1.0`。设为 `0` 可以完全关闭手腕旋转，只保留平移。

### 安全限幅参数

- `--max-step-translation`：每个控制周期允许 TCP 目标最多平移多少，单位 `m`，默认 `0.01`。这是最直接的速度限制之一。样例 `0.001` 在 `100 Hz` 下理论上约等于 `0.1 m/s` 的目标变化上限。
- `--max-step-rotation`：每个控制周期允许 TCP 目标最多旋转多少，单位 `rad`，默认 `0.05`。如果感觉末端翻转太快，优先降低它或降低 `--rotation-scale`。
- `--max-joint-step`：每个控制周期允许单个关节目标最多变化多少，单位 `rad`，默认 `0.04`。这是 `servo_j` 下限制关节跳变的关键参数。样例 `0.003` 比较保守。
- `--workspace-min x,y,z`：可选 TCP 工作空间下边界，单位 `m`，例如 `--workspace-min -0.5,-0.7,0.05`。
- `--workspace-max x,y,z`：可选 TCP 工作空间上边界，单位 `m`，例如 `--workspace-max 0.5,-0.2,0.6`。

### DH 夹爪参数

- `--disable-gripper`：禁用夹爪，只控制机械臂。
- `--gripper-port`：夹爪串口，默认 `/tmp/ttyUR`。当前链路里它来自 ToolComm Forwarder。
- `--gripper-baudrate`：夹爪串口波特率，默认 `115200`。
- `--gripper-rate-hz`：夹爪命令发送线程频率，默认 `30 Hz`。这个不用跟机械臂一样高。
- `--gripper-speed`：DH 夹爪目标速度，范围 `1-100`，默认 `100`。
- `--gripper-force`：DH 夹爪目标力，范围 `20-100`，默认 `50`。
- `--gripper-open-raw`：张开位置原始值，默认 `1000`。
- `--gripper-closed-raw`：夹住位置原始值，默认 `0`。
- `--gripper-binary-control`：把 trigger 从连续控制改成二值控制。不开这个参数时，trigger 是连续数值。
- `--gripper-trigger-threshold`：二值夹爪的触发阈值，默认 `0.5`。只在 `--gripper-binary-control` 开启时生效。
- `--gripper-deadband`：夹爪目标变化死区，默认 `4`。目标 raw 变化小于这个值时不重复发命令，减少串口压力。
- `--gripper-init-timeout-sec`：等待 DH 夹爪初始化完成的最长时间，默认 `8 s`。
- `--gripper-read-hz`：读取夹爪当前位置的频率，默认 `5 Hz`。读取值会写入 state file，供 recorder 生成夹爪 observation/action。

### State File 参数

- `--disable-state-file`：不写 teleop action 状态文件。开启后 recorder 无法记录 teleop 的 action。
- `--state-file`：teleop action 状态文件路径，默认 `/tmp/ur_teleop_state.json`。
- `--state-hz`：写状态文件频率，默认 `30 Hz`。这不影响机械臂控制频率，只影响 recorder 能拿到 action 的刷新频率。

### Recorder 控制参数

- `--enable-recorder-control`：允许 teleop 用手柄按钮调用 recorder 的 ROS2 service。不开这个参数时，A/B/X/Y 不控制 recorder。
- `--recorder-start-button`：开始录制按钮，默认 `A`。
- `--recorder-stop-button`：停止录制按钮，默认 `B`。默认行为是先回 home，再 stop。
- `--recorder-save-button`：保存 episode 按钮，默认 `X`。
- `--recorder-discard-button`：丢弃 episode 按钮，默认 `Y`。
- `--recorder-debounce-sec`：按钮防抖时间，默认 `0.5 s`，避免长按触发多次 service。

### Stop 前自动回位参数

- `--disable-return-home-before-stop`：关闭 stop 前自动回位。开启后，按 `B` 会直接请求 `/recorder/stop`。
- `--return-home-joints q1,q2,q3,q4,q5,q6`：`B` 键 stop 前自动回到的 joint-space home 位姿，单位 `rad`。不传则使用代码里的默认采集结束位。
- `--return-home-max-joint-step`：自动回位时每个控制周期允许单个关节最多变化多少，单位 `rad`，默认 `0.0006`。它只影响 B 键回 home，不影响正常遥操作的 `--max-joint-step`。
- `--return-home-tolerance`：到位判断的最大关节误差，单位 `rad`，默认 `0.01`。
- `--return-home-timeout-sec`：回位最长等待时间，默认 `20 s`。如果把 `--return-home-max-joint-step` 调得很小，建议设到 `30 s` 或更高；超时也会请求 recorder stop。
- `--return-home-hold-sec`：到位后保持多久再 stop recorder，默认 `0.3 s`。这段保持也会继续被 recorder 记录。

## 推荐样例

当前建议先用下面这一组平稳采集参数。它的思路是：`servo_j` 直控，`100 Hz` 主循环，速度/加速度和单周期步长都压低，先保证方向、手感和数据质量稳定。

```bash
cd /home/ustc/UR
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
source /opt/ros/jazzy/setup.bash
source /home/ustc/UR/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip 192.168.1.102 \
  --control-mode servo_j \
  --rate-hz 100 \
  --max-velocity 0.08 \
  --max-acceleration 0.15 \
  --lookahead-time 0.12 \
  --servo-gain 200 \
  --scale-factor 0.8 \
  --rotation-scale 0.8 \
  --max-step-translation 0.001 \
  --max-step-rotation 0.005 \
  --max-joint-step 0.003 \
  --gripper-port /tmp/ttyUR \
  --state-file /tmp/ur_teleop_state.json \
  --enable-recorder-control \
  --return-home-max-joint-step 0.0006 \
  --return-home-timeout-sec 30
```

如果只想做平移采集，不想让手腕姿态影响末端旋转，把 `--rotation-scale` 改成 `0`：

```bash
python3 src/teleop/teleop.py \
  --robot-ip 192.168.1.102 \
  --control-mode servo_j \
  --rate-hz 100 \
  --max-velocity 0.08 \
  --max-acceleration 0.15 \
  --lookahead-time 0.12 \
  --servo-gain 200 \
  --scale-factor 0.8 \
  --rotation-scale 0 \
  --max-step-translation 0.001 \
  --max-step-rotation 0.005 \
  --max-joint-step 0.003 \
  --gripper-port /tmp/ttyUR \
  --state-file /tmp/ur_teleop_state.json \
  --enable-recorder-control \
  --return-home-max-joint-step 0.0006 \
  --return-home-timeout-sec 30
```

## 与 recorder 的关系

teleop 的 state file 包含：

```json
{
  "wall_time": 1710000000.123,
  "gripper_target_raw": 450,
  "gripper_current_raw": 430,
  "target_tcp_pose": [x, y, z, rx, ry, rz],
  "target_q": [q0, q1, q2, q3, q4, q5]
}
```

recorder 读取这个文件后生成最终训练数据：

```json
{
  "frame_idx": 0,
  "timestamp": 1710000000.123,
  "observation": {
    "joint_positions": [...],
    "tcp_pose": [...],
    "gripper_position": 0.43
  },
  "action": {
    "joint_positions": [...],
    "tcp_pose": [...],
    "gripper_position": 0.45
  }
}
```

如果 state file 不存在、不新鲜，或者缺少 `target_q/target_tcp_pose`，recorder 会跳过该帧，避免把无效 action 写进数据集。

## 常见问题

`xrobotoolkit_sdk is not importable`：先 `conda activate XRobo`，再用 `python -c "import xrobotoolkit_sdk"` 检查。

`xrt.init()` 能运行但时间戳一直是 `0`：PC Service 通了，但 XR 客户端还没有连接或没有数据流。打开头显端 XRoboToolkit 客户端并连接 PC。

`RoboticsServiceProcess` 不存在：运行 `/opt/apps/roboticsservice/runService.sh` 启动 PC Service。

`/tmp/ttyUR` 不存在：先运行 ToolComm Forwarder 的 PC 端脚本，并确认机器人端 URCap 已启动。

夹爪控制异常：确认没有同时运行 ROS2 `gripper` 节点，因为它也会占用 `/tmp/ttyUR`。

recorder 按键无反应：确认 recorder 已启动，且 teleop 启动时加了 `--enable-recorder-control`。

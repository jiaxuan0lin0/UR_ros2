# Teleop 使用说明

`src/teleop` 是 XR 遥操作采集链路。它直接读取 XRoboToolkit 输入，通过 RTDE 控制 UR 机械臂，并通过 `/tmp/ttyUR` 控制 DH 夹爪。采集时，`teleop.py` 同步写出 state file，`recorder` 读取该文件生成训练数据。

## 1. 数据流

```text
XR headset/controller
  -> XRoboToolkit PC Service
  -> xrobotoolkit_sdk
  -> src/teleop/teleop.py
      -> RTDE speedJ / servoJ / servoL
      -> /tmp/ttyUR DH gripper
      -> /tmp/ur_teleop_state.json
  -> src/recorder reads state file
  -> recorder episode
```

## 2. 控制入口规则

同一时间只能保留一个机械臂控制入口。

| 入口 | 规则 |
| --- | --- |
| `teleop.py` | 推荐用于 XR 数据采集 |
| `policy_client` | 推荐用于 policy 部署 |
| `controllers` | 用于 ROS2 action/topic 调试 |
| `ur_robot_driver` + MoveIt Servo | 不与 direct RTDE teleop 同时运行 |

`/tmp/ttyUR` 同一时间只能被一个夹爪进程占用。运行 `teleop.py` 时，不启动 `controllers/gripper`。

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

## 4. 低延迟环境检查

500 Hz 遥操作依赖稳定的 PC 调度、网络和 RTDE 周期。推荐使用 lowlatency kernel、`performance` CPU governor 和实时优先级。

### 通用命令

```bash
uname -r
groups
ulimit -r
ulimit -l
systemctl status ur-performance-governor.service --no-pager
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  cat "$f"
done | sort | uniq -c
```

### 示例期望输出

```text
6.8.0-111-lowlatency
groups 中包含 realtime
ulimit -r 输出 99
ulimit -l 输出 unlimited
CPU governor 为 performance
```

## 5. 启动 XRoboToolkit PC Service

### 通用命令

```bash
cd <UR_WORKSPACE>
src/teleop/start_xr_service_rt.sh
```

### 示例

```bash
cd /data/jiaxuanLin/UR
src/teleop/start_xr_service_rt.sh
```

### 重启服务

```bash
cd /data/jiaxuanLin/UR
src/teleop/start_xr_service_rt.sh --restart
```

### 检查服务

```bash
ps -o pid,pri,rtprio,cls,ni,cmd -p "$(pgrep -d, -x RoboticsService)"
```

XR 客户端启动时，软件正前方应朝向机器人 base 坐标系 `+X` 方向。该朝向决定手柄坐标到机器人 base 坐标的映射。

## 6. Python 环境

### 通用命令

```bash
source <CONDA_ROOT>/etc/profile.d/conda.sh
conda activate <XR_ENV_NAME>
python -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

### 示例

```bash
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
python -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

### 检查 XR 时间戳

```bash
python -c "import xrobotoolkit_sdk as xrt; xrt.init(); print(xrt.get_time_stamp_ns()); xrt.close()"
```

时间戳为 `0` 表示 PC Service 已连接但 XR 客户端尚未发送有效 tracking 数据。

## 7. XR 链路诊断

### 通用命令

```bash
cd <UR_WORKSPACE>
source <CONDA_ROOT>/etc/profile.d/conda.sh
conda activate <XR_ENV_NAME>
python3 src/teleop/diagnose_xr_stream.py \
  --duration-sec 30 \
  --poll-hz 1000
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
python3 src/teleop/diagnose_xr_stream.py \
  --duration-sec 30 \
  --poll-hz 1000
```

| 输出项 | 含义 |
| --- | --- |
| `Effective update rate` | XR 源头有效更新率 |
| `p95/p99/max` | XR 数据间隔分布 |
| `Large gaps` | 大于 100 ms 或 200 ms 的链路断续次数 |

`--rate-hz 500` 是机器人控制环重采样频率，不代表 XR 手柄源头能达到 500 Hz。

## 8. 准备机器人

### 上电和释放刹车

通用命令：

```bash
cd <UR_WORKSPACE>
python3 src/teleop/start_ur.py --robot-ip <ROBOT_IP>
```

示例：

```bash
cd /data/jiaxuanLin/UR
python3 src/teleop/start_ur.py --robot-ip 192.168.1.102
```

### 保护停解锁

确认现场安全后使用：

```bash
python3 src/teleop/start_ur.py \
  --robot-ip 192.168.1.102 \
  --unlock-protective-stop
```

### 只检查 Dashboard 状态

```bash
python3 src/teleop/start_ur.py \
  --robot-ip 192.168.1.102 \
  --skip-power-on \
  --skip-brake-release \
  --no-wait-running
```

## 9. 创建夹爪串口

### 通用命令

```bash
source /opt/ros/jazzy/setup.bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py \
  <ROBOT_IP> \
  --device-name <TOOL_TTY>
```

### 示例

```bash
source /opt/ros/jazzy/setup.bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py \
  192.168.1.102 \
  --device-name /tmp/ttyUR
```

该终端需要保持运行。禁用夹爪时，在 teleop 命令中加入 `--disable-gripper`。

## 10. 启动 teleop

完整推荐命令见 [cmd.md](cmd.md)。

### 通用命令

```bash
cd <UR_WORKSPACE>
source <CONDA_ROOT>/etc/profile.d/conda.sh
conda activate <XR_ENV_NAME>
source /opt/ros/jazzy/setup.bash
source <UR_WORKSPACE>/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip <ROBOT_IP> \
  --ur-type <UR_TYPE> \
  --control-mode speed_j_dls \
  --speedj-backend register_bridge \
  --rate-hz 500 \
  --gripper-port <TOOL_TTY> \
  --gripper-rate-hz 30 \
  --gripper-read-hz 30 \
  --state-file <TELEOP_STATE_FILE> \
  --state-hz 30 \
  --enable-recorder-control
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/UR/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip 192.168.1.102 \
  --ur-type ur7e \
  --control-mode speed_j_dls \
  --speedj-backend register_bridge \
  --rate-hz 500 \
  --gripper-port /tmp/ttyUR \
  --gripper-rate-hz 30 \
  --gripper-read-hz 30 \
  --state-file /tmp/ur_teleop_state.json \
  --state-hz 30 \
  --enable-recorder-control
```

## 11. 按键

| 控件 | 作用 |
| --- | --- |
| 右手 `grip` | 按住后机械臂跟随 |
| 右手 `trigger` | 控制夹爪开合 |
| 右摇杆按下 | 急停 teleop |
| `A` | recorder start |
| `B` | 回 home 后 recorder stop |
| `X` | recorder save |
| `Y` | recorder discard |
| 左手 `grip` | 回位后 replay 触发，需启用 `--replay-after-return-home` |

## 12. 频率约定

| 项目 | 默认值 | 说明 |
| --- | --- | --- |
| `--rate-hz` | `500` | 机械臂主控制循环 |
| `--state-hz` | `30` | 写 `/tmp/ur_teleop_state.json` |
| `--gripper-rate-hz` | `30` | DH 夹爪命令线程 |
| `--gripper-read-hz` | `30` | DH 夹爪位置读取线程 |
| recorder `record_hz` | `30` | 最终落盘频率 |

AG95 通信命令不按 500 Hz 运行。代码会把夹爪命令和读取频率限制到不超过 50 Hz。

## 13. State file 格式

`teleop.py` 按 `--state-hz` 写出 JSON。recorder 用该文件生成 action，并优先使用其中的 actual state 作为 observation。

```json
{
  "wall_time": 1710000000.123,
  "status": "running",
  "control_mode": "speed_j_dls",
  "gripper_target_raw": 450,
  "gripper_current_raw": 430,
  "motion_grip_value": 0.97,
  "motion_enabled": true,
  "xr_age_sec": 0.0,
  "actual_tcp_pose": [0, 0, 0, 0, 0, 0],
  "actual_q": [0, 0, 0, 0, 0, 0],
  "target_tcp_pose": [0, 0, 0, 0, 0, 0],
  "target_q": [0, 0, 0, 0, 0, 0]
}
```

| 字段 | 作用 |
| --- | --- |
| `actual_q` | 实测关节角，作为 observation |
| `actual_tcp_pose` | 实测 TCP pose，作为 observation |
| `target_q` | 当前控制目标，作为 action |
| `target_tcp_pose` | 当前 TCP 目标，作为 action |
| `gripper_target_raw` | 夹爪控制目标 |
| `gripper_current_raw` | 夹爪实测位置 |

## 14. 参数说明

### 机械臂 RTDE

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--robot-ip` | `192.168.1.102` | UR 控制器 IP |
| `--ur-type` | `ur7e` | URDF/Jacobian 使用的机器人型号 |
| `--control-mode` | `servo_j` | 控制模式：`speed_j_dls`、`servo_j`、`servo_l` |
| `--rate-hz` | `500.0` | 主控制循环频率 |
| `--max-velocity` | `0.5` | `servoJ/servoL` 速度参数 |
| `--max-acceleration` | `1.0` | `servoJ/servoL` 加速度参数 |
| `--lookahead-time` | `0.06` | UR servo lookahead time |
| `--servo-gain` | `500.0` | UR servo gain |
| `--speed-slider-fraction` | `1.0` | 启动 bridge 时设置 UR speed slider |
| `--realtime-priority` | `0` | teleop 进程 SCHED_FIFO 优先级；`0` 表示关闭 |
| `--lock-memory` | 关闭 | 使用 `mlockall` 锁住进程内存 |
| `--control-latency-warning-sec` | `0.02` | 控制周期 gap warning 阈值 |
| `--ik-max-position-error` | `1e-4` | `servo_j` IK 位置误差阈值 |
| `--ik-max-orientation-error` | `1e-3` | `servo_j` IK 姿态误差阈值 |

### SpeedJ bridge

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--speedj-backend` | `direct` | `direct` 由 Python 发 `speedJ`；`register_bridge` 由机器人端 2 ms loop 执行 |
| `--speedj-register-start` | `42` | RTDE input integer register 起点 |
| `--speedj-velocity-scale` | `10000.0` | 速度写入 integer register 的缩放 |
| `--speedj-watchdog-sec` | `0.06` | bridge 短时无命令时进入零速度 |
| `--speedj-stale-stop-sec` | `1.0` | bridge 长时间无命令时退出脚本 |
| `--speedj-upload-port` | `30001` | 上传 URScript 的端口 |

### DLS 速度 IK

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--dls-linear-gain` | `8.0` | TCP 平移误差到线速度的比例 |
| `--dls-angular-gain` | `6.0` | TCP 姿态误差到角速度的比例 |
| `--dls-damping` | `0.03` | DLS 基础阻尼 |
| `--dls-max-damping` | `0.25` | 接近奇异点时的最大阻尼 |
| `--dls-position-deadband` | `0.0` | TCP 平移误差死区 |
| `--dls-orientation-deadband` | `0.0` | TCP 姿态误差死区 |
| `--dls-singularity-threshold` | `0.05` | 最小奇异值低于该值时开始降速 |
| `--dls-hard-stop-threshold` | `0.015` | 最小奇异值低于该值时硬停 |
| `--dls-frame-name` | `tool0` | 本地 Jacobian 末端 frame |
| `--dls-jacobian-source` | `local` | Jacobian 来源：`local` 或 `rtde` |
| `--ur-description-path` | `None` | 自定义 `ur_description` 路径 |
| `--max-tcp-linear-speed` | `0.25` | DLS 输入 TCP 平移速度上限 |
| `--max-tcp-angular-speed` | `0.8` | DLS 输入 TCP 角速度上限 |
| `--max-joint-speed` | `0.8` | 输出关节速度上限 |
| `--max-joint-acceleration` | `2.0` | 输出关节加速度上限 |

### XR 输入

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--pose-source` | `right_controller` | XR 位姿源 |
| `--grip-name` | `right_grip` | 机械臂运动使能输入 |
| `--trigger-name` | `right_trigger` | 夹爪控制输入 |
| `--emergency-button` | `right_axis_click` | 急停按钮 |
| `--xr-service-realtime-priority` | `0` | XR service SCHED_FIFO 优先级 |
| `--grip-threshold` | `0.95` | grip 进入运动的阈值 |
| `--grip-release-threshold` | `None` | grip 释放阈值；为空时自动低于 start threshold |
| `--grip-release-grace-sec` | `0.0` | grip 短暂抖动的宽限时间 |
| `--xr-timeout-sec` | `0.2` | XR 数据超时停止阈值 |
| `--xr-target-interpolation-sec` | `0.0` | XR 短 gap 外推时间 |
| `--scale-factor` | `0.6` | 手柄平移缩放 |
| `--rotation-scale` | `1.0` | 手柄旋转缩放 |

### 安全限幅与平滑

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--max-step-translation` | `0.01` | 每周期 TCP 目标最大平移 |
| `--max-step-rotation` | `0.05` | 每周期 TCP 目标最大旋转 |
| `--max-joint-step` | `0.04` | `servo_j` 每周期最大关节步长 |
| `--unsafe-disable-teleop-safety-limits` | 关闭 | 禁用 Python 侧限速、平滑、workspace clamp 和 lag limit |
| `--target-smoothing-mode` | `linear` | 目标重采样方式：`none`、`linear`、`ruckig` |
| `--target-smoothing-sec` | `0.0` | `linear` 平滑窗口 |
| `--ruckig-max-linear-acceleration` | `1.5` | Ruckig 平移加速度上限 |
| `--ruckig-max-linear-jerk` | `30.0` | Ruckig 平移 jerk 上限 |
| `--ruckig-target-prediction-sec` | `0.006` | Ruckig 目标短预测时间 |
| `--ruckig-velocity-hold-sec` | `0.015` | XR 时间戳停顿时保留最近速度的时间 |
| `--max-target-lag-translation` | `0.0` | 目标 TCP 最多领先当前 TCP 的平移距离 |
| `--max-target-lag-rotation` | `0.0` | 目标 TCP 最多领先当前 TCP 的旋转距离 |
| `--workspace-min` | `None` | TCP workspace 下界，格式 `x,y,z` |
| `--workspace-max` | `None` | TCP workspace 上界，格式 `x,y,z` |

### DH 夹爪

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--disable-gripper` | 关闭 | 禁用夹爪 |
| `--gripper-port` | `/tmp/ttyUR` | 夹爪串口 |
| `--gripper-baudrate` | `115200` | 串口波特率 |
| `--gripper-rate-hz` | `30.0` | 命令线程频率 |
| `--gripper-speed` | `100` | DH 目标速度 |
| `--gripper-force` | `50` | DH 目标力 |
| `--gripper-open-raw` | `1000` | 张开 raw 值 |
| `--gripper-closed-raw` | `0` | 闭合 raw 值 |
| `--gripper-binary-control` | 关闭 | trigger 二值控制 |
| `--gripper-trigger-threshold` | `0.5` | 二值控制触发阈值 |
| `--gripper-deadband` | `4` | raw 目标变化死区 |
| `--gripper-init-timeout-sec` | `8.0` | 初始化超时 |
| `--gripper-read-hz` | `30.0` | 当前位置读取频率 |

### State file

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--disable-state-file` | 关闭 | 禁止写 state file |
| `--state-file` | `/tmp/ur_teleop_state.json` | state file 路径 |
| `--state-hz` | `30.0` | state file 写入频率 |

### Recorder 控制

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--enable-recorder-control` | 关闭 | XR 按键控制 recorder service |
| `--recorder-start-button` | `A` | 开始录制按钮 |
| `--recorder-stop-button` | `B` | 停止录制按钮 |
| `--recorder-save-button` | `X` | 保存按钮 |
| `--recorder-discard-button` | `Y` | 丢弃按钮 |
| `--recorder-debounce-sec` | `0.5` | recorder 按键防抖 |

### 回 home 和 replay

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--disable-return-home-before-stop` | 关闭 | `B` 直接 stop，不自动回 home |
| `--return-home-joints` | 代码默认采集结束位 | home 关节角，格式 `q1,q2,q3,q4,q5,q6` |
| `--return-home-max-joint-step` | `0.0006` | 回 home 每周期最大关节步长 |
| `--return-home-tolerance` | `0.01` | home 到位关节误差阈值 |
| `--return-home-timeout-sec` | `20.0` | 回 home 超时 |
| `--return-home-hold-sec` | `0.3` | 到位后保持时间 |
| `--return-home-stop-grace-sec` | `0.2` | 写最后 state 后再 stop recorder 的等待时间 |
| `--replay-after-return-home` | 关闭 | 回 home 后加载本 episode sampled replay |
| `--replay-button` | `left_grip` | replay 触发输入 |
| `--replay-trigger-threshold` | `0.8` | replay 模拟量触发阈值 |
| `--replay-delay-sec` | `2.0` | 触发后延迟执行 replay |
| `--replay-record-hz` | `30.0` | 历史兼容参数；实际跟随 recorder sampled replay |
| `--replay-speed-scale` | `1.0` | replay 时间缩放 |
| `--replay-joint-gain` | `10.0` | replay 关节误差速度增益 |
| `--replay-tolerance` | `0.02` | replay 终点误差阈值 |

## 15. 与 recorder 的同步规则

recorder 按 `record_hz` 读取 `teleop_state_file`。推荐配置：

```text
teleop --state-hz 30
teleop --gripper-read-hz 30
recorder record_hz:=30.0
recorder color_fps:=30
```

action 语义保持为控制目标。夹爪 `gripper_target_raw` 不改写为下一帧 observation。

## 16. 故障处理

| 现象 | 检查项 |
| --- | --- |
| `xr_timeout` | XR 客户端、PC Service、无线链路、`--xr-timeout-sec` |
| 机械臂不动 | remote control、上电、刹车、speed slider、旧 RTDE 进程 |
| 夹爪不可用 | ToolComm、`/tmp/ttyUR`、串口占用 |
| recorder 没有 action | `--state-file` 路径、`--state-hz`、是否按住 grip |
| 500 Hz 下抖动 | lowlatency kernel、实时优先级、CPU governor、网络 |
| Ruckig import 失败 | 在 `XRobo` 环境安装 `ruckig` |

```bash
python3 -m pip install ruckig
python3 -c "import ruckig; print(ruckig.__version__)"
```

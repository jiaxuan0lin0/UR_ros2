# UR7e 机器人工作区说明

本工作区包含 UR7e 遥操作、数据采集、数据转换、传统 ROS2 控制封装和 policy 部署工具。每个包的详细命令和参数见对应 README。

## 1. 包目录

| 路径 | 用途 |
| --- | --- |
| [src/teleop](src/teleop/README.md) | XR 遥操作，RTDE 直接控制机械臂，DH 夹爪直连控制 |
| [src/recorder](src/recorder/README.md) | 同步采集相机、机械臂 observation 和 teleop action |
| [src/converter](src/converter/README.md) | 将 recorder episode 转换为 TFDS/RLDS |
| [src/policy_client](src/policy_client/README.md) | 机器人侧 policy 推理部署与 500 Hz RTDE 执行 |
| [src/controllers](src/controllers/README.md) | 传统 ROS2 action/topic 控制封装 |

## 2. 工作站登录

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

## 3. 推荐工作流

### 数据采集

1. 启动 XRoboToolkit PC Service。
2. 启动 UR 机器人并创建 `/tmp/ttyUR`。
3. 启动 `teleop.py`。
4. 启动 `recorder`。
5. 使用 XR 手柄控制录制 start/stop/save/discard。

详细说明：

```text
src/teleop/README.md
src/recorder/README.md
```

### 数据转换

1. 检查 recorder episode。
2. 转换为 RLDS。
3. 用 TFDS 读取验证。

详细说明：

```text
src/converter/README.md
```

### Policy 部署

1. 启动具体 policy server。
2. 通过 adapter 暴露统一 7D action contract。
3. 启动 `policy_client`。
4. 使用 `/policy_control/start` 或 `/policy_control/step` 执行。

详细说明：

```text
src/policy_client/README.md
src/policy_client/adapters/README.md
```

## 4. 构建

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. 控制入口互斥规则

同一时间只能使用一个机械臂控制入口。

| 控制入口 | 是否推荐用于采集 | 说明 |
| --- | --- | --- |
| `src/teleop/teleop.py` | 推荐 | XR 遥操作采集主链路 |
| `src/policy_client` | 推荐 | policy 部署主链路 |
| `src/controllers` | 调试/兼容 | ROS2 action/topic 控制封装 |
| `ur_robot_driver` + MoveIt Servo | 仅特定调试 | 不与 direct RTDE teleop/policy 同时运行 |

`/tmp/ttyUR` 同一时间只能被一个夹爪控制进程占用。

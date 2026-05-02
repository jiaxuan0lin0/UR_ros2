# recorder

`recorder` 是 ROS2 数据采集包，负责同步相机图像、UR RTDE 实际状态，以及 teleop 写出的 action 状态文件，最后保存成 episode 数据。

它不控制机械臂，也不控制夹爪。高性能方案中，控制由 `src/teleop/teleop.py` 直接完成，`recorder` 只做采集。

## 节点

### `record`

主采集节点：

- 启动 RealSense 相机。
- 通过 RTDE 读取 UR 当前关节角和 TCP 位姿。
- 从 `/tmp/ur_teleop_state.json` 读取 teleop action。
- 通过 service 控制 episode 的 start/stop/save/discard。
- 保存图像和 `robot_states.json`。

### `keyboard_control`

键盘控制客户端：

- `r`：开始录制
- `c`：停止当前 episode
- `s`：保存已停止的 episode
- `q`：丢弃已停止的 episode
- `x`：退出键盘控制节点

如果 teleop 启动了 `--enable-recorder-control`，也可以不用这个键盘节点，直接用 XR 手柄按键控制录制。

## 启动

```bash
cd /home/ustc/UR
source /opt/ros/jazzy/setup.bash
source /home/ustc/UR/.venv/bin/activate
colcon build --symlink-install --packages-select recorder
source /home/ustc/UR/install/setup.bash

ros2 launch recorder record.launch.py \
  robot_ip:=192.168.1.102 \
  dataset_root:=/home/ustc/UR/dataset \
  color_fps:=30 \
  teleop_state_file:=/tmp/ur_teleop_state.json \
  python_executable:=/home/ustc/UR/.venv/bin/python3
```

另开一个终端启动键盘控制：

```bash
cd /home/ustc/UR
source /opt/ros/jazzy/setup.bash
source /home/ustc/UR/.venv/bin/activate
source /home/ustc/UR/install/setup.bash

ros2 run recorder keyboard_control
```

也可以直接调用 service：

```bash
ros2 service call /recorder/start std_srvs/srv/Trigger {}
ros2 service call /recorder/stop std_srvs/srv/Trigger {}
ros2 service call /recorder/save std_srvs/srv/Trigger {}
ros2 service call /recorder/discard std_srvs/srv/Trigger {}
```

## Launch 参数

- `robot_ip`：UR 控制器 IP，默认 `192.168.1.102`。
- `rtde_frequency`：RTDE receive 频率，默认 `-1.0`，表示使用库默认值。
- `dataset_root`：episode 保存目录，默认 `/home/ustc/UR/dataset`。
- `color_width`：相机彩色图宽度，默认 `640`。
- `color_height`：相机彩色图高度，默认 `480`。
- `color_fps`：相机 FPS，默认 `30`。两台 RealSense 建议分别插到不同 USB 控制器/总线上。
- `teleop_state_file`：teleop action 状态文件，默认 `/tmp/ur_teleop_state.json`。
- `teleop_state_timeout_sec`：状态文件最大允许年龄，默认 `1.0` 秒。
- `python_executable`：运行 recorder 的 Python，默认使用当前 `VIRTUAL_ENV` 下的 python3，否则 `/usr/bin/python3`。本机推荐显式传 `/home/ustc/UR/.venv/bin/python3`，因为 `rtde_receive` 装在这个虚拟环境里。

## 输出目录

每次 start 会创建一个临时 episode 目录：

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

只有调用 `/recorder/save` 后非空 episode 才会保留；空 episode 会在 save 时自动删除。调用 `/recorder/discard` 会删除当前 episode。

## 数据格式

`robot_states.json` 是一个 frame list，每一帧结构如下：

```json
{
  "frame_idx": 0,
  "timestamp": 1710000000.123,
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

字段语义：

- `observation.joint_positions`：RTDE 读到的当前真实关节角。
- `observation.tcp_pose`：RTDE 读到的当前真实 TCP 位姿，UR rotvec 表达。
- `observation.gripper_position`：夹爪当前开合，归一化到 `0.0-1.0`。
- `action.joint_positions`：teleop 实际发送给 `servoJ` 的目标关节角。
- `action.tcp_pose`：teleop 限幅后的目标 TCP 位姿。
- `action.gripper_position`：夹爪目标开合，归一化到 `0.0-1.0`。

这个结构同时支持 joint-space 训练和 EEF/TCP-space 训练。

## 与 teleop 同步

recorder 会先检查 teleop state file：

- 文件不存在：跳过采样。
- 文件过旧：跳过采样。
- 缺少 `target_q` 或 `target_tcp_pose`：跳过采样。

这样可以避免把 idle frame、无 action frame 或过期 action 写进训练数据。

## 依赖

- `rclpy`
- `std_srvs`
- `pyrealsense2`
- `rtde_receive`
- OpenCV Python

如果 `record` 启动失败，优先确认当前终端是否 source 了 ROS2、工作空间和 `.venv`：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ustc/UR/.venv/bin/activate
source /home/ustc/UR/install/setup.bash

python -c "import rclpy, rtde_receive, pyrealsense2; print('recorder deps ok')"
```

## 常见问题

录不到数据：确认 teleop 正在运行，并且 `/tmp/ur_teleop_state.json` 在持续更新。

一直提示 `missing target_joint_positions or target_tcp_pose`：teleop state file 还没有有效 action。确认 Quest 客户端已开启 `Tracking - Controller` 和 `Tracking - Data & Control - Send`，并且录制时按住右手 `grip` 让机械臂进入遥操作；`/tmp/ur_teleop_state.json` 里应出现 `target_q` 和 `target_tcp_pose`。

一直提示 skipping sample：通常是没有按住 grip 产生有效 action，或者 teleop state file 已过期。

相机打不开：确认 RealSense 设备没有被其他进程占用。

保存后没有 episode：需要先 `r` start，再 `c` stop，最后 `s` save。

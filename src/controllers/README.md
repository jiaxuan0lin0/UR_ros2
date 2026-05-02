# controllers

`controllers` 是 ROS2 控制封装包，提供传统 ROS2 控制链路下的 UR 机械臂和 DH 夹爪控制节点。

当前高性能 XR teleop 推荐使用 `src/teleop/teleop.py` 直接 RTDE 控制，不需要启动本包的节点。本包保留用于非 teleop 场景、调试、MoveIt/ros2_control 集成，以及需要 ROS2 action/topic 接口的程序。

## 重要提醒

如果你正在运行 `src/teleop/teleop.py` 的高性能方案：

- 不要启动本包的 `gripper` 节点，因为它会占用 `/tmp/ttyUR`。
- 不要同时用 `ur_joint_control` 或 `ur_cartesian_delta_control` 控制同一台机械臂。
- `controllers` 和 `teleop` 是两套控制入口，同一时间只选一套。

## 节点

### `gripper`

DH 夹爪 ROS2 节点。

订阅：

- `/gripper_trajectory` (`trajectory_msgs/JointTrajectory`)

发布：

- `/gripper_joint_states` (`sensor_msgs/JointState`)

默认参数：

- `port`：`/tmp/ttyUR`
- `baudrate`：`115200`
- `max_queue_points`：`500`

启动：

```bash
cd /home/ustc/UR
source /home/ustc/UR/install/setup.bash

ros2 launch controllers gripper.launch.py
```

### `ur_joint_control`

把 UR 机械臂的 FollowJointTrajectory action 和 DH 夹爪 topic 封装成一个 7-DoF joint action facade。

订阅：

- `/joint_states`：UR 机械臂 joint state。
- `/gripper_joint_states`：夹爪 joint state。

发布：

- `/ur/joint_states`：合并后的 6 轴机械臂 + 1 轴夹爪状态。
- `/gripper_trajectory`：夹爪目标轨迹。

Action server：

- `/ur/joint_control/follow_joint_trajectory`

Action client：

- `/scaled_joint_trajectory_controller/follow_joint_trajectory`

启动：

```bash
cd /home/ustc/UR
source /home/ustc/UR/install/setup.bash

ros2 launch controllers ur_joint_control.launch.py
```

### `ur_cartesian_delta_control`

面向 MoveIt Servo 的 Cartesian delta action facade，同时支持夹爪命令。

Action server：

- `/ur/cartesian_delta_control/follow_cartesian_delta`

发布：

- `/servo_node/pose_target_cmds`
- `/ur/tcp_pose`
- `/ur/cartesian_delta_control/state`
- `/ur/cartesian_delta_control/target_pose`
- `/gripper_trajectory`

依赖：

- MoveIt Servo 已启动。
- `/tcp_pose_broadcaster/pose` 可用。
- `/servo_node/switch_command_type` service 可用。

启动：

```bash
cd /home/ustc/UR
source /home/ustc/UR/install/setup.bash

ros2 launch controllers ur_cartesian_delta_control.launch.py
```

## 完整传统 ROS2 链路示例

传统链路通常是：

```text
ur_robot_driver
  -> /joint_states
  -> scaled_joint_trajectory_controller
  -> controllers/ur_joint_control 或 controllers/ur_cartesian_delta_control
  -> controllers/gripper
```

如果使用本包的 `ur_joint_control`，需要先启动 `ur_robot_driver`。下面这条命令会启动 UR ROS2 driver、创建 `/tmp/ttyUR` 工具串口、加载并激活 `scaled_joint_trajectory_controller`：

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=192.168.1.102 \
  launch_rviz:=false \
  headless_mode:=true \
  use_tool_communication:=true \
  tool_device_name:=/tmp/ttyUR \
  tool_baud_rate:=115200 \
  tool_parity:=0 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_voltage:=24 \
  initial_joint_controller:=scaled_joint_trajectory_controller
```

参数作用：

- `ur_type:=ur7e`：机器人型号，决定 URDF、joint limits、kinematics 和 driver 配置。
- `robot_ip:=192.168.1.102`：UR 控制柜 IP。
- `launch_rviz:=false`：不启动 RViz，减少不必要进程。
- `headless_mode:=true`：使用 driver 的 headless 模式，让 driver 通过 Dashboard/External Control 链路管理机器人控制程序，适合无 RViz、无手动点 play 的运行方式。
- `use_tool_communication:=true`：启用 UR e-Series tool communication 转发；driver 会创建本机伪串口。
- `tool_device_name:=/tmp/ttyUR`：PC 端生成的工具串口路径，`controllers/gripper` 会打开这个设备控制 DH 夹爪。
- `tool_baud_rate:=115200`：工具串口波特率，需要和 DH 夹爪配置一致。
- `tool_parity:=0`：串口校验位，`0` 通常表示 none。
- `tool_stop_bits:=1`：串口停止位。
- `tool_rx_idle_chars:=1.5`：Modbus RTU 接收方向帧间空闲字符数。
- `tool_tx_idle_chars:=3.5`：Modbus RTU 发送方向帧间空闲字符数。
- `tool_voltage:=24`：设置机器人 tool 输出电压为 24V。如果夹爪不是从机器人 tool 口取电，需要按实际接线调整。
- `initial_joint_controller:=scaled_joint_trajectory_controller`：启动并激活 UR driver 内置的 scaled joint trajectory controller。本包的 `ur_joint_control` 默认会把机械臂轨迹转发到 `/scaled_joint_trajectory_controller/follow_joint_trajectory`。

然后启动本包的 joint facade 和夹爪节点：

```bash
ros2 launch controllers ur_joint_control.launch.py
```

这时数据/命令关系是：

```text
你的上层程序
  -> /ur/joint_control/follow_joint_trajectory
  -> controllers/ur_joint_control
      -> /scaled_joint_trajectory_controller/follow_joint_trajectory
      -> /gripper_trajectory
  -> ur_robot_driver 控机械臂
  -> controllers/gripper 通过 /tmp/ttyUR 控 DH 夹爪
```

注意：使用这条 driver launch 时，不要再单独运行：

```bash
python3 /opt/ros/jazzy/lib/ur_client_library/tool_communication.py ...
```

因为 `use_tool_communication:=true` 已经由 `ur_robot_driver` 创建 `/tmp/ttyUR`，两个进程会抢同一个 tool communication 通道。

如果使用 Cartesian delta 控制，还需要启动 MoveIt Servo：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur7e \
  launch_servo:=true \
  launch_rviz:=false
```

## 与高性能 teleop 的区别

`controllers` 方案：

- 优点：ROS2 接口完整，适合集成 MoveIt、action client、传统 ROS 应用。
- 缺点：链路较长，延迟和抖动通常高于 direct RTDE。

`src/teleop` 方案：

- 优点：XR 到 UR 控制链路短，RTDE direct control 性能更好。
- 缺点：不是 ROS2 action 接口，主要服务于遥操作采集。

采集数据时推荐：

```text
teleop.py direct RTDE 控机械臂和夹爪
recorder 只记录相机、RTDE observation 和 teleop action
```

## 构建

```bash
cd /home/ustc/UR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select controllers controllers_msgs
source /home/ustc/UR/install/setup.bash
```

## 常见问题

`/tmp/ttyUR` 被占用：检查是否同时启动了 `teleop.py` 和 `controllers/gripper`。

夹爪无响应：确认 ToolComm Forwarder URCap 运行，且 PC 端 `/tmp/ttyUR` 已创建。

Cartesian delta action 无响应：确认 MoveIt Servo 正在运行，`/servo_node/pose_target_cmds` 有订阅者。

joint action 不执行：确认 `scaled_joint_trajectory_controller` 已启动，并且 action name 与参数一致。

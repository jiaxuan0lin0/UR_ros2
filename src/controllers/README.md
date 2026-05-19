# Controllers 使用说明

`src/controllers` 提供传统 ROS2 action/topic 控制封装。该包用于非 XR teleop 场景、MoveIt Servo 调试、ROS2 action 集成和兼容旧控制链路。XR 数据采集推荐使用 `src/teleop/teleop.py`；policy 部署推荐使用 `src/policy_client`。

## 1. 控制入口规则

同一时间只启动一种机械臂控制链路。

| 链路 | 说明 |
| --- | --- |
| `teleop.py` | direct RTDE XR 遥操作 |
| `policy_client` | direct RTDE policy 执行 |
| `controllers` | ROS2 driver/action/topic 控制 |

运行 `controllers/gripper` 时，不运行 `teleop.py` 的夹爪控制；两者都会占用 `/tmp/ttyUR`。

## 2. 控制方式总览

| 方式 | 底层 controller | MoveIt Servo | 本包 launch | Action |
| --- | --- | --- | --- | --- |
| Joint Trajectory | `scaled_joint_trajectory_controller` | 不需要 | `ur_joint_control.launch.py` | `/ur/joint_control/follow_joint_trajectory` |
| Cartesian RPY Delta | `forward_position_controller` | 需要 | `ur_cartesian_delta_control.launch.py` | `/ur/cartesian_delta_control/follow_cartesian_delta` |
| TCP Rotvec Delta | `forward_position_controller` | 需要 | `ur_tcp_rotvec_delta_control.launch.py` | `/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta` |

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

## 4. 构建

### 通用命令

```bash
cd <UR_WORKSPACE>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select controllers controllers_msgs
source install/setup.bash
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select controllers controllers_msgs
source install/setup.bash
```

## 5. Joint Trajectory 控制

### 启动 UR driver

通用命令：

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=<UR_TYPE> \
  robot_ip:=<ROBOT_IP> \
  launch_rviz:=false \
  headless_mode:=true \
  use_tool_communication:=true \
  tool_device_name:=<TOOL_TTY> \
  tool_baud_rate:=115200 \
  tool_parity:=0 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_voltage:=24 \
  initial_joint_controller:=scaled_joint_trajectory_controller
```

示例：

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

### 启动本包节点

```bash
ros2 launch controllers ur_joint_control.launch.py
```

### 检查

```bash
ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory
ros2 action info /ur/joint_control/follow_joint_trajectory
```

## 6. Cartesian RPY Delta 控制

动作格式：

```text
[dx, dy, dz, droll, dpitch, dyaw, gripper]
```

### 启动 UR driver

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
  initial_joint_controller:=forward_position_controller
```

### 启动 MoveIt Servo

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur7e \
  launch_servo:=true \
  launch_rviz:=false
```

### 启动本包节点

```bash
ros2 launch controllers ur_cartesian_delta_control.launch.py
```

### 检查

```bash
ros2 topic info /tcp_pose_broadcaster/pose -v
ros2 topic info /servo_node/pose_target_cmds -v
ros2 action info /ur/cartesian_delta_control/follow_cartesian_delta
```

## 7. TCP Rotvec Delta 控制

动作格式：

```text
[dx, dy, dz, drx, dry, drz, gripper]
```

`drx/dry/drz` 为 rotation vector delta，不是 RPY delta。默认 `command_frame:=base`，对应 UR RTDE base/TCP delta 语义。

### 启动 UR driver

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
  initial_joint_controller:=forward_position_controller
```

### 启动 MoveIt Servo

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur7e \
  launch_servo:=true \
  launch_rviz:=false
```

### 启动本包节点

```bash
ros2 launch controllers ur_tcp_rotvec_delta_control.launch.py
```

### 检查

```bash
ros2 topic info /tcp_pose_broadcaster/pose -v
ros2 topic info /servo_node/pose_target_cmds -v
ros2 action info /ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta
```

## 8. 单独启动夹爪节点

### 通用命令

```bash
ros2 launch controllers gripper.launch.py
```

### Topic

| 类型 | 名称 | 作用 |
| --- | --- | --- |
| subscribe | `/gripper_trajectory` | 接收夹爪轨迹 |
| publish | `/gripper_joint_states` | 发布夹爪状态 |

## 9. 链路检查命令

```bash
ros2 node list
ros2 control list_controllers
ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory
ros2 action info /ur/joint_control/follow_joint_trajectory
ros2 topic info /tcp_pose_broadcaster/pose -v
ros2 topic info /servo_node/pose_target_cmds -v
ros2 action info /ur/cartesian_delta_control/follow_cartesian_delta
ros2 action info /ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta
```

## 10. Launch 参数

### `ur_cartesian_delta_control.launch.py`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `default_point_duration_sec` | `0.03333333333333333` | action point 默认时长 |
| `servo_command_hz` | `125.0` | 发送 MoveIt Servo pose target 的频率 |
| `wait_for_final_pose_target` | `false` | chunk 发送完后是否等待最终 TCP 进入容差 |

### `ur_tcp_rotvec_delta_control.launch.py`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `default_point_duration_sec` | `0.03333333333333333` | action point 默认时长 |
| `servo_command_hz` | `125.0` | 发送 MoveIt Servo pose target 的频率 |
| `wait_for_final_pose_target` | `false` | chunk 发送完后是否等待最终 TCP 进入容差 |
| `command_frame` | `base` | delta action 所在坐标系 |

## 11. Node 参数

### `gripper`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `port` | `/tmp/ttyUR` | DH 夹爪串口 |
| `baudrate` | `115200` | 串口波特率 |
| `max_queue_points` | `500` | 夹爪轨迹队列最大点数 |

### `ur_joint_control`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `arm_joint_state_topic` | `/joint_states` | UR 机械臂 joint state |
| `gripper_joint_state_topic` | `/gripper_joint_states` | 夹爪 joint state |
| `joint_state_topic` | `/ur/joint_states` | 合并后的 7D joint state |
| `joint_control_action_name` | `/ur/joint_control/follow_joint_trajectory` | 本包对外 action |
| `arm_action_name` | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | 底层 UR action |
| `gripper_trajectory_topic` | `/gripper_trajectory` | 夹爪轨迹 topic |
| `state_publish_hz` | `50.0` | 合并 joint state 发布频率 |
| `gripper_raw_min` | `0.0` | 夹爪 raw 最小值 |
| `gripper_raw_max` | `1000.0` | 夹爪 raw 最大值 |
| `gripper_default_speed` | `100.0` | 夹爪默认速度 |
| `gripper_default_force` | `100.0` | 夹爪默认力 |
| `arm_goal_response_timeout_sec` | `2.0` | 等待底层 action 接受目标的超时 |
| `execution_timeout_margin_sec` | `2.0` | 执行超时余量 |
| `wait_poll_sec` | `0.02` | 等待轮询周期 |

### Cartesian / TCP Rotvec delta controller

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `cartesian_delta_action_name` | `/ur/cartesian_delta_control/follow_cartesian_delta` | Cartesian RPY action 名称 |
| `tcp_rotvec_delta_action_name` | `/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta` | TCP rotvec action 名称 |
| `servo_pose_topic` | `/servo_node/pose_target_cmds` | MoveIt Servo pose target topic |
| `servo_command_type_service` | `/servo_node/switch_command_type` | Servo command type service |
| `tcp_pose_topic` | `/tcp_pose_broadcaster/pose` | UR driver TCP pose topic |
| `tcp_pose_publish_topic` | `/ur/tcp_pose` | 本包转发 TCP pose topic |
| `gripper_joint_state_topic` | `/gripper_joint_states` | 夹爪 state topic |
| `gripper_trajectory_topic` | `/gripper_trajectory` | 夹爪命令 topic |
| `state_topic` | controller 对应 state topic | 控制状态发布 |
| `target_pose_publish_topic` | controller 对应 target topic | 目标 pose 发布 |
| `command_frame` | Cartesian 默认 `tool0`，rotvec 默认 `base` | delta action 坐标系 |
| `servo_pose_frame` | `base_link` | 发送给 MoveIt Servo 的 frame |
| `tool_frame` | `tool0` | ROS tool frame |
| `default_point_duration_sec` | `1/30` | action point 默认时长 |
| `minimum_point_duration_sec` | `0.004` | 最小 point 时长 |
| `servo_command_hz` | `125.0` | Servo target 发布频率 |
| `state_publish_hz` | `30.0` | state topic 发布频率 |
| `require_servo_subscriber` | `true` | 要求 Servo topic 有订阅者 |
| `set_servo_pose_command_type` | `true` | 启动时切换 Servo command type |
| `require_gripper_subscriber` | `true` | 要求夹爪 topic 有订阅者 |
| `subscriber_timeout_sec` | `2.0` | 等待订阅者超时 |
| `servo_service_timeout_sec` | `2.0` | 等待 Servo service 超时 |
| `tf_timeout_sec` | `0.2` | TF 查询超时 |
| `require_recent_tcp_pose` | `true` | 要求 TCP pose 新鲜 |
| `tcp_pose_timeout_sec` | `1.0` | TCP pose 最大年龄 |
| `pose_linear_tolerance` | `0.003` | 最终 TCP 平移容差 |
| `pose_angular_tolerance` | `0.03` | 最终 TCP 姿态容差 |
| `wait_for_final_pose_target` | node 默认 `true`，launch 默认 `false` | 是否等待最终 TCP 达到容差 |
| `pose_target_timeout_sec` | `5.0` | 等待最终 TCP 超时 |
| `gripper_joint_name` | `gripper_joint` | 夹爪 joint 名称 |
| `gripper_default_speed` | `100.0` | 夹爪默认速度 |
| `gripper_default_force` | `100.0` | 夹爪默认力 |
| `gripper_position_tolerance` | `5.0` | 夹爪到位 raw 容差 |
| `gripper_timeout_sec` | `3.0` | 夹爪等待超时 |
| `gripper_settle_time_sec` | `0.0` | 夹爪命令后额外等待 |

## 12. 故障处理

| 现象 | 检查项 |
| --- | --- |
| `/tmp/ttyUR` 被占用 | 是否同时运行 teleop 和 controllers gripper |
| 夹爪无响应 | ToolComm、URCap、`/tmp/ttyUR`、波特率 |
| joint action 不执行 | `scaled_joint_trajectory_controller` 是否 active |
| Cartesian action 无响应 | MoveIt Servo、`forward_position_controller`、`/servo_node/pose_target_cmds` |
| TCP rotvec action 无响应 | MoveIt Servo、`/tcp_pose_broadcaster/pose`、action 名称 |
| `No recent TCP pose` | UR driver 或 `tcp_pose_broadcaster` 未正常发布 |
| 等待最终 pose 超时 | `wait_for_final_pose_target`、动作幅度、容差设置 |

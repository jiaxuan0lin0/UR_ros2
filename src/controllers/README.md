# controllers

`controllers` provides classic ROS 2 action/topic wrappers for the UR7e arm and the DH gripper. They sit on top of `ur_robot_driver` (and MoveIt Servo for the Cartesian modes). Use them for debugging, ROS 2 action integration and compatibility with older control chains.

The main data pipeline does not use this package: `teleop` → `recorder` → `converter` → `policy_client` drive the arm over direct RTDE. The custom messages and actions live in the sibling package `controllers_msgs` and are documented [below](#controllers_msgs-interfaces).

For workspace setup and build, see the [top-level README](../../README.md).

## How it works

| Node | Launch file | Driver controller | MoveIt Servo | Action |
| --- | --- | --- | --- | --- |
| `ur_joint_control` | `ur_joint_control.launch.py` | `scaled_joint_trajectory_controller` | no | `/ur/joint_control/follow_joint_trajectory` |
| `ur_cartesian_delta_control` | `ur_cartesian_delta_control.launch.py` | `forward_position_controller` | yes | `/ur/cartesian_delta_control/follow_cartesian_delta` |
| `ur_tcp_rotvec_delta_control` | `ur_tcp_rotvec_delta_control.launch.py` | `forward_position_controller` | yes | `/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta` |
| `gripper` | `gripper.launch.py` | none | no | none (topics only) |

Each of the three arm launch files also starts the `gripper` node.

- **`gripper`** opens the DH gripper on `/tmp/ttyUR` over Modbus and retries every second until it is ready. Commands arriving on `/gripper_trajectory` are queued. The node runs them one at a time and starts the next only after the gripper reports that the current motion has finished. It publishes the raw gripper state at 20 Hz.
- **`ur_joint_control`** accepts a 7-DoF `FollowJointTrajectory` (six UR joints plus `gripper_joint`). It forwards the six arm joints to `scaled_joint_trajectory_controller`. Each gripper value (normalized 0–1) is converted to a raw target and published at that point's `time_from_start`. The node also publishes a merged 7-D joint state.
- **`ur_cartesian_delta_control`** starts from the current `tool0` pose in `base_link` (taken from TF). Each point's `[dx, dy, dz, droll, dpitch, dyaw]` is applied to the previous *target* pose, so deltas accumulate. Each target is held constant for `default_point_duration_sec` and published to `/servo_node/pose_target_cmds` at `servo_command_hz`. Translation and rotation are expressed in `command_frame` (default `tool0`).
- **`ur_tcp_rotvec_delta_control`** works the same way, but in UR TCP terms. It reads the TCP pose from `/tcp_pose_broadcaster/pose` and applies `[dx, dy, dz, drx, dry, drz]` as `R_next = exp(rotvec) * R_current` in `command_frame` (default `base`, matching UR RTDE semantics). Within each point it interpolates linearly (position) and with slerp (orientation). It converts the TCP target back to a `tool0` target using the tool0→TCP offset captured at goal start.

In both delta modes the gripper value is a **raw** DH position (0–1000), and NaN means "leave the gripper alone". All gripper values in a goal are sent to the gripper node at once at goal start. The gripper then runs them back to back, so it is not time-aligned with the arm points.

## Quick start

The robot must be in remote control, powered on, with brakes released. No other control entry (`teleop`, `policy_client`) may be running.

1. Start the UR driver. Use `scaled_joint_trajectory_controller` for joint control and `forward_position_controller` for the two delta modes. With `use_tool_communication:=true` the driver creates `/tmp/ttyUR`, so do not also run `tool_communication.py`.

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e \
  robot_ip:=<ROBOT_IP> \
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
  initial_joint_controller:=<scaled_joint_trajectory_controller|forward_position_controller>
```

2. Delta modes only: start MoveIt Servo.

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur7e launch_servo:=true launch_rviz:=false
```

3. Start one wrapper (each one also starts `gripper`).

```bash
ros2 launch controllers ur_joint_control.launch.py
# or
ros2 launch controllers ur_cartesian_delta_control.launch.py
# or
ros2 launch controllers ur_tcp_rotvec_delta_control.launch.py command_frame:=base
```

4. Check the chain and send a small test goal. In the rotvec example below, the arm moves 5 mm in +z of `base` and the gripper is left unchanged.

```bash
ros2 control list_controllers
ros2 topic info /servo_node/pose_target_cmds -v
ros2 action info /ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta

ros2 action send_goal /ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta \
  controllers_msgs/action/FollowTcpRotvecDelta \
  "{points: [{translation: {x: 0.0, y: 0.0, z: 0.005}, rotvec: {x: 0.0, y: 0.0, z: 0.0}, gripper: .nan}]}"
```

Joint control goal (replace `<q1>`…`<q6>` with a pose close to the current one; `gripper_joint` is normalized, 1.0 = open):

```bash
ros2 action send_goal /ur/joint_control/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint, gripper_joint],
    points: [{positions: [<q1>, <q2>, <q3>, <q4>, <q5>, <q6>, 1.0], time_from_start: {sec: 3}}]}}"
```

Gripper on its own: `ros2 launch controllers gripper.launch.py`. It takes no launch arguments and uses `port:=/tmp/ttyUR` and `baudrate:=115200`.

## Parameters

### Launch arguments

| Argument | Default | Launch files | Description |
| --- | --- | --- | --- |
| `default_point_duration_sec` | `0.03333333333333333` (1/30) | cartesian, rotvec | Time spent on each action point |
| `servo_command_hz` | `125.0` | cartesian, rotvec | Rate of pose targets sent to Servo |
| `wait_for_final_pose_target` | `false` | cartesian, rotvec | Wait for the TCP to reach the final target before succeeding |
| `command_frame` | `base` | rotvec | Frame of the delta. The Cartesian launch file sets `tool0` and does not expose it |

`ur_joint_control.launch.py` and `gripper.launch.py` take no arguments. The launch files set the topic and action names to the node defaults listed below.

### `gripper` node

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/tmp/ttyUR` | DH gripper serial device |
| `baudrate` | `115200` | Serial baud rate |
| `max_queue_points` | `500` | A new trajectory is dropped if queued + new points would exceed this |

The topic names are hard-coded. Commanded points are clamped to position 0–1000, speed 1–100 and force 20–100. A missing velocity or effort defaults to 100. `time_from_start` is ignored.

### `ur_joint_control` node

| Parameter | Default | Description |
| --- | --- | --- |
| `arm_joint_state_topic` | `/joint_states` | UR joint states from the driver |
| `gripper_joint_state_topic` | `/gripper_joint_states` | Raw gripper state |
| `joint_state_topic` | `/ur/joint_states` | Merged 7-D joint state output |
| `joint_control_action_name` | `/ur/joint_control/follow_joint_trajectory` | Wrapper action server |
| `arm_action_name` | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | Driver action used for the arm |
| `gripper_trajectory_topic` | `/gripper_trajectory` | Gripper command output |
| `state_publish_hz` | `50.0` | Merged joint state rate |
| `gripper_raw_min` / `gripper_raw_max` | `0.0` / `1000.0` | Raw range mapped to normalized 0–1 |
| `gripper_default_speed` / `gripper_default_force` | `100.0` / `100.0` | Speed/force sent with each gripper target |
| `arm_goal_response_timeout_sec` | `2.0` | Wait for the driver action server and goal acceptance |
| `execution_timeout_margin_sec` | `2.0` | Added to the last `time_from_start` to form the execution timeout |
| `wait_poll_sec` | `0.02` | Internal polling period |

### `ur_cartesian_delta_control` and `ur_tcp_rotvec_delta_control` nodes

| Parameter | Default | Description |
| --- | --- | --- |
| `cartesian_delta_action_name` / `tcp_rotvec_delta_action_name` | `/ur/cartesian_delta_control/follow_cartesian_delta` / `/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta` | Action server name |
| `state_topic` | `/ur/<mode>/state` | State output (`<mode>` = `cartesian_delta_control` or `tcp_rotvec_delta_control`) |
| `target_pose_publish_topic` | `/ur/<mode>/target_pose` | Copy of every pose target sent to Servo |
| `servo_pose_topic` | `/servo_node/pose_target_cmds` | MoveIt Servo pose command topic |
| `servo_command_type_service` | `/servo_node/switch_command_type` | Service used to switch Servo to POSE |
| `tcp_pose_topic` / `tcp_pose_publish_topic` | `/tcp_pose_broadcaster/pose` / `/ur/tcp_pose` | Driver TCP pose input / relay output |
| `gripper_joint_state_topic` / `gripper_trajectory_topic` | `/gripper_joint_states` / `/gripper_trajectory` | Gripper state input / command output |
| `command_frame` | `tool0` (cartesian), `base` (rotvec) | Default delta frame when the goal leaves it empty |
| `servo_pose_frame` / `tool_frame` | `base_link` / `tool0` | Frame of Servo targets / tool frame for TF lookups |
| `default_point_duration_sec` | `1/30` | Time per point (node default; launch default is the same) |
| `minimum_point_duration_sec` | `0.004` | Lower bound on the point duration |
| `servo_command_hz` | `125.0` | Pose target publish rate |
| `state_publish_hz` | `30.0` | State topic rate |
| `require_servo_subscriber` / `require_gripper_subscriber` | `true` / `true` | Abort if nobody subscribes within `subscriber_timeout_sec` |
| `set_servo_pose_command_type` | `true` | Switch Servo to POSE at the start of every goal |
| `subscriber_timeout_sec` / `servo_service_timeout_sec` | `2.0` / `2.0` | Wait limits for subscribers / Servo service |
| `tf_timeout_sec` | `0.2` | TF lookup timeout |
| `require_recent_tcp_pose` / `tcp_pose_timeout_sec` | `true` / `1.0` | Abort unless a TCP pose newer than this arrived |
| `wait_for_final_pose_target` | `true` (node), `false` (launch) | Wait until the TCP is within tolerance of the final target |
| `pose_linear_tolerance` / `pose_angular_tolerance` | `0.003` m / `0.03` rad | Final-pose tolerances |
| `pose_target_timeout_sec` | `5.0` | Final-pose wait limit |
| `gripper_joint_name` | `gripper_joint` | Joint name used in gripper messages |
| `gripper_default_speed` / `gripper_default_force` | `100.0` / `100.0` | Used when the goal value is ≤ 0 |
| `gripper_position_tolerance` / `gripper_timeout_sec` / `gripper_settle_time_sec` | `5.0` raw / `3.0` / `0.0` | Defaults for `wait_for_gripper` when the goal value is ≤ 0 |

## Interfaces

| Name | Type | Role |
| --- | --- | --- |
| `/ur/joint_control/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action server (`ur_joint_control`) |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action client (`ur_joint_control`) |
| `/ur/cartesian_delta_control/follow_cartesian_delta` | `controllers_msgs/action/FollowCartesianDelta` | Action server |
| `/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta` | `controllers_msgs/action/FollowTcpRotvecDelta` | Action server |
| `/ur/joint_states` | `sensor_msgs/msg/JointState` | Published: 6 UR joints + `gripper_joint` (normalized 0–1). Only published once both arm and gripper states have arrived |
| `/gripper_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Subscribed by `gripper`; published by all wrappers |
| `/gripper_joint_states` | `sensor_msgs/msg/JointState` | Published by `gripper` (20 Hz): position = raw 0–1000, velocity/effort = target speed/force |
| `/servo_node/pose_target_cmds` | `geometry_msgs/msg/PoseStamped` | Published by delta nodes (frame `base_link`) |
| `/servo_node/switch_command_type` | `moveit_msgs/srv/ServoCommandType` | Service client (delta nodes) |
| `/tcp_pose_broadcaster/pose` → `/ur/tcp_pose` | `geometry_msgs/msg/PoseStamped` | Subscribed and relayed by delta nodes |
| `/ur/<mode>/state` | `controllers_msgs/msg/CartesianDeltaState` / `TcpRotvecDeltaState` | Published at 30 Hz |
| `/ur/<mode>/target_pose` | `geometry_msgs/msg/PoseStamped` | Published: copy of each Servo target |

For `ur_joint_control`, `joint_names` must contain `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` and `gripper_joint`, in any order. Every point needs all seven positions. Any failure returns `error_code = INVALID_GOAL` with the reason in `error_string`.

### controllers_msgs interfaces

| Message | Fields |
| --- | --- |
| `CartesianDeltaPoint` | `geometry_msgs/Vector3 translation` (m), `geometry_msgs/Vector3 rotation` (droll, dpitch, dyaw in rad), `float64 gripper` (raw DH position) |
| `TcpRotvecDeltaPoint` | `geometry_msgs/Vector3 translation` (m), `geometry_msgs/Vector3 rotvec` (axis-angle in rad), `float64 gripper` (raw DH position) |
| `CartesianDeltaState`, `TcpRotvecDeltaState` | `std_msgs/Header header`, `geometry_msgs/PoseStamped tcp_pose`, `sensor_msgs/JointState gripper_state`, `bool busy`, `uint32 active_point_index`, `uint32 active_point_count`, `string status` (`idle`, `starting`, `running`) |

`FollowCartesianDelta` and `FollowTcpRotvecDelta` share the same layout and differ only in the point type:

| Part | Fields |
| --- | --- |
| Goal | `points[]`; `string command_frame` (empty = node `command_frame`); `float64 gripper_velocity`, `float64 gripper_effort` (≤ 0 = node default); `bool wait_for_gripper`; `float64 gripper_position_tolerance`, `float64 gripper_timeout`, `float64 gripper_settle_time` (≤ 0 = node default) |
| Result | `bool success`, `string message`, `geometry_msgs/PoseStamped final_tcp_pose`, `sensor_msgs/JointState final_gripper_state` |
| Feedback | `uint32 current_point_index`, `uint32 total_points`, `geometry_msgs/PoseStamped tcp_pose`, `sensor_msgs/JointState gripper_state`, `string message` |

## Safety notes

- **One control entry at a time.** Do not run `controllers` together with `teleop` or `policy_client`. `ur_robot_driver` + Servo and direct RTDE clients fight over the robot.
- **One owner of `/tmp/ttyUR`.** Every arm launch file starts its own `gripper` node. Run only one launch file, and do not run teleop's gripper or `policy_client` with `direct_gripper:=true` at the same time.
- **No delta limits in the wrapper.** Deltas accumulate on the target pose without clamping. Only MoveIt Servo's own limits apply. Start with millimetre/milliradian steps.
- **Busy handling.** Delta nodes reject a new goal while one is running. `ur_joint_control` aborts it with `UR joint control is busy.`
- **Cancel.** Delta goals accept cancel and publish the current pose as the Servo target. `ur_joint_control` rejects cancel requests, so stop joint motion from the teach pendant or by stopping the driver.
- **Gripper units differ.** Joint mode uses normalized 0–1 (1 = open). Delta modes use raw 0–1000 (1000 = open); NaN skips the gripper.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `failed to open gripper port /tmp/ttyUR` | Driver started with `use_tool_communication:=true`; no other process (teleop, `policy_client`, a second launch) holds the port |
| Gripper ignores commands | `gripper not ready` / `queue is full` in the log; ToolComm, URCap, baud rate |
| `/ur/joint_states` not published | Both `/joint_states` and `/gripper_joint_states` must be arriving |
| `Arm action server not ready` / joint goal rejected | `scaled_joint_trajectory_controller` active (`ros2 control list_controllers`); robot program running |
| `7-DoF trajectory joint_names must contain ...` | Include all six UR joints plus `gripper_joint` |
| `No recent TCP pose on /tcp_pose_broadcaster/pose` | Driver running and `tcp_pose_broadcaster` active |
| `Could not transform tool0 pose to base_link` | `robot_state_publisher` / TF from the driver |
| `No Servo subscriber` / `Servo command type service not ready` | MoveIt Servo launched with `launch_servo:=true`; `forward_position_controller` active |
| `Timed out waiting for final ... pose target` | `wait_for_final_pose_target`, step size, `pose_*_tolerance`, `pose_target_timeout_sec` |
| `Timed out waiting for final gripper target` | `gripper_position_tolerance` is in raw units; increase `gripper_timeout` |

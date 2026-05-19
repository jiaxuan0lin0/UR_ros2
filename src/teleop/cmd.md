# Teleop 推荐启动命令

本文件保存当前推荐的 XR 遥操作采集命令。完整参数说明见 [README.md](README.md)。

## 1. 通用命令

```bash
cd <UR_WORKSPACE>
source /home/ustc/anaconda3/etc/profile.d/conda.sh
conda activate XRobo
source /opt/ros/jazzy/setup.bash
source <UR_WORKSPACE>/install/setup.bash

python3 src/teleop/teleop.py \
  --robot-ip <ROBOT_IP> \
  --ur-type <UR_TYPE> \
  --control-mode speed_j_dls \
  --speedj-backend register_bridge \
  --speed-slider-fraction 1.0 \
  --realtime-priority 80 \
  --lock-memory \
  --control-latency-warning-sec 0.02 \
  --rate-hz 500 \
  --xr-service-realtime-priority 60 \
  --grip-threshold 0.8 \
  --grip-release-threshold 0.55 \
  --xr-timeout-sec 0.35 \
  --xr-target-interpolation-sec 0.000 \
  --scale-factor 1 \
  --rotation-scale 1 \
  --max-step-translation 0.001 \
  --max-step-rotation 0.0019 \
  --target-smoothing-mode ruckig \
  --target-smoothing-sec 0.008 \
  --ruckig-max-linear-acceleration 1.6 \
  --ruckig-max-linear-jerk 30 \
  --ruckig-target-prediction-sec 0.000 \
  --ruckig-velocity-hold-sec 0.000 \
  --max-target-lag-translation 0.016 \
  --max-target-lag-rotation 0.055 \
  --dls-linear-gain 14.5 \
  --dls-angular-gain 8 \
  --dls-damping 0.12 \
  --dls-max-damping 0.25 \
  --dls-position-deadband 0.000 \
  --dls-orientation-deadband 0.006 \
  --dls-singularity-threshold 0.05 \
  --dls-hard-stop-threshold 0.015 \
  --dls-jacobian-source local \
  --dls-frame-name tool0 \
  --max-tcp-linear-speed 0.25 \
  --max-tcp-angular-speed 0.95 \
  --max-joint-speed 0.68 \
  --max-joint-acceleration 3.0 \
  --gripper-port <TOOL_TTY> \
  --gripper-rate-hz 30 \
  --gripper-read-hz 30 \
  --state-file <TELEOP_STATE_FILE> \
  --state-hz 30 \
  --enable-recorder-control \
  --return-home-max-joint-step 0.003 \
  --return-home-timeout-sec 30 \
  --return-home-stop-grace-sec 0.2 \
  --replay-after-return-home \
  --replay-button left_grip \
  --replay-trigger-threshold 0.8 \
  --replay-delay-sec 0.0 \
  --servo-gain 500
```

## 2. 示例

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
  --speed-slider-fraction 1.0 \
  --realtime-priority 80 \
  --lock-memory \
  --control-latency-warning-sec 0.02 \
  --rate-hz 500 \
  --xr-service-realtime-priority 60 \
  --grip-threshold 0.8 \
  --grip-release-threshold 0.55 \
  --xr-timeout-sec 0.35 \
  --xr-target-interpolation-sec 0.000 \
  --scale-factor 1 \
  --rotation-scale 1 \
  --max-step-translation 0.001 \
  --max-step-rotation 0.0019 \
  --target-smoothing-mode ruckig \
  --target-smoothing-sec 0.008 \
  --ruckig-max-linear-acceleration 1.6 \
  --ruckig-max-linear-jerk 30 \
  --ruckig-target-prediction-sec 0.000 \
  --ruckig-velocity-hold-sec 0.000 \
  --max-target-lag-translation 0.016 \
  --max-target-lag-rotation 0.055 \
  --dls-linear-gain 14.5 \
  --dls-angular-gain 8 \
  --dls-damping 0.12 \
  --dls-max-damping 0.25 \
  --dls-position-deadband 0.000 \
  --dls-orientation-deadband 0.006 \
  --dls-singularity-threshold 0.05 \
  --dls-hard-stop-threshold 0.015 \
  --dls-jacobian-source local \
  --dls-frame-name tool0 \
  --max-tcp-linear-speed 0.25 \
  --max-tcp-angular-speed 0.95 \
  --max-joint-speed 0.68 \
  --max-joint-acceleration 3.0 \
  --gripper-port /tmp/ttyUR \
  --gripper-rate-hz 30 \
  --gripper-read-hz 30 \
  --state-file /tmp/ur_teleop_state.json \
  --state-hz 30 \
  --enable-recorder-control \
  --return-home-max-joint-step 0.003 \
  --return-home-timeout-sec 30 \
  --return-home-stop-grace-sec 0.2 \
  --replay-after-return-home \
  --replay-button left_grip \
  --replay-trigger-threshold 0.8 \
  --replay-delay-sec 0.0 \
  --servo-gain 500
```

## 3. 频率约定

| 项目 | 值 | 说明 |
| --- | --- | --- |
| 机械臂控制环 | `500 Hz` | 对齐 UR 2 ms 控制周期 |
| teleop state file | `30 Hz` | 供 recorder 读取 action |
| AG95 命令线程 | `30 Hz` | 代码限制不超过 `50 Hz` |
| AG95 读取线程 | `30 Hz` | 供 recorder 写 gripper observation |

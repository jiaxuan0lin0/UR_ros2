# Converter 使用说明

`src/converter` 将 `recorder` 生成的 episode 文件夹转换为 TFDS/RLDS 数据集。转换过程不需要 ROS2，不控制机器人。

## 1. 输入与输出

### 输入

```text
dataset/
  episode_1710000000123/
    cam_0/
      00000.jpg
    cam_1/
      00000.jpg
    robot_states.json
```

### 输出

```text
rlds_dataset/
  ur_robot_rlds/
    1.0.0/
      dataset_info.json
      features.json
      ur_robot_rlds-train.tfrecord-00000-of-00001
```

## 2. 创建转换环境

转换建议使用独立 Python venv。不要在该终端 source ROS2。

### 通用命令

```bash
cd <UR_WORKSPACE>
python3 -m venv src/converter/.venv
source src/converter/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

### 示例

```bash
cd /data/jiaxuanLin/UR
python3 -m venv src/converter/.venv
source src/converter/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

### 依赖检查

```bash
python - <<'PY'
import tensorflow as tf
import tensorflow_datasets as tfds

print("tensorflow:", tf.__version__)
print("tensorflow_datasets:", tfds.__version__)
PY
```

## 3. 检查原始 episode

### 通用命令

```bash
cd <UR_WORKSPACE>
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS.py \
  --dataset-root <DATASET_ROOT> \
  --inspect-only
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --inspect-only
```

### 只检查状态和动作

```bash
python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --inspect-only \
  --no-images
```

检查内容：

| 项目 | 要求 |
| --- | --- |
| episode 目录 | `episode_*` |
| metadata | 每个 episode 有 `robot_states.json` |
| frame index | `frame_idx` 连续 |
| joint | `joint_positions` 为 6 维 |
| tcp | `tcp_pose` 为 6 维 |
| gripper | `gripper_position` 为数值 |
| image | 启用图像时，`cam_*/00000.jpg` 存在 |

## 4. 转换为 RLDS

### 通用命令

```bash
cd <UR_WORKSPACE>
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS.py \
  --dataset-root <DATASET_ROOT> \
  --output-dir <RLDS_OUTPUT_DIR> \
  --language-instruction "<LANGUAGE_INSTRUCTION>" \
  --overwrite
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --output-dir /data/jiaxuanLin/UR/rlds_dataset \
  --language-instruction "perform the demonstrated robot task" \
  --overwrite
```

### 只转换前 N 个 episode

```bash
python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --output-dir /data/jiaxuanLin/UR/rlds_dataset \
  --max-episodes 3 \
  --overwrite
```

### 不包含图像

```bash
python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --output-dir /data/jiaxuanLin/UR/rlds_dataset \
  --no-images \
  --overwrite
```

## 5. RLDS 字段映射

| recorder 字段 | RLDS 字段 |
| --- | --- |
| `episode_*` | 一个 RLDS episode |
| `robot_states.json` 中的一帧 | 一个 RLDS step |
| `observation.joint_positions` | `step["observation"]["joint_positions"]` |
| `observation.tcp_pose` | `step["observation"]["tcp_pose"]` |
| `observation.gripper_position` | `step["observation"]["gripper_position"]` |
| `cam_0/00000.jpg` | `step["observation"]["images"]["cam_0"]` |
| `action.joint_positions` | `step["action"]["joint_positions"]` |
| `action.tcp_pose` | `step["action"]["tcp_pose"]` |
| `action.gripper_position` | `step["action"]["gripper_position"]` |
| 第一帧 | `is_first=True` |
| 最后一帧 | `is_last=True` |

额外合并向量：

| 字段 | 内容 |
| --- | --- |
| `observation.state` | 6 维 joint + 6 维 tcp + 1 维 gripper，共 13 维 |
| `action.joint_action` | 6 维 target joint + 1 维 target gripper，共 7 维 |
| `action.tcp_action` | 6 维 target tcp + 1 维 target gripper，共 7 维 |

## 6. TCP rotvec unwrap

UR TCP pose 使用 rotation vector。接近 `+pi/-pi` 分支边界时，原始 rotvec 可能出现接近 `2pi` 的数值跳变。转换脚本默认对每个 episode 内的以下字段做 unwrap：

```text
observation.tcp_pose[3:]
action.tcp_pose[3:]
```

保留原始 rotvec 时使用：

```bash
python src/converter/scripts/RLDS.py \
  --dataset-root /data/jiaxuanLin/UR/dataset \
  --output-dir /data/jiaxuanLin/UR/rlds_dataset \
  --no-unwrap-tcp-rotvec \
  --overwrite
```

## 7. 读取转换结果

### 通用命令

```bash
cd <UR_WORKSPACE>
source src/converter/.venv/bin/activate

python - <<'PY'
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory("<RLDS_DATASET_VERSION_DIR>")
ds = builder.as_dataset(split="train")

for episode in ds.take(1):
    print(episode.keys())
    for step in episode["steps"].take(1):
        print(step["observation"].keys())
        print(step["action"].keys())
        print(step["is_first"].numpy(), step["is_last"].numpy())
PY
```

### 示例

```bash
cd /data/jiaxuanLin/UR
source src/converter/.venv/bin/activate

python - <<'PY'
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(
    "/data/jiaxuanLin/UR/rlds_dataset/ur_robot_rlds/1.0.0"
)
ds = builder.as_dataset(split="train")

for episode in ds.take(1):
    print(episode.keys())
    for step in episode["steps"].take(1):
        print(step["observation"].keys())
        print(step["action"].keys())
        print(step["is_first"].numpy(), step["is_last"].numpy())
PY
```

## 8. 参数说明

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--dataset-root` | `/home/ustc/UR/dataset` | recorder 原始 episode 目录 |
| `--output-dir` | `/home/ustc/UR/rlds_dataset` | TFDS/RLDS 输出目录 |
| `--inspect-only` | 关闭 | 只检查输入，不写出 RLDS |
| `--no-images` | 关闭 | 不读取和写入图像 |
| `--max-episodes N` | `None` | 只转换前 N 个 episode |
| `--language-instruction` | 空字符串 | 写入每个 step 的语言任务 |
| `--reward-mode zero` | 默认 | 所有 step reward 为 `0.0` |
| `--reward-mode final_one` | 可选 | 最后一个 step reward 为 `1.0` |
| `--overwrite` | 关闭 | 输出版本目录存在时先删除 |
| `--unwrap-tcp-rotvec` | 开启 | 对 rotvec 做 episode 内 unwrap |
| `--no-unwrap-tcp-rotvec` | 关闭 | 保留原始 rotvec 数值 |

## 9. 故障处理

| 现象 | 检查项 |
| --- | --- |
| `No module named 'tensorflow'` | 当前环境不是 `src/converter/.venv` 或依赖未安装 |
| pip 提示 ROS dependency conflicts | 当前 shell source 过 ROS2；新开终端重新进入 venv |
| `No episode_* directories found` | `dataset_root` 下没有保存 episode |
| `Missing image` | `robot_states.json` 有帧，但对应 `cam_*/xxxxx.jpg` 缺失 |
| `Camera folders differ` | 各 episode 的 `cam_*` 目录不一致 |

修复依赖：

```bash
cd /data/jiaxuanLin/UR
source src/converter/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

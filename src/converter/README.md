# converter

`converter` 用来把 `recorder` 采集出来的 episode 文件夹转换成
TFDS/RLDS 数据集。

当前转换脚本：

```text
src/converter/scripts/RLDS
```

## 建议单独使用 venv

建议给 RLDS 转换单独建一个 Python venv。原因是 TensorFlow /
TensorFlow Datasets 依赖比较重，和 ROS2、teleop、RTDE、RealSense
环境混在一起容易互相影响。

推荐在一个新终端里创建并使用，不要先 `source /opt/ros/jazzy/setup.bash`。
RLDS 转换不需要 ROS2 环境，干净 venv 更省心。

你的 venv 放在 `src/converter/.venv`，后续命令都按这个路径写：

```bash
cd /home/ustc/UR

python3 -m venv src/converter/.venv
source src/converter/.venv/bin/activate

python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

检查依赖：

```bash
python - <<'PY'
import tensorflow as tf
import tensorflow_datasets as tfds

print("tensorflow:", tf.__version__)
print("tensorflow_datasets:", tfds.__version__)
PY
```

如果 `pip install tensorflow` 找不到合适版本，通常是当前 Python
版本不被你要安装的 TensorFlow wheel 支持。可以换一个 Python 版本重新建
venv，例如 Python 3.10/3.11/3.12。

## TFDS 是什么

TFDS 是 TensorFlow Datasets，作用可以理解成：

1. 把原始数据转换成 TensorFlow 训练更容易读取的标准目录。
2. 用 `DatasetBuilder` 描述数据结构，例如 image、state、action。
3. 生成后可以用 `tfds.builder_from_directory(...).as_dataset()` 读取。

这里我们不是用 TFDS 下载公开数据集，而是用它的自定义 dataset builder
把本机采集的 robot demonstration 写成标准化数据集。

RLDS 是 Reinforcement Learning Datasets 的格式约定。它的核心结构是：

```text
dataset
  episode
    steps
      observation
      action
      reward
      discount
      is_first
      is_last
      is_terminal
```

官方参考：

- TFDS overview: https://www.tensorflow.org/datasets/overview
- TFDS custom dataset guide: https://www.tensorflow.org/datasets/add_dataset
- RLDS format: https://github.com/google-research/rlds

## 输入数据格式

转换脚本读取 `recorder` 生成的目录：

```text
/home/ustc/UR/dataset/
  episode_1710000000123/
    cam_0/
      00000.jpg
      00001.jpg
    cam_1/
      00000.jpg
      00001.jpg
    robot_states.json
```

`robot_states.json` 每一帧需要类似：

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

## 先检查原始数据

正式转换前，先跑检查模式：

```bash
cd /home/ustc/UR
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS \
  --dataset-root /home/ustc/UR/dataset \
  --inspect-only
```

如果你暂时没有图像，或者只想检查 `robot_states.json`：

```bash
python src/converter/scripts/RLDS \
  --dataset-root /home/ustc/UR/dataset \
  --inspect-only \
  --no-images
```

检查模式会验证：

- `dataset-root` 下是否有 `episode_*` 文件夹。
- 每个 episode 是否有 `robot_states.json`。
- 每帧 `frame_idx` 是否连续。
- `joint_positions` 是否是 6 维。
- `tcp_pose` 是否是 6 维。
- `gripper_position` 是否是数字。
- 如果启用图像，所有 `cam_*/00000.jpg` 这类图片是否存在。

## 转换成 RLDS

基本转换命令：

```bash
cd /home/ustc/UR
source src/converter/.venv/bin/activate

python src/converter/scripts/RLDS \
  --dataset-root /home/ustc/UR/dataset \
  --output-dir /home/ustc/UR/rlds_dataset \
  --language-instruction "perform the demonstrated robot task" \
  --overwrite
```

只转换前几个 episode 做测试：

```bash
python src/converter/scripts/RLDS \
  --dataset-root /home/ustc/UR/dataset \
  --output-dir /home/ustc/UR/rlds_dataset \
  --max-episodes 3 \
  --overwrite
```

不包含图像：

```bash
python src/converter/scripts/RLDS \
  --dataset-root /home/ustc/UR/dataset \
  --output-dir /home/ustc/UR/rlds_dataset \
  --no-images \
  --overwrite
```

转换完成后，输出路径类似：

```text
/home/ustc/UR/rlds_dataset/ur_robot_rlds/1.0.0/
```

## RLDS 字段映射

| recorder 字段 | RLDS 字段 |
| --- | --- |
| 一个 `episode_*` 文件夹 | 一个 episode |
| `robot_states.json` 的一条 frame | 一个 step |
| `observation.joint_positions` | `step["observation"]["joint_positions"]` |
| `observation.tcp_pose` | `step["observation"]["tcp_pose"]` |
| `observation.gripper_position` | `step["observation"]["gripper_position"]` |
| `cam_0/00000.jpg` | `step["observation"]["images"]["cam_0"]` |
| `action.joint_positions` | `step["action"]["joint_positions"]` |
| `action.tcp_pose` | `step["action"]["tcp_pose"]` |
| `action.gripper_position` | `step["action"]["gripper_position"]` |
| 第一帧 | `is_first=True` |
| 最后一帧 | `is_last=True` |

脚本还额外生成了几个训练时常用的合并向量：

| 字段 | 内容 |
| --- | --- |
| `observation.state` | 6 维 joint + 6 维 tcp + 1 维 gripper，共 13 维 |
| `action.joint_action` | 6 维 target joint + 1 维 target gripper，共 7 维 |
| `action.tcp_action` | 6 维 target tcp + 1 维 target gripper，共 7 维 |

## 读取转换后的数据

用 TFDS 从输出目录读取：

```bash
cd /home/ustc/UR
source src/converter/.venv/bin/activate

python - <<'PY'
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(
    "/home/ustc/UR/rlds_dataset/ur_robot_rlds/1.0.0"
)
ds = builder.as_dataset(split="train")

for episode in ds.take(1):
    print(episode.keys())
    steps = episode["steps"]
    for step in steps.take(1):
        print(step["observation"].keys())
        print(step["action"].keys())
        print(step["is_first"].numpy(), step["is_last"].numpy())
PY
```

如果后续训练框架要求 numpy：

```python
import tensorflow_datasets as tfds

builder = tfds.builder_from_directory(
    "/home/ustc/UR/rlds_dataset/ur_robot_rlds/1.0.0"
)
ds = builder.as_dataset(split="train")
ds_np = tfds.as_numpy(ds)
```

## 参数说明

```text
--dataset-root
```

原始 recorder 数据目录，默认 `/home/ustc/UR/dataset`。

```text
--output-dir
```

转换后的 TFDS 数据目录，默认 `/home/ustc/UR/rlds_dataset`。

```text
--inspect-only
```

只检查，不写出 RLDS。

```text
--no-images
```

跳过 `cam_*` 图片，只转换状态和动作。

```text
--max-episodes N
```

只处理前 N 个 episode，适合先小规模测试。

```text
--language-instruction
```

写入每个 step 的语言任务描述。如果暂时没有语言任务，可以留空。

```text
--reward-mode zero
```

所有 step 的 reward 都写成 `0.0`，这是默认行为。

```text
--reward-mode final_one
```

最后一个 step 的 reward 写成 `1.0`，其余为 `0.0`。

```text
--overwrite
```

如果输出版本目录已经存在，先删除再重新生成。

## 常见问题

### No module named 'tensorflow'

说明当前环境不是 `src/converter/.venv`，或者还没有安装依赖：

```bash
source /home/ustc/UR/src/converter/.venv/bin/activate
python -m pip install tensorflow tensorflow-datasets
```

### pip 提示 ROS 相关 dependency conflicts

如果安装时看到类似：

```text
generate-parameter-library-py requires jinja2, which is not installed
launch-ros requires pyyaml, which is not installed
```

这通常说明当前 shell 已经 source 过 ROS2，导致 ROS 的 Python 包通过
`PYTHONPATH` 暴露给了这个 venv。转换 RLDS 不需要 ROS2，推荐开一个新终端：

```bash
cd /home/ustc/UR
source src/converter/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

如果你就想继续用当前 shell，也可以补齐这些 ROS 包缺的依赖：

```bash
python -m pip install jinja2 pyyaml typeguard
```

只要下面检查能通过，RLDS 转换本身就可以继续：

```bash
python - <<'PY'
import tensorflow as tf
import tensorflow_datasets as tfds

print(tf.__version__)
print(tfds.__version__)
PY
```

### No episode_* directories found

说明 `/home/ustc/UR/dataset` 下还没有保存过 episode。需要先用
`recorder` 采集，并调用 `/recorder/save` 保存。

### Missing image

说明 `robot_states.json` 有某个 frame，但对应的 `cam_*/00000.jpg`
不存在。可以先用 `--no-images` 转纯状态数据，也可以回头检查采集过程。

### Camera folders differ

脚本要求所有 episode 的相机目录一致，例如都要有 `cam_0` 和 `cam_1`。
如果有的 episode 只有 `cam_0`，建议先把不完整 episode 移出去，或者统一采集配置。

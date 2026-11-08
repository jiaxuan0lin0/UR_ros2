# converter

Offline tool that converts recorder episode folders into a [TFDS](https://www.tensorflow.org/datasets)/[RLDS](https://github.com/google-research/rlds) dataset for policy training.

Pipeline: [teleop](../teleop/README.md) → [recorder](../recorder/README.md) → **converter** → [policy_client](../policy_client/README.md)

All of the logic lives in one script, `scripts/RLDS.py`. It does not need ROS and never talks to the robot. The colcon package has no entry points and does not install the script, so run it from the source tree. Workspace setup is in the [top-level README](../../README.md).

## How it works

1. Find the `episode_*` folders directly under `--dataset-root`. The search is not recursive. Folders are sorted by name, which is chronological because names are timestamps. `--max-episodes` keeps the first N.
2. Take the camera set from the first episode's `cam_*` folders. Every other episode must have the same set, unless `--no-images` is passed.
3. Validate **all** episodes (see [Validation](#validation)) and print a summary for each one. Nothing is written until every episode passes.
4. Unless `--inspect-only` is passed, build the TFDS dataset `ur_robot_rlds` version `1.0.0`. Each episode folder becomes one RLDS episode, and each record becomes one step. There is a single `train` split. By default the TCP rotation vectors are unwrapped within each episode.

Steps are not resampled. There is one step per recorded sample (nominally the recorder's `record_hz`), and the gaps the recorder leaves are kept. Use `timestamp` when you need real elapsed time.

## Setup

Use a separate virtual environment, in a shell where ROS is **not** sourced:

```bash
python3 -m venv <CONVERTER_VENV>          # e.g. src/converter/.venv
source <CONVERTER_VENV>/bin/activate
python -m pip install --upgrade pip
python -m pip install tensorflow tensorflow-datasets
```

`--inspect-only` imports only the standard library and works with any Python ≥ 3.9.

## Quick start

Always pass `--dataset-root` and `--output-dir`. The defaults in the script are hard-coded to a developer's home directory.

```bash
cd <UR_WORKSPACE>
source <CONVERTER_VENV>/bin/activate

# 1. Validate and summarize; writes nothing
python src/converter/scripts/RLDS.py --dataset-root <DATASET_DIR> --inspect-only

# 2. Convert
python src/converter/scripts/RLDS.py \
  --dataset-root <DATASET_DIR> \
  --output-dir <RLDS_DIR> \
  --language-instruction "<TASK_INSTRUCTION>" \
  --overwrite
```

`--inspect-only` prints one line per episode:

```text
episode_1710000000123: 412 frames, 13.700s, cameras=[cam_0, cam_1], images=824, statuses=[return_home=38, running=374]
Checked 1 episodes, 412 frames, 824 images.
```

Load the result:

```bash
python - <<'PY'
import tensorflow_datasets as tfds
ds = tfds.builder_from_directory("<RLDS_DIR>/ur_robot_rlds/1.0.0").as_dataset(split="train")
for episode in ds.take(1):
    print(episode["episode_metadata"])
    for step in episode["steps"].take(1):
        print(step["observation"].keys(), step["action"].keys())
PY
```

## Options

| Flag | Default | Description |
| --- | --- | --- |
| `--dataset-root` | hard-coded developer path | Folder that directly contains the `episode_*` folders. |
| `--output-dir` | hard-coded developer path | TFDS `data_dir`. The dataset is written to `<output-dir>/ur_robot_rlds/1.0.0/`. |
| `--state-file` | `robot_states.json` | Name of the JSON file inside each episode. |
| `--max-episodes N` | all | Use only the first N episodes, in name order. |
| `--language-instruction` | `""` | One string, stored on every step of every episode. |
| `--reward-mode` | `zero` | `zero`: every reward is 0.0. `final_one`: the last step gets 1.0 and the rest 0.0. |
| `--terminal-on-last` / `--no-terminal-on-last` | on | Whether the last step has `is_terminal=True`. |
| `--unwrap-tcp-rotvec` / `--no-unwrap-tcp-rotvec` | on | Unwrap the rotation vectors (see below). |
| `--no-images` | off | Ignore the `cam_*` folders. Image checks are skipped and `observation/images` is left out. |
| `--inspect-only` | off | Validate and summarize only. TensorFlow is not imported. |
| `--overwrite` | off | Delete `<output-dir>/ur_robot_rlds/1.0.0/` before writing. Without it, TFDS by default reuses an existing version folder and does not regenerate it. |

## Validation

Every check runs before anything is written. A failed check prints `ERROR: …` and exits with code 1.

| Check | Error |
| --- | --- |
| `--dataset-root` exists and is a directory | `Dataset root does not exist` / `is not a directory` |
| At least one `episode_*` folder | Prints `No episode_* directories found in …` and exits 0 |
| The state file exists and holds a non-empty JSON list | `Missing state file` / `must contain a JSON list` / `has no frames` |
| `frame_idx` equals the list index (0, 1, 2, …) | `frame i has frame_idx=…` |
| `timestamp` is a number | `bad timestamp` |
| `observation` and `action` each have a 6-number `joint_positions`, a 6-number `tcp_pose` and a numeric `gripper_position` | `field … must be a length-6 list` / `must be a number` |
| Every episode has the same `cam_*` set | `Camera folders differ in …` |
| `cam_N/<frame_idx:05d>.jpg` exists for every frame and camera | `Missing image: …` |

## Output format

```text
<RLDS_DIR>/
  ur_robot_rlds/
    1.0.0/
      dataset_info.json
      features.json
      ur_robot_rlds-train.tfrecord-00000-of-0000N   # one or more shards
```

Each episode is keyed by its folder name and has this metadata:

| Key | Type | Content |
| --- | --- | --- |
| `episode_metadata/episode_id` | text | Episode folder name, e.g. `episode_1710000000123` |
| `episode_metadata/file_path` | text | Episode folder path, built from `--dataset-root` |
| `episode_metadata/num_steps` | int64 | Number of steps |

Step fields. Units follow the [recorder format](../recorder/README.md#output-format): rad, m, gripper 0–1 where 1.0 is open with the teleop defaults.

| Key | Dtype | Shape | Content |
| --- | --- | --- | --- |
| `observation/joint_positions` | float32 | (6,) | Observed joint angles |
| `observation/tcp_pose` | float32 | (6,) | Observed `[x, y, z, rx, ry, rz]`. The rotation vector is unwrapped by default |
| `observation/gripper_position` | float32 | () | Observed gripper |
| `observation/state` | float32 | (13,) | `joint_positions` (6) + `tcp_pose` (6) + `gripper_position` (1) |
| `observation/images/cam_N` | uint8 | (H, W, 3) | JPEG-encoded RGB, one key per camera folder (480×640 with recorder defaults). Left out with `--no-images` |
| `action/joint_positions` | float32 | (6,) | Target joint angles |
| `action/tcp_pose` | float32 | (6,) | Target TCP pose (unwrapped by default) |
| `action/gripper_position` | float32 | () | Gripper target |
| `action/joint_action` | float32 | (7,) | `joint_positions` (6) + `gripper_position` (1) |
| `action/tcp_action` | float32 | (7,) | `tcp_pose` (6) + `gripper_position` (1) |
| `reward` | float32 | () | Set by `--reward-mode` |
| `discount` | float32 | () | 1.0, except 0.0 on the last step (regardless of `--terminal-on-last`) |
| `is_first` / `is_last` | bool | () | True on the first / last step |
| `is_terminal` | bool | () | True on the last step if `--terminal-on-last` |
| `timestamp` | float64 | () | Recorder wall-clock time (Unix seconds) |
| `frame_idx` | int64 | () | Index within the episode |
| `language_instruction` | text | () | `--language-instruction` |

The recorder's `status` and `control_mode` fields are not carried into RLDS. They appear only in the `--inspect-only` summary. The gripper action is the commanded target. If you need next-observation gripper labels instead, run the recorder's `fix_gripper_targets` on a copy of the dataset before converting.

## TCP rotation-vector unwrap

A UR TCP orientation is a rotation vector. Near a rotation angle of π its sign can flip from one frame to the next, a jump of about 2π, even though the rotation barely changed. With unwrapping on (the default), the converter keeps the first frame's value as it is. For each later frame it picks, among the equivalent vectors `r + 2πk·axis` (k within ±2 of the projected turn count), the one closest to the previous frame's value.

This is done separately for `observation.tcp_pose[3:]` and `action.tcp_pose[3:]` within each episode. The unwrapped values also flow into `observation/state` and `action/tcp_action`. The resulting vectors can have a norm above π but describe the same rotations. Pass `--no-unwrap-tcp-rotvec` to keep the raw values.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `RLDS conversion needs TensorFlow and TensorFlow Datasets` / `No module named 'tensorflow'` | Activate `<CONVERTER_VENV>`, or install the packages listed under [Setup](#setup). |
| pip reports ROS dependency conflicts | ROS is sourced in this shell. Open a fresh shell and activate only the venv. |
| `Dataset root does not exist` | The hard-coded default was used. Pass `--dataset-root`. |
| `No episode_* directories found` | `--dataset-root` must be the folder that directly contains the episodes. |
| `Missing state file` | The episode folder is left over from a recorder that was killed before save. Move it out of the dataset root. |
| `Missing image: …` | A camera dropped a frame. Move that episode out, or convert with `--no-images`. |
| `Camera folders differ in …` | Episodes were recorded with different numbers of cameras. Split them into separate dataset roots, or use `--no-images`. |
| `frame i has frame_idx=…` | `robot_states.json` was edited or merged by hand. `frame_idx` must run 0…N-1. |
| A re-run does not include new episodes | Add `--overwrite`. |

#!/usr/bin/env python3
"""Convert recorder episodes to a TFDS/RLDS dataset."""

import argparse
from collections import Counter
import json
import math
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


EPISODE_PREFIX = "episode_"
STATE_FILE_NAME = "robot_states.json"
CAMERA_DIR_PREFIX = "cam_"
DATASET_VERSION = "1.0.0"
TWO_PI = 2.0 * math.pi
ROTATION_EPS = 1.0e-12


class EpisodeFormatError(ValueError):
    """Raised when a recorder episode does not match the expected format."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert UR recorder data to TFDS/RLDS format.",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path.home() / "UR" / "dataset",
        help="Directory containing recorder episode_* folders.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "UR" / "rlds_dataset",
        help="TFDS data directory to write the converted dataset into.",
    )
    parser.add_argument(
        "--state-file",
        default=STATE_FILE_NAME,
        help="Per-episode JSON state filename.",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Only convert or inspect the first N episodes.",
    )
    parser.add_argument(
        "--language-instruction",
        default="",
        help="Optional language instruction saved on every RLDS step.",
    )
    parser.add_argument(
        "--reward-mode",
        choices=("zero", "final_one"),
        default="zero",
        help="How to fill the RLDS reward field.",
    )
    parser.add_argument(
        "--terminal-on-last",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Whether the final step should have is_terminal=True.",
    )
    parser.add_argument(
        "--no-images",
        action="store_true",
        help="Do not include cam_* image folders in the RLDS observations.",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="Only validate and summarize episodes; do not write RLDS files.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove the existing converted dataset version before writing.",
    )
    parser.add_argument(
        "--unwrap-tcp-rotvec",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Unwrap tcp_pose rotation-vector branches across each episode before "
            "writing RLDS. Enabled by default; use --no-unwrap-tcp-rotvec to keep "
            "raw UR rotvec values."
        ),
    )
    return parser.parse_args()


def find_episode_dirs(
    dataset_root: Path,
    max_episodes: Optional[int],
) -> List[Path]:
    """Find recorder episode directories under dataset_root."""
    if not dataset_root.exists():
        raise FileNotFoundError(f"Dataset root does not exist: {dataset_root}")
    if not dataset_root.is_dir():
        raise NotADirectoryError(f"Dataset root is not a directory: {dataset_root}")

    episode_dirs = sorted(
        path
        for path in dataset_root.iterdir()
        if path.is_dir() and path.name.startswith(EPISODE_PREFIX)
    )
    if max_episodes is not None:
        episode_dirs = episode_dirs[:max_episodes]
    return episode_dirs


def read_episode_records(episode_dir: Path, state_file: str) -> List[Dict[str, Any]]:
    """Load and validate one recorder robot_states.json file."""
    state_path = episode_dir / state_file
    if not state_path.exists():
        raise EpisodeFormatError(f"Missing state file: {state_path}")

    with state_path.open("r", encoding="utf-8") as f:
        records = json.load(f)

    if not isinstance(records, list):
        raise EpisodeFormatError(f"{state_path} must contain a JSON list")
    if not records:
        raise EpisodeFormatError(f"{state_path} has no frames")

    for index, record in enumerate(records):
        validate_record(record, index, state_path)
    return records


def validate_record(record: Dict[str, Any], index: int, state_path: Path) -> None:
    """Check one frame from robot_states.json."""
    if not isinstance(record, dict):
        raise EpisodeFormatError(f"{state_path}: frame {index} is not an object")

    frame_idx = record.get("frame_idx")
    if frame_idx != index:
        raise EpisodeFormatError(
            f"{state_path}: frame {index} has frame_idx={frame_idx}"
        )

    if not isinstance(record.get("timestamp"), (int, float)):
        raise EpisodeFormatError(f"{state_path}: frame {index} has bad timestamp")

    observation = record.get("observation")
    action = record.get("action")
    if not isinstance(observation, dict) or not isinstance(action, dict):
        raise EpisodeFormatError(
            f"{state_path}: frame {index} needs observation and action objects"
        )

    check_vector(observation, "joint_positions", 6, state_path, index)
    check_vector(observation, "tcp_pose", 6, state_path, index)
    check_number(observation, "gripper_position", state_path, index)

    check_vector(action, "joint_positions", 6, state_path, index)
    check_vector(action, "tcp_pose", 6, state_path, index)
    check_number(action, "gripper_position", state_path, index)


def check_vector(
    container: Dict[str, Any],
    key: str,
    expected_len: int,
    state_path: Path,
    index: int,
) -> None:
    value = container.get(key)
    if not isinstance(value, list) or len(value) != expected_len:
        raise EpisodeFormatError(
            f"{state_path}: frame {index} field {key} must be "
            f"a length-{expected_len} list"
        )
    if not all(isinstance(item, (int, float)) for item in value):
        raise EpisodeFormatError(
            f"{state_path}: frame {index} field {key} must contain numbers"
        )


def check_number(
    container: Dict[str, Any],
    key: str,
    state_path: Path,
    index: int,
) -> None:
    if not isinstance(container.get(key), (int, float)):
        raise EpisodeFormatError(
            f"{state_path}: frame {index} field {key} must be a number"
        )


def camera_dirs(episode_dir: Path) -> List[Path]:
    """Return cam_* folders inside one episode directory."""
    return sorted(
        path
        for path in episode_dir.iterdir()
        if path.is_dir() and path.name.startswith(CAMERA_DIR_PREFIX)
    )


def collect_camera_names(episode_dirs: Sequence[Path]) -> List[str]:
    """Collect camera names and require all episodes to use the same cameras."""
    if not episode_dirs:
        return []

    camera_names = [path.name for path in camera_dirs(episode_dirs[0])]
    expected = set(camera_names)

    for episode_dir in episode_dirs[1:]:
        current = {path.name for path in camera_dirs(episode_dir)}
        if current != expected:
            raise EpisodeFormatError(
                f"Camera folders differ in {episode_dir}. "
                f"Expected {sorted(expected)}, got {sorted(current)}"
            )

    return camera_names


def image_path_for(episode_dir: Path, camera_name: str, frame_idx: int) -> Path:
    return episode_dir / camera_name / f"{frame_idx:05d}.jpg"


def validate_images(
    episode_dir: Path,
    records: Sequence[Dict[str, Any]],
    camera_names: Sequence[str],
) -> int:
    """Check that every frame has an image for every selected camera."""
    image_count = 0
    for camera_name in camera_names:
        camera_dir = episode_dir / camera_name
        if not camera_dir.is_dir():
            raise EpisodeFormatError(f"Missing camera directory: {camera_dir}")

        for record in records:
            frame_idx = int(record["frame_idx"])
            image_path = image_path_for(episode_dir, camera_name, frame_idx)
            if not image_path.exists():
                raise EpisodeFormatError(f"Missing image: {image_path}")
            image_count += 1
    return image_count


def load_episode_summaries(
    episode_dirs: Sequence[Path],
    state_file: str,
    camera_names: Sequence[str],
) -> List[Tuple[Path, List[Dict[str, Any]], int]]:
    """Read all episodes once so validation errors appear before conversion."""
    summaries = []
    for episode_dir in episode_dirs:
        records = read_episode_records(episode_dir, state_file)
        image_count = validate_images(episode_dir, records, camera_names)
        summaries.append((episode_dir, records, image_count))
    return summaries


def print_summaries(
    summaries: Sequence[Tuple[Path, List[Dict[str, Any]], int]],
    camera_names: Sequence[str],
) -> None:
    total_frames = 0
    total_images = 0
    camera_label = ", ".join(camera_names) if camera_names else "none"

    for episode_dir, records, image_count in summaries:
        duration = records[-1]["timestamp"] - records[0]["timestamp"]
        status_counts = Counter(
            record["status"]
            if isinstance(record.get("status"), str) and record.get("status")
            else "unknown"
            for record in records
        )
        status_summary = ", ".join(
            f"{status}={count}"
            for status, count in sorted(status_counts.items())
        )
        total_frames += len(records)
        total_images += image_count
        print(
            f"{episode_dir.name}: {len(records)} frames, "
            f"{duration:.3f}s, cameras=[{camera_label}], "
            f"images={image_count}, statuses=[{status_summary}]"
        )

    print(
        f"Checked {len(summaries)} episodes, "
        f"{total_frames} frames, {total_images} images."
    )


def as_float_list(values: Sequence[Any]) -> List[float]:
    return [float(value) for value in values]


def squared_distance(left: Sequence[float], right: Sequence[float]) -> float:
    return sum((a - b) * (a - b) for a, b in zip(left, right))


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def nearest_equivalent_rotvec(
    rotvec: Sequence[float],
    reference: Sequence[float],
) -> List[float]:
    """Choose the 2pi-equivalent rotvec nearest to the previous unwrapped value."""
    current = [float(value) for value in rotvec]
    ref = [float(value) for value in reference]

    norm = vector_norm(current)
    if norm > ROTATION_EPS:
        axis = [value / norm for value in current]
    else:
        ref_norm = vector_norm(ref)
        if ref_norm <= ROTATION_EPS:
            return current
        axis = [value / ref_norm for value in ref]

    projected_turns = sum((ref[i] - current[i]) * axis[i] for i in range(3)) / TWO_PI
    center = int(round(projected_turns))
    candidates = [
        [current[i] + TWO_PI * turns * axis[i] for i in range(3)]
        for turns in range(center - 2, center + 3)
    ]
    return min(candidates, key=lambda candidate: squared_distance(candidate, ref))


def unwrap_rotvec_sequence(rotvecs: Sequence[Sequence[Any]]) -> List[List[float]]:
    """Make a sequence of UR rotation vectors continuous across +/-pi branches."""
    unwrapped = []
    previous = None
    for raw_rotvec in rotvecs:
        current = as_float_list(raw_rotvec)
        if previous is None:
            chosen = current
        else:
            chosen = nearest_equivalent_rotvec(current, previous)
        unwrapped.append(chosen)
        previous = chosen
    return unwrapped


def records_with_unwrapped_tcp_rotvecs(
    records: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Return records with observation/action tcp_pose rotvec branches unwrapped."""
    observation_rotvecs = [
        record["observation"]["tcp_pose"][3:]
        for record in records
    ]
    action_rotvecs = [
        record["action"]["tcp_pose"][3:]
        for record in records
    ]
    unwrapped_observation_rotvecs = unwrap_rotvec_sequence(observation_rotvecs)
    unwrapped_action_rotvecs = unwrap_rotvec_sequence(action_rotvecs)

    unwrapped_records = []
    for record, observation_rotvec, action_rotvec in zip(
        records,
        unwrapped_observation_rotvecs,
        unwrapped_action_rotvecs,
    ):
        observation = dict(record["observation"])
        action = dict(record["action"])

        observation_tcp_pose = as_float_list(observation["tcp_pose"])
        action_tcp_pose = as_float_list(action["tcp_pose"])
        observation["tcp_pose"] = observation_tcp_pose[:3] + observation_rotvec
        action["tcp_pose"] = action_tcp_pose[:3] + action_rotvec

        converted = dict(record)
        converted["observation"] = observation
        converted["action"] = action
        unwrapped_records.append(converted)
    return unwrapped_records


def observation_state(record: Dict[str, Any]) -> List[float]:
    observation = record["observation"]
    return (
        as_float_list(observation["joint_positions"])
        + as_float_list(observation["tcp_pose"])
        + [float(observation["gripper_position"])]
    )


def joint_action(record: Dict[str, Any]) -> List[float]:
    action = record["action"]
    return as_float_list(action["joint_positions"]) + [
        float(action["gripper_position"])
    ]


def tcp_action(record: Dict[str, Any]) -> List[float]:
    action = record["action"]
    return as_float_list(action["tcp_pose"]) + [float(action["gripper_position"])]


def reward_for_step(index: int, num_steps: int, reward_mode: str) -> float:
    if reward_mode == "final_one":
        return float(index == num_steps - 1)
    return 0.0


def import_tfds_modules():
    """Import heavy TFDS dependencies only when conversion is requested."""
    try:
        import numpy as np
        import tensorflow as tf
        import tensorflow_datasets as tfds
    except ImportError as exc:
        raise RuntimeError(
            "RLDS conversion needs TensorFlow and TensorFlow Datasets. "
            "Install them in this environment first, for example:\n"
            "  pip install tensorflow tensorflow-datasets"
        ) from exc
    return np, tf, tfds


def make_builder_class(
    np_module: Any,
    tfds_module: Any,
    version: str,
):
    """Create a TFDS DatasetBuilder class after TFDS has been imported."""

    class URRobotRlds(tfds_module.core.GeneratorBasedBuilder):
        """TFDS builder for UR recorder demonstrations."""

        VERSION = tfds_module.core.Version(version)
        RELEASE_NOTES = {
            version: "Initial conversion from UR recorder episodes.",
        }

        def __init__(
            self,
            *,
            episode_dirs: Sequence[Path],
            state_file: str,
            camera_names: Sequence[str],
            language_instruction: str,
            reward_mode: str,
            terminal_on_last: bool,
            unwrap_tcp_rotvec: bool,
            **kwargs: Any,
        ):
            self.episode_dirs = [Path(path) for path in episode_dirs]
            self.state_file = state_file
            self.camera_names = list(camera_names)
            self.language_instruction = language_instruction
            self.reward_mode = reward_mode
            self.terminal_on_last = terminal_on_last
            self.unwrap_tcp_rotvec = bool(unwrap_tcp_rotvec)
            super().__init__(**kwargs)

        def _info(self):
            image_features = {
                camera_name: tfds_module.features.Image(
                    shape=(None, None, 3),
                    dtype=np_module.uint8,
                    encoding_format="jpeg",
                    doc=f"RGB image from {camera_name}.",
                )
                for camera_name in self.camera_names
            }

            observation_features = {
                "joint_positions": tfds_module.features.Tensor(
                    shape=(6,),
                    dtype=np_module.float32,
                    doc="Observed UR joint positions in radians.",
                ),
                "tcp_pose": tfds_module.features.Tensor(
                    shape=(6,),
                    dtype=np_module.float32,
                    doc="Observed TCP pose [x, y, z, rx, ry, rz].",
                ),
                "gripper_position": tfds_module.features.Scalar(
                    dtype=np_module.float32,
                    doc="Observed gripper opening normalized to 0-1.",
                ),
                "state": tfds_module.features.Tensor(
                    shape=(13,),
                    dtype=np_module.float32,
                    doc="Concatenated joint positions, TCP pose, and gripper.",
                ),
            }
            if image_features:
                observation_features["images"] = tfds_module.features.FeaturesDict(
                    image_features
                )

            return self.dataset_info_from_configs(
                features=tfds_module.features.FeaturesDict(
                    {
                        "steps": tfds_module.features.Dataset(
                            {
                                "observation": (
                                    tfds_module.features.FeaturesDict(
                                        observation_features
                                    )
                                ),
                                "action": tfds_module.features.FeaturesDict(
                                    {
                                        "joint_positions": (
                                            tfds_module.features.Tensor(
                                                shape=(6,),
                                                dtype=np_module.float32,
                                                doc="Target joint positions.",
                                            )
                                        ),
                                        "tcp_pose": tfds_module.features.Tensor(
                                            shape=(6,),
                                            dtype=np_module.float32,
                                            doc="Target TCP pose.",
                                        ),
                                        "gripper_position": (
                                            tfds_module.features.Scalar(
                                                dtype=np_module.float32,
                                                doc="Target gripper position.",
                                            )
                                        ),
                                        "joint_action": (
                                            tfds_module.features.Tensor(
                                                shape=(7,),
                                                dtype=np_module.float32,
                                                doc="Joint target plus gripper.",
                                            )
                                        ),
                                        "tcp_action": tfds_module.features.Tensor(
                                            shape=(7,),
                                            dtype=np_module.float32,
                                            doc="TCP target plus gripper.",
                                        ),
                                    }
                                ),
                                "reward": tfds_module.features.Scalar(
                                    dtype=np_module.float32,
                                    doc="Reward filled by --reward-mode.",
                                ),
                                "discount": tfds_module.features.Scalar(
                                    dtype=np_module.float32,
                                    doc="1.0 except on the last step.",
                                ),
                                "is_first": tfds_module.features.Scalar(
                                    dtype=np_module.bool_,
                                    doc="True on the first step.",
                                ),
                                "is_last": tfds_module.features.Scalar(
                                    dtype=np_module.bool_,
                                    doc="True on the last step.",
                                ),
                                "is_terminal": tfds_module.features.Scalar(
                                    dtype=np_module.bool_,
                                    doc="True if the episode ended terminally.",
                                ),
                                "timestamp": tfds_module.features.Scalar(
                                    dtype=np_module.float64,
                                    doc="Wall-clock timestamp from recorder.",
                                ),
                                "frame_idx": tfds_module.features.Scalar(
                                    dtype=np_module.int64,
                                    doc="Frame index inside the episode.",
                                ),
                                "language_instruction": tfds_module.features.Text(
                                    doc="Optional task instruction.",
                                ),
                            }
                        ),
                        "episode_metadata": tfds_module.features.FeaturesDict(
                            {
                                "episode_id": tfds_module.features.Text(),
                                "file_path": tfds_module.features.Text(),
                                "num_steps": tfds_module.features.Scalar(
                                    dtype=np_module.int64
                                ),
                            }
                        ),
                    }
                ),
                homepage="https://github.com/google-research/rlds",
                description=(
                    "UR robot demonstrations converted from recorder "
                    "episode folders to the RLDS episode/step format."
                ),
            )

        def _split_generators(self, dl_manager):
            del dl_manager
            return {
                "train": self._generate_examples(self.episode_dirs),
            }

        def _generate_examples(self, episode_dirs):
            for episode_dir in episode_dirs:
                records = read_episode_records(episode_dir, self.state_file)
                if self.unwrap_tcp_rotvec:
                    records = records_with_unwrapped_tcp_rotvecs(records)
                steps = []
                num_steps = len(records)

                for index, record in enumerate(records):
                    is_last = index == num_steps - 1
                    observation = record["observation"]
                    action = record["action"]

                    observation_data = {
                        "joint_positions": as_float_list(
                            observation["joint_positions"]
                        ),
                        "tcp_pose": as_float_list(observation["tcp_pose"]),
                        "gripper_position": float(
                            observation["gripper_position"]
                        ),
                        "state": observation_state(record),
                    }
                    if self.camera_names:
                        observation_data["images"] = {
                            camera_name: str(
                                image_path_for(
                                    episode_dir,
                                    camera_name,
                                    int(record["frame_idx"]),
                                )
                            )
                            for camera_name in self.camera_names
                        }

                    steps.append(
                        {
                            "observation": observation_data,
                            "action": {
                                "joint_positions": as_float_list(
                                    action["joint_positions"]
                                ),
                                "tcp_pose": as_float_list(action["tcp_pose"]),
                                "gripper_position": float(
                                    action["gripper_position"]
                                ),
                                "joint_action": joint_action(record),
                                "tcp_action": tcp_action(record),
                            },
                            "reward": reward_for_step(
                                index,
                                num_steps,
                                self.reward_mode,
                            ),
                            "discount": 0.0 if is_last else 1.0,
                            "is_first": index == 0,
                            "is_last": is_last,
                            "is_terminal": bool(
                                is_last and self.terminal_on_last
                            ),
                            "timestamp": float(record["timestamp"]),
                            "frame_idx": int(record["frame_idx"]),
                            "language_instruction": self.language_instruction,
                        }
                    )

                yield episode_dir.name, {
                    "steps": steps,
                    "episode_metadata": {
                        "episode_id": episode_dir.name,
                        "file_path": str(episode_dir),
                        "num_steps": num_steps,
                    },
                }

    return URRobotRlds


def build_rlds_dataset(
    args: argparse.Namespace,
    episode_dirs: Sequence[Path],
    camera_names: Sequence[str],
) -> Path:
    np_module, _tf_module, tfds_module = import_tfds_modules()
    builder_class = make_builder_class(
        np_module=np_module,
        tfds_module=tfds_module,
        version=DATASET_VERSION,
    )
    builder = builder_class(
        data_dir=str(args.output_dir),
        episode_dirs=episode_dirs,
        state_file=args.state_file,
        camera_names=camera_names,
        language_instruction=args.language_instruction,
        reward_mode=args.reward_mode,
        terminal_on_last=args.terminal_on_last,
        unwrap_tcp_rotvec=args.unwrap_tcp_rotvec,
    )

    dataset_path = Path(str(builder.data_dir)) / builder.name / str(builder.version)
    if args.overwrite and dataset_path.exists():
        shutil.rmtree(dataset_path)

    builder.download_and_prepare()
    return dataset_path


def main() -> int:
    try:
        args = parse_args()
        episode_dirs = find_episode_dirs(args.dataset_root, args.max_episodes)

        if not episode_dirs:
            print(f"No {EPISODE_PREFIX}* directories found in {args.dataset_root}")
            return 0

        camera_names = [] if args.no_images else collect_camera_names(episode_dirs)
        summaries = load_episode_summaries(
            episode_dirs=episode_dirs,
            state_file=args.state_file,
            camera_names=camera_names,
        )
        print_summaries(summaries, camera_names)

        if args.inspect_only:
            print("Inspect-only mode finished; no RLDS files were written.")
            return 0

        dataset_path = build_rlds_dataset(args, episode_dirs, camera_names)
        print(f"Wrote RLDS dataset to: {dataset_path}")
        print("Load it with:")
        print(f"  tfds.builder_from_directory('{dataset_path}').as_dataset()")
        return 0
    except (
        EpisodeFormatError,
        FileNotFoundError,
        NotADirectoryError,
        RuntimeError,
    ) as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Convert recorded episodes to next-observation gripper target semantics."""

import argparse
import json
from pathlib import Path

from recorder.config import DATASET_ROOT, STATE_FILE_NAME
from recorder.gripper_target_alignment import (
    align_gripper_targets_with_next_observation,
)


def _iter_state_files(dataset_root: Path):
    yield from sorted(dataset_root.rglob(STATE_FILE_NAME))


def _rewrite_json(path: Path, records) -> None:
    temp_path = path.with_suffix(path.suffix + ".tmp")
    with open(temp_path, "w") as f:
        json.dump(records, f, indent=4)
        f.write("\n")
    temp_path.replace(path)


def repair_dataset(dataset_root: Path, dry_run: bool = False) -> tuple[int, int, int]:
    files_scanned = 0
    files_modified = 0
    records_updated = 0

    for state_file in _iter_state_files(dataset_root):
        files_scanned += 1
        with open(state_file, "r") as f:
            records = json.load(f)

        updated = align_gripper_targets_with_next_observation(records)
        if updated <= 0:
            continue

        files_modified += 1
        records_updated += updated
        if not dry_run:
            _rewrite_json(state_file, records)

    return files_scanned, files_modified, records_updated


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert each action.gripper_position to the next frame's "
            "observation.gripper_position. This is only for next-state "
            "prediction datasets, not command-target control datasets."
        )
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=DATASET_ROOT,
        help=f"dataset root to scan (default: {DATASET_ROOT})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report how many files would change without rewriting them",
    )
    args = parser.parse_args()

    files_scanned, files_modified, records_updated = repair_dataset(
        dataset_root=args.dataset_root,
        dry_run=args.dry_run,
    )
    mode = "Would update" if args.dry_run else "Updated"
    print(
        f"{mode} {records_updated} records across "
        f"{files_modified}/{files_scanned} files under {args.dataset_root}"
    )


if __name__ == "__main__":
    main()

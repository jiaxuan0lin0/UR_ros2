"""Optional conversion utilities for next-observation gripper targets."""


def _as_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _gripper_position(record: dict, key: str):
    if not isinstance(record, dict):
        return None
    payload = record.get(key)
    if not isinstance(payload, dict):
        return None
    return _as_float(payload.get("gripper_position"))


def align_gripper_targets_with_next_observation(records) -> int:
    """Replace each action gripper target with the next frame observation.

    The last frame falls back to its own observation because there is no next
    frame available at episode end.
    """

    if not isinstance(records, list):
        return 0

    updates = 0
    total = len(records)
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            continue

        action = record.get("action")
        if not isinstance(action, dict):
            continue

        next_observation = None
        if index + 1 < total:
            next_observation = _gripper_position(records[index + 1], "observation")
        if next_observation is None:
            next_observation = _gripper_position(record, "observation")
        if next_observation is None:
            continue

        previous = _as_float(action.get("gripper_position"))
        if previous != next_observation:
            updates += 1
        action["gripper_position"] = next_observation

    return updates

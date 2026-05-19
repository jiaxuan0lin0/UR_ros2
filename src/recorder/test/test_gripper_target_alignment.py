from recorder.gripper_target_alignment import (
    align_gripper_targets_with_next_observation,
)


def test_align_gripper_targets_with_next_observation():
    records = [
        {
            "observation": {"gripper_position": 0.10},
            "action": {"gripper_position": 1.00},
        },
        {
            "observation": {"gripper_position": 0.35},
            "action": {"gripper_position": 0.00},
        },
        {
            "observation": {"gripper_position": 0.80},
            "action": {"gripper_position": 0.00},
        },
    ]

    updated = align_gripper_targets_with_next_observation(records)

    assert updated == 3
    assert [record["action"]["gripper_position"] for record in records] == [
        0.35,
        0.80,
        0.80,
    ]

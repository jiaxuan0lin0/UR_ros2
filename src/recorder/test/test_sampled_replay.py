import pytest

from recorder.sampled_replay import build_sampled_replay_payload


def test_build_sampled_replay_payload_uses_sampled_action_gripper_command():
    records = [
        {
            "timestamp": 10.0,
            "status": "running",
            "observation": {
                "gripper_position": 0.1,
            },
            "action": {
                "joint_positions": [0.0] * 6,
                "tcp_pose": [0.0] * 6,
                "gripper_position": 0.9,
            },
        },
        {
            "timestamp": 10.1,
            "status": "return_home",
            "observation": {
                "gripper_position": 0.4,
            },
            "action": {
                "joint_positions": [1.0] * 6,
                "tcp_pose": [1.0] * 6,
                "gripper_position": 0.2,
            },
        },
    ]

    payload = build_sampled_replay_payload(records)

    assert payload["point_count"] == 2
    assert payload["points"][0]["time"] == 0.0
    assert payload["points"][1]["time"] == pytest.approx(0.1)
    assert payload["points"][0]["phase"] == "running"
    assert payload["points"][1]["phase"] == "return_home"
    assert payload["points"][0]["q"] == [0.0] * 6
    assert payload["points"][1]["q"] == [1.0] * 6
    assert payload["points"][0]["gripper_raw"] == 900
    assert payload["points"][1]["gripper_raw"] == 200
    assert payload["points"][0]["gripper_target_raw"] == 900
    assert payload["points"][1]["gripper_target_raw"] == 200
    assert payload["points"][0]["gripper_observation_raw"] == 100
    assert payload["points"][1]["gripper_observation_raw"] == 400

import numpy as np

from policy_client.controller_clients import (
    _apply_joint_step_limit,
    _build_servo_path,
    _pchip_tangents,
)


def _point(value: float, time_from_start: float):
    return (
        np.full(6, value, dtype=np.float64),
        0.0,
        float(time_from_start),
    )


def test_servo_path_uses_global_500hz_timing_for_30hz_chunk():
    points = [_point(float(index + 1), float(index + 1) / 30.0) for index in range(50)]

    sample_times, samples = _build_servo_path(
        points,
        start_q=np.zeros(6, dtype=np.float64),
        servo_time=0.002,
        interpolation_mode="pchip",
    )

    assert len(sample_times) == 833
    assert samples.shape == (833, 6)
    np.testing.assert_allclose(sample_times[-1], 50.0 / 30.0)
    np.testing.assert_allclose(samples[-1], points[-1][0])


def test_pchip_tangents_stop_at_direction_reversal_without_overshoot():
    knot_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    knots = np.asarray(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    tangents = _pchip_tangents(knot_times, knots)

    np.testing.assert_allclose(tangents[1], np.zeros(6, dtype=np.float64))

    _, samples = _build_servo_path(
        [_point(1.0, 1.0), _point(0.0, 2.0)],
        start_q=np.zeros(6, dtype=np.float64),
        servo_time=0.05,
        interpolation_mode="pchip",
    )
    assert np.min(samples) >= -1.0e-12
    assert np.max(samples) <= 1.0 + 1.0e-12


def test_joint_step_limit_preserves_500hz_sample_count_and_caps_steps():
    raw_samples = np.asarray(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    limited, limited_count, max_raw_step = _apply_joint_step_limit(
        raw_samples,
        start_q=np.zeros(6, dtype=np.float64),
        max_joint_delta=0.25,
    )

    assert limited_count == 2
    assert max_raw_step == 1.75
    assert len(limited) == len(raw_samples)
    step_norms = np.linalg.norm(
        np.diff(np.vstack([np.zeros((1, 6), dtype=np.float64), limited]), axis=0),
        axis=1,
    )
    assert np.max(step_norms) <= 0.25 + 1.0e-12

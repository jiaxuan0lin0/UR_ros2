import numpy as np

from policy_client.rtc_control import RTCControl, RTCControlConfig


def test_rtc_control_blends_new_chunk_prefix_with_previous_tail():
    rtc = RTCControl(
        RTCControlConfig(
            enabled=True,
            execution_horizon=2,
            schedule="linear",
        )
    )
    previous = [
        np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.asarray([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        np.asarray([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]),
    ]
    rtc.commit(previous)

    new = [
        np.asarray([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]),
        np.asarray([20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]),
        np.asarray([30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]),
    ]

    aligned, info = rtc.align(
        new,
        action_period_sec=0.1,
        inference_latency_sec=0.25,
    )

    assert info.prefix_count == 2
    assert info.inference_delay_steps == 3
    np.testing.assert_allclose(aligned[0], previous[-2])
    np.testing.assert_allclose(aligned[1], new[1])
    np.testing.assert_allclose(aligned[2], new[2])


def test_rtc_control_disabled_returns_actions_unchanged():
    rtc = RTCControl(RTCControlConfig(enabled=False, execution_horizon=2))
    rtc.commit([np.ones(7)])
    new = [np.arange(7, dtype=np.float64)]

    aligned, info = rtc.align(new, action_period_sec=0.1)

    assert not info.enabled
    np.testing.assert_allclose(aligned[0], new[0])

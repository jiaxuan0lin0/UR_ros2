"""Log text for policy action chunks, observations and RTC alignment."""

from __future__ import annotations

from typing import Optional

import numpy as np


def format_action_summary(
    actions: list[np.ndarray],
    result,
    observation: Optional[object] = None,
) -> str:
    raw_count = int(
        getattr(result.action_chunk, 'shape', [len(actions)])[0]
    )
    obs_summary = format_observation_summary(observation)
    if not actions:
        return (
            f'policy action chunk: raw_actions={raw_count}, '
            f'selected_actions=0{obs_summary}'
        )
    first = np.array2string(actions[0], precision=4)
    last = np.array2string(actions[-1], precision=4)
    return (
        f'policy action chunk: raw_actions={raw_count}, '
        f'selected_actions={len(actions)}{obs_summary}, '
        f'first={first}, last={last}'
    )


def format_observation_summary(observation: Optional[object]) -> str:
    if observation is None:
        return ''
    proprio = getattr(observation, 'proprio', None)
    if proprio is None:
        return ''
    values = np.asarray(proprio, dtype=np.float64)
    if values.shape[0] < 7:
        return ''
    first3 = np.array2string(values[:3], precision=4)
    return f', proprio0_2={first3}, gripper={values[6]:.3f}'


def format_rtc_summary(info) -> str:
    if info is None or not getattr(info, 'enabled', False):
        return ''
    return (
        f"rtc_prefix={info.prefix_count}, "
        f"rtc_prev={info.previous_count}, "
        f"rtc_delay_steps={info.inference_delay_steps}, "
        f"rtc_schedule={info.schedule}"
    )
